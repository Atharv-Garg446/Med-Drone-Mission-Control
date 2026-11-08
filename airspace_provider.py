"""
airspace_provider.py
====================
Airspace-data adapter layer between external airspace sources and the
planner's polygon list (``CityConfig.no_fly_zones`` / A* / mission validation).

Every snapshot records WHERE the data came from, WHEN it was retrieved, when
the source says it was last updated (if it says), a content hash, and whether
it is stale. Planning code asks for polygons through
``AirspaceSnapshot.planning_polygons()`` so category policy (e.g. treating
yellow zones as blocked unless ATC permission is held) lives in one place.

Zone categories
---------------
  red                    -- published no-drone zone (Digital Sky "red")
  yellow                 -- published controlled zone, needs ATC permission
  green                  -- published permitted zone (never blocks)
  temporary_restriction  -- a time-bounded restriction *supplied by a source*
  scenario               -- synthetic/test geometry (never official)

Do not label scenario geometry as a TFR or as official data.

Providers
---------
  GeoJSONFileProvider        -- polygons from a GeoJSON file the operator is
                                authorised to use (e.g. an export from an
                                authorised source). Fully implemented.
  ScenarioProvider           -- wraps the polygons in config.py scenarios,
                                labelled ``scenario``.
  DigitalSkyIntersectionChecker -- see docs/data_sources_and_provenance.md. The Digital Sky
                                public interface checks flight-plan
                                intersection only; it requires a pilot token
                                and the officially published request schema.
                                Without both, it reports UNAVAILABLE rather
                                than guessing at an undocumented protocol.
  CachedAirspaceProvider     -- last-known-good fallback with explicit
                                staleness on outages.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]

CATEGORIES = ("red", "yellow", "green", "temporary_restriction", "scenario")
DEFAULT_BLOCKING = ("red", "yellow", "temporary_restriction", "scenario")

METERS_TO_FEET = 3.280839895
DIGITALSKY_MAX_ALTITUDE_FT = 400.0
DIGITALSKY_VALIDATE_URL = "https://digitalsky.dgca.gov.in/digital-sky/flightplan/validate/airspace"


class SourceClassification:
    """Explicit authority & provenance classification across all airspace/context layers."""

    LIVE_EXTERNAL = "LIVE_EXTERNAL"
    OFFICIAL_API_VALIDATION = "OFFICIAL_API_VALIDATION"
    MAP_VISUALIZATION_ONLY = "MAP_VISUALIZATION_ONLY"
    SIMULATED_SENSOR = "SIMULATED_SENSOR"
    CACHED = "CACHED"
    SYNTHETIC_FIXTURE = "SYNTHETIC_FIXTURE"



class AirspaceUnavailable(RuntimeError):
    """The source could not be reached / is not configured."""


class AirspaceDataStale(RuntimeError):
    """Only data older than the permitted age is available."""


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(dt: Optional[datetime]) -> Optional[str]:
    return dt.isoformat() if dt else None


@dataclass
class AirspaceZone:
    zone_id: str
    category: str
    polygon: List[Point]                      # (lat, lon), implicitly closed
    name: str = ""
    floor_m_agl: Optional[float] = None
    ceiling_m_agl: Optional[float] = None
    valid_from: Optional[datetime] = None
    valid_to: Optional[datetime] = None
    properties: Dict = field(default_factory=dict)

    def __post_init__(self):
        if self.category not in CATEGORIES:
            raise ValueError("unknown airspace category %r" % self.category)
        if len(self.polygon) < 3:
            raise ValueError("zone %s has fewer than 3 vertices" % self.zone_id)
        for lat, lon in self.polygon:
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                raise ValueError("zone %s has out-of-range vertex %s" % (self.zone_id, (lat, lon)))

    def active_at(self, when: datetime) -> bool:
        if self.valid_from and when < self.valid_from:
            return False
        if self.valid_to and when >= self.valid_to:
            return False
        return True

    def altitude_conflicts(self, alt_m_agl: Optional[float]) -> bool:
        """True if the given altitude (m AGL) penetrates the zone's vertical limits."""
        if alt_m_agl is None:
            return True  # If unspecified, conservatively assume altitude conflict
        if self.floor_m_agl is not None and alt_m_agl < self.floor_m_agl:
            return False
        if self.ceiling_m_agl is not None and alt_m_agl > self.ceiling_m_agl:
            return False
        return True

    def time_conflicts(self, start_dt: Optional[datetime], end_dt: Optional[datetime] = None) -> bool:
        """True if the flight interval [start_dt, end_dt] overlaps the restriction's validity."""
        if not self.valid_from and not self.valid_to:
            return True  # Permanent restriction
        if start_dt is None:
            return True
        end_dt = end_dt or start_dt
        if self.valid_from and end_dt < self.valid_from:
            return False
        if self.valid_to and start_dt >= self.valid_to:
            return False
        return True


@dataclass
class AirspaceSnapshot:
    provider: str
    source: str                                # URL / file path / description
    zones: List[AirspaceZone]
    retrieved_at: datetime
    source_updated_at: Optional[datetime] = None
    content_sha256: Optional[str] = None
    version: Optional[str] = None
    official: bool = False
    stale: bool = False
    error: Optional[str] = None
    notes: str = ""

    def age_s(self, now: Optional[datetime] = None) -> float:
        return ((now or _utc_now()) - self.retrieved_at).total_seconds()

    def planning_polygons(self, blocking: Sequence[str] = DEFAULT_BLOCKING,
                          when: Optional[datetime] = None) -> List[List[Point]]:
        when = when or _utc_now()
        return [list(z.polygon) for z in self.zones
                if z.category in blocking and z.active_at(when)]

    def active_zones_for_flight(
        self,
        blocking: Sequence[str] = DEFAULT_BLOCKING,
        alt_m_agl: Optional[float] = None,
        start_dt: Optional[datetime] = None,
        end_dt: Optional[datetime] = None,
    ) -> List[AirspaceZone]:
        """Filter zones that conflict in category, altitude, and flight time interval."""
        return [
            z for z in self.zones
            if z.category in blocking
            and z.altitude_conflicts(alt_m_agl)
            and z.time_conflicts(start_dt, end_dt)
        ]

    def planning_polygons_4d(
        self,
        blocking: Sequence[str] = DEFAULT_BLOCKING,
        alt_m_agl: Optional[float] = None,
        start_dt: Optional[datetime] = None,
        end_dt: Optional[datetime] = None,
    ) -> List[List[Point]]:
        """Polygons of all zones that conflict in category, altitude, and flight time window."""
        return [list(z.polygon) for z in self.active_zones_for_flight(blocking, alt_m_agl, start_dt, end_dt)]

    @property
    def provenance_classification(self) -> str:
        if self.stale:
            return SourceClassification.CACHED
        if self.official:
            return SourceClassification.OFFICIAL_API_VALIDATION
        if self.provider == "scenario_config" or (
            self.zones and all(z.category == "scenario" for z in self.zones)
        ):
            return SourceClassification.SYNTHETIC_FIXTURE
        return SourceClassification.LIVE_EXTERNAL

    def metadata(self) -> dict:
        counts: Dict[str, int] = {}
        for z in self.zones:
            counts[z.category] = counts.get(z.category, 0) + 1
        return {"provider": self.provider, "source": self.source, "official": self.official,
                "provenance_classification": self.provenance_classification,
                "retrieved_at": _iso(self.retrieved_at),
                "source_updated_at": _iso(self.source_updated_at),
                "content_sha256": self.content_sha256, "version": self.version,
                "stale": self.stale, "error": self.error, "zone_counts": counts,
                "notes": self.notes}


# ---------------------------------------------------------------------------
# GeoJSON ingestion
# ---------------------------------------------------------------------------
def _parse_time(v) -> Optional[datetime]:
    if not v:
        return None
    dt = datetime.fromisoformat(str(v).replace("Z", "+00:00"))
    return dt if dt.tzinfo else dt.replace(tzinfo=timezone.utc)


def _rings(geometry: dict) -> List[List[Point]]:
    """Outer rings of Polygon / MultiPolygon, converted (lon,lat)->(lat,lon).
    Holes are ignored, which is conservative (more area blocked)."""
    gtype = geometry.get("type")
    if gtype == "Polygon":
        polys = [geometry["coordinates"]]
    elif gtype == "MultiPolygon":
        polys = geometry["coordinates"]
    else:
        return []
    out = []
    for poly in polys:
        ring = [(float(c[1]), float(c[0])) for c in poly[0]]
        if len(ring) > 1 and ring[0] == ring[-1]:
            ring = ring[:-1]
        out.append(ring)
    return out


def zones_from_geojson(doc: dict, category_property: str = "category",
                       default_category: Optional[str] = None,
                       category_map: Optional[Dict[str, str]] = None) -> List[AirspaceZone]:
    if doc.get("type") != "FeatureCollection":
        raise ValueError("expected a GeoJSON FeatureCollection")
    category_map = {k.lower(): v for k, v in (category_map or {}).items()}
    zones = []
    for i, feat in enumerate(doc.get("features", [])):
        props = feat.get("properties") or {}
        raw = props.get(category_property, default_category)
        if raw is None:
            raise ValueError("feature %d has no %r property" % (i, category_property))
        cat = category_map.get(str(raw).lower(), str(raw).lower())
        rings = _rings(feat.get("geometry") or {})
        for j, ring in enumerate(rings):
            zid = str(props.get("id", feat.get("id", i)))
            zones.append(AirspaceZone(
                zone_id=zid if len(rings) == 1 else "%s.%d" % (zid, j),
                category=cat, polygon=ring, name=str(props.get("name", "")),
                floor_m_agl=props.get("floor_m_agl"), ceiling_m_agl=props.get("ceiling_m_agl"),
                valid_from=_parse_time(props.get("valid_from")),
                valid_to=_parse_time(props.get("valid_to")),
                properties=props))
    return zones


class GeoJSONFileProvider:
    def __init__(self, path: str, provider_name: str, official: bool = False,
                 category_property: str = "category", category_map: Optional[Dict] = None,
                 notes: str = ""):
        self.path = path
        self.provider_name = provider_name
        self.official = official
        self.category_property = category_property
        self.category_map = category_map
        self.notes = notes

    def fetch(self) -> AirspaceSnapshot:
        try:
            with open(self.path, "rb") as f:
                raw = f.read()
        except OSError as e:
            raise AirspaceUnavailable("cannot read %s: %s" % (self.path, e))
        doc = json.loads(raw)
        meta = doc.get("metadata") or {}
        updated = _parse_time(meta.get("updated_at")) or datetime.fromtimestamp(
            os.path.getmtime(self.path), timezone.utc)
        return AirspaceSnapshot(
            provider=self.provider_name, source=os.path.abspath(self.path),
            zones=zones_from_geojson(doc, self.category_property, category_map=self.category_map),
            retrieved_at=_utc_now(), source_updated_at=updated,
            content_sha256=hashlib.sha256(raw).hexdigest(), version=meta.get("version"),
            official=self.official, notes=self.notes or meta.get("notes", ""))


class ScenarioProvider:
    """The hand-made polygons in config.py, explicitly labelled non-official."""

    def __init__(self, city, name: str = "scenario_config"):
        self.city = city
        self.name = name

    def fetch(self) -> AirspaceSnapshot:
        zones = [AirspaceZone("%s-%d" % (self.name, i), "scenario", [tuple(p) for p in z],
                              name="scenario zone %d" % i)
                 for i, z in enumerate(self.city.no_fly_zones)]
        blob = json.dumps([z.polygon for z in zones]).encode()
        return AirspaceSnapshot(provider=self.name, source="config.py:%s" % self.city.name,
                                zones=zones, retrieved_at=_utc_now(),
                                content_sha256=hashlib.sha256(blob).hexdigest(),
                                official=False,
                                notes="simplified stand-in polygons; not Digital Sky data")


class CachedAirspaceProvider:
    """Last-known-good cache with explicit staleness.

    * fetch succeeds         -> fresh snapshot, cached
    * fetch fails, cache ok  -> cached snapshot with stale=True and error set,
                                if younger than ``max_age_s``
    * otherwise              -> AirspaceDataStale / AirspaceUnavailable
    """

    def __init__(self, inner, max_age_s: float = 6 * 3600, clock=_utc_now):
        self.inner = inner
        self.max_age_s = max_age_s
        self.clock = clock
        self.last_good: Optional[AirspaceSnapshot] = None

    def fetch(self) -> AirspaceSnapshot:
        try:
            snap = self.inner.fetch()
            self.last_good = snap
            return snap
        except (AirspaceUnavailable, OSError, ValueError) as e:
            if self.last_good is None:
                raise AirspaceUnavailable("no airspace data: %s" % e)
            age = self.last_good.age_s(self.clock())
            if age > self.max_age_s:
                raise AirspaceDataStale("last good airspace data is %.0f s old (limit %.0f s); "
                                        "source error: %s" % (age, self.max_age_s, e))
            s = self.last_good
            return AirspaceSnapshot(**{**s.__dict__, "stale": True, "error": str(e)})


# ---------------------------------------------------------------------------
# Digital Sky flight-plan intersection check (credential + spec gated)
# ---------------------------------------------------------------------------
@dataclass
class IntersectionResult:
    checked: bool
    intersects: Optional[bool]
    provider: str
    checked_at: datetime
    detail: str = ""
    raw: Optional[dict] = None
    official_clearance: bool = False
    provenance_classification: str = SourceClassification.OFFICIAL_API_VALIDATION


def build_digitalsky_validate_request(
    polygon: Sequence[Point],
    floor_ft: float = 0.0,
    ceiling_ft: float = 400.0,
    start: Optional[datetime] = None,
    end: Optional[datetime] = None,
    is_circle: bool = False,
    radius_m: Optional[float] = None,
    source_app: str = "med-drone-mission-control",
    correlation_id: Optional[str] = None,
) -> dict:
    """Build the genuine DigitalSky OpenAPI 3.0.0 FlightPlanAreaValidateRequest payload & headers.

    Verified against docs/external_specs/digitalsky_interactive_maps_api_v1.0.yml:
      - Endpoint: POST https://digitalsky.dgca.gov.in/digital-sky/flightplan/validate/airspace
      - Headers: X-API-VERSION=1, source=<source_app>, x-auth=<token>, x-correlation-id=<uuid1>
      - Body: FlightPlanAreaValidateRequest (maxAltitude <= 400 ft, minAltitude, startDateTime,
        endDateTime, base64-encoded GeoJSON FeatureCollection in geom, isCircle, optional radius).
    """
    if len(polygon) < 3:
        raise ValueError("DigitalSky flight plan validation requires at least 3 polygon vertices")
    if ceiling_ft > DIGITALSKY_MAX_ALTITUDE_FT:
        raise ValueError(
            f"DigitalSky maxAltitude ({ceiling_ft:.1f} ft) exceeds OpenAPI maximum of "
            f"{DIGITALSKY_MAX_ALTITUDE_FT:.0f} ft (121.92 m AGL)"
        )
    if floor_ft < 0.0 or floor_ft > ceiling_ft:
        raise ValueError(f"Invalid altitude bounds: minAltitude={floor_ft}, maxAltitude={ceiling_ft}")
    if is_circle and (radius_m is None or radius_m <= 0):
        raise ValueError("DigitalSky radius is mandatory when isCircle is True")

    # RFC 7946 GeoJSON Polygon ring uses [lon, lat] and must be closed
    ring = [[float(lon), float(lat)] for lat, lon in polygon]
    if ring[0] != ring[-1]:
        ring.append(list(ring[0]))

    geojson_fc = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [ring],
                },
            }
        ],
    }
    geom_b64 = base64.b64encode(
        json.dumps(geojson_fc, separators=(",", ":")).encode("utf-8")
    ).decode("ascii")

    now = _utc_now()
    start_dt = start or now
    end_dt = end or start_dt

    body: Dict = {
        "maxAltitude": float(round(ceiling_ft, 2)),
        "minAltitude": float(round(floor_ft, 2)),
        "startDateTime": _iso(start_dt),
        "endDateTime": _iso(end_dt),
        "geom": geom_b64,
        "isCircle": bool(is_circle),
    }
    if is_circle and radius_m is not None:
        body["radius"] = float(radius_m)

    headers = {
        "X-API-VERSION": "1",
        "source": str(source_app),
        "x-correlation-id": str(correlation_id or uuid.uuid1()),
        "Content-Type": "application/json",
    }
    return {
        "url": DIGITALSKY_VALIDATE_URL,
        "headers": headers,
        "body": body,
        "decoded_geojson": geojson_fc,
    }


def parse_digitalsky_error_payload(raw_bytes: bytes, http_status: int) -> str:
    """Parse DigitalSky OpenAPI 3.0 Error and HAL ValidationError response schemas.

    Supports both:
      - Error: {"code": "...", "message": "..."}
      - ValidationError: {"message": "...", "_links": {...}, "_embeded": {"errors": [{"message": "..."}]}}
    """
    if not raw_bytes:
        return f"HTTP {http_status} (empty body)"
    try:
        doc = json.loads(raw_bytes.decode("utf-8", errors="replace"))
    except json.JSONDecodeError:
        return f"HTTP {http_status}: {raw_bytes.decode('utf-8', errors='replace')[:200]}"
    if not isinstance(doc, dict):
        return f"HTTP {http_status}: {doc}"
    code = doc.get("code")
    msg = doc.get("message") or ""
    embedded = doc.get("_embeded") or doc.get("_embedded") or {}
    sub_errors = []
    if isinstance(embedded, dict):
        for err_item in embedded.get("errors") or []:
            if isinstance(err_item, dict) and err_item.get("message"):
                sub_errors.append(str(err_item["message"]))
    parts = [f"HTTP {http_status}"]
    if code:
        parts.append(f"code={code}")
    if msg:
        parts.append(str(msg))
    if sub_errors:
        parts.append("validation_errors=[" + "; ".join(sub_errors) + "]")
    return ": ".join(parts)


def default_digitalsky_http_transport(
    token: str,
    polygon: Sequence[Point],
    floor_ft: float = 0.0,
    ceiling_ft: float = 400.0,
    start: Optional[datetime] = None,
    end: Optional[datetime] = None,
    timeout_s: float = 10.0,
    url: str = DIGITALSKY_VALIDATE_URL,
    source_app: str = "med-drone-mission-control",
    opener=None,
) -> dict:
    """Official DigitalSky OpenAPI 3.0.0 HTTP transport for /flightplan/validate/airspace."""
    spec = build_digitalsky_validate_request(
        polygon=polygon,
        floor_ft=floor_ft,
        ceiling_ft=ceiling_ft,
        start=start,
        end=end,
        source_app=source_app,
    )
    headers = dict(spec["headers"])
    headers["x-auth"] = token
    payload = json.dumps(spec["body"]).encode("utf-8")
    req = urllib.request.Request(url, data=payload, headers=headers, method="POST")
    _open = opener or urllib.request.urlopen
    try:
        with _open(req, timeout=timeout_s) as resp:
            raw_bytes = resp.read()
            resp_headers = getattr(resp, "headers", {}) or {}
            corr_id = (
                resp_headers.get("x-correlation-id")
                if hasattr(resp_headers, "get")
                else spec["headers"]["x-correlation-id"]
            )
            data = json.loads(raw_bytes.decode("utf-8"))
            if isinstance(data, dict):
                if corr_id and "x_correlation_id" not in data:
                    data["x_correlation_id"] = corr_id
                if "intersects" not in data:
                    red_zones = data.get("redZones") or data.get("red") or []
                    yellow_zones = data.get("yellowZones") or data.get("yellow") or []
                    status_str = str(data.get("status") or data.get("zoneStatus") or "").upper()
                    intersects = bool(
                        red_zones or yellow_zones or status_str in ("RED", "YELLOW", "INTERSECTS")
                    )
                    data["intersects"] = intersects
                data["operational_approval_granted"] = False
                data["regulatory_note"] = (
                    "Green-zone intersection check validates airspace geometry only; "
                    "it does NOT constitute blanket operational flight approval."
                )
            return data
    except urllib.error.HTTPError as http_err:
        err_body = b""
        try:
            err_body = http_err.read()
        except Exception:
            pass
        detail = parse_digitalsky_error_payload(err_body, http_err.code)
        raise AirspaceUnavailable(f"DigitalSky API error ({detail})") from http_err


class DigitalSkyIntersectionChecker:
    """Credential- and spec-gated Digital Sky flight-plan intersection check.

    Implements the genuine OpenAPI 3.0.0 specification retrieved from
    ``https://digitalsky.aai.aero/Interactive-Maps-API-Swagger-Documentation-v1.0.yml``
    (saved in ``docs/external_specs/digitalsky_interactive_maps_api_v1.0.yml``):
      * Endpoint: ``POST https://digitalsky.dgca.gov.in/digital-sky/flightplan/validate/airspace``
      * Required headers: ``X-API-VERSION: 1``, ``source``, ``x-auth`` (pilot token)
      * Request body: ``FlightPlanAreaValidateRequest`` with base64-encoded GeoJSON
        ``FeatureCollection`` in ``geom`` and ``maxAltitude <= 400`` feet.

    Credentials are read strictly from environment variables (``DIGITALSKY_AUTH_TOKEN``
    or ``DIGITALSKY_TOKEN``) and are never logged or hardcoded. When credentials or
    transport are absent, ``check()`` returns ``checked=False`` and
    ``official_clearance=False`` so callers never mistake local scenario polygon
    checks for official DGCA/AAI airspace clearance.
    """

    name = "digitalsky_intersection"

    def __init__(
        self,
        transport=None,
        token: Optional[str] = None,
        source_app: str = "med-drone-mission-control",
    ):
        self.transport = transport
        self.token = (
            token
            or os.environ.get("DIGITALSKY_AUTH_TOKEN")
            or os.environ.get("DIGITALSKY_TOKEN")
        )
        self.source_app = source_app

    def check(self, polygon: Sequence[Point], floor_ft: float = 0, ceiling_ft: float = 400,
              start: Optional[datetime] = None, end: Optional[datetime] = None) -> IntersectionResult:
        now = _utc_now()
        try:
            req_preview = build_digitalsky_validate_request(
                polygon=polygon,
                floor_ft=floor_ft,
                ceiling_ft=ceiling_ft,
                start=start,
                end=end,
                source_app=self.source_app,
            )
        except ValueError as ve:
            return IntersectionResult(
                checked=False,
                intersects=None,
                provider=self.name,
                checked_at=now,
                detail=f"invalid flight plan parameters: {ve}",
                official_clearance=False,
            )

        if self.transport is None or not self.token:
            missing = [n for n, v in (("transport", self.transport), ("DIGITALSKY_TOKEN", self.token)) if not v]
            return IntersectionResult(
                False,
                None,
                self.name,
                now,
                "not configured: missing %s" % ", ".join(missing),
                raw={
                    "official_clearance": False,
                    "status": "UNAUTHORISED_NO_PILOT_TOKEN" if not self.token else "NO_TRANSPORT",
                    "openapi_endpoint": req_preview["url"],
                    "headers_redacted": {**req_preview["headers"], "x-auth": "<REDACTED_MISSING>"},
                    "body_summary": {
                        "minAltitude_ft": req_preview["body"]["minAltitude"],
                        "maxAltitude_ft": req_preview["body"]["maxAltitude"],
                        "isCircle": req_preview["body"]["isCircle"],
                        "geom_base64_len": len(req_preview["body"]["geom"]),
                    },
                },
                official_clearance=False,
            )
        try:
            raw = self.transport(token=self.token, polygon=list(polygon), floor_ft=floor_ft,
                                 ceiling_ft=ceiling_ft, start=start, end=end)
        except Exception as e:      # network / auth failure
            return IntersectionResult(False, None, self.name, now, "request failed: %s" % e,
                                      official_clearance=False)
        if not isinstance(raw, dict) or not isinstance(raw.get("intersects"), bool):
            return IntersectionResult(False, None, self.name, now,
                                      "unrecognised response", raw if isinstance(raw, dict) else None,
                                      official_clearance=False)
        return IntersectionResult(
            True,
            raw["intersects"],
            self.name,
            now,
            raw.get("detail", ""),
            raw,
            official_clearance=(not raw["intersects"]),
        )

    def validate_route_corridor(
        self,
        waypoints: Sequence[Point],
        alt_m_agl: float,
        start: Optional[datetime] = None,
        end: Optional[datetime] = None,
        pad_deg: float = 0.002,
    ) -> IntersectionResult:
        """Validate a planned route corridor against the DigitalSky 400 ft AGL schema."""
        poly = mission_bounding_polygon(waypoints, pad_deg=pad_deg)
        ceiling_ft = min(DIGITALSKY_MAX_ALTITUDE_FT, max(1.0, alt_m_agl * METERS_TO_FEET))
        if alt_m_agl * METERS_TO_FEET > DIGITALSKY_MAX_ALTITUDE_FT:
            return IntersectionResult(
                checked=False,
                intersects=True,
                provider=self.name,
                checked_at=_utc_now(),
                detail=(
                    f"Planned altitude {alt_m_agl:.1f} m AGL ({alt_m_agl * METERS_TO_FEET:.1f} ft) "
                    f"exceeds DigitalSky Green-zone 400 ft limit"
                ),
                official_clearance=False,
            )
        return self.check(poly, floor_ft=0.0, ceiling_ft=ceiling_ft, start=start, end=end)


@dataclass(frozen=True)
class MapplsAirspaceLayerSpec:
    """Verified specification of Mappls Airspace Web SDK visual layers.

    Verified against https://developer.mappls.com/mapping/air-space/:
      Mappls exposes a browser JavaScript vector-tile overlay widget
      ``mappls.airspaceLayer({ map, layers })`` with 4 named visual layers.
      It does NOT provide a REST polygon geometry download endpoint and is
      strictly classified as MAP_VISUALIZATION_ONLY.
    """

    provider: str = "mappls_airspace_web_sdk"
    documentation_url: str = "https://developer.mappls.com/mapping/air-space/"
    classification: str = SourceClassification.MAP_VISUALIZATION_ONLY
    supports_polygon_export: bool = False
    supports_autonomous_route_clearance: bool = False
    layers: Tuple[str, ...] = (
        "coastal_area_india_region_25km",
        "airport_region_radius_8_to_12_km_yellow",
        "airport_region_radius_5_to_8_km",
        "airport_region_radius_0_to_5_km",
    )

    def to_dict(self) -> dict:
        return {
            "provider": self.provider,
            "documentation_url": self.documentation_url,
            "classification": self.classification,
            "supports_polygon_export": self.supports_polygon_export,
            "supports_autonomous_route_clearance": self.supports_autonomous_route_clearance,
            "layers": list(self.layers),
        }


def mission_bounding_polygon(points: Sequence[Point], pad_deg: float = 0.002) -> List[Point]:
    lats = [p[0] for p in points]
    lons = [p[1] for p in points]
    a, b = min(lats) - pad_deg, max(lats) + pad_deg
    c, d = min(lons) - pad_deg, max(lons) + pad_deg
    return [(a, c), (a, d), (b, d), (b, c)]


class DynamicAirspaceMonitor:
    """In-flight airspace monitor that polls an airspace provider and tracks 4D restriction changes.

    Tracks for every update:
      - source, retrieved_at, effective_from, expires_at
      - geographic geometry or validated intersection result
      - altitude restrictions and time restrictions
      - source_classification, freshness_age_s, confidence
      - added_zones and expired_zones

    Safety Invariant:
      If the airspace source is unavailable or stale, it NEVER silently clears active
      restrictions or treats the region as clear (``safe_to_clear_restrictions=False``).
    """

    def __init__(
        self,
        provider,
        digitalsky_checker: Optional[DigitalSkyIntersectionChecker] = None,
        poll_interval_s: float = 30.0,
        clock=_utc_now,
    ):
        self.provider = provider
        self.digitalsky_checker = digitalsky_checker
        self.poll_interval_s = poll_interval_s
        self.clock = clock
        self.last_poll_time: Optional[datetime] = None
        self.last_content_hash: Optional[str] = None
        self.active_zone_ids: set = set()

    def check_for_updates(
        self,
        current_zones: Sequence,
        route_waypoints: Optional[Sequence[Point]] = None,
        alt_m_agl: float = 60.0,
        now: Optional[datetime] = None,
    ) -> dict:
        eval_time = now or self.clock()
        self.last_poll_time = eval_time

        # Normalize existing AirspaceZone objects
        existing_by_id: Dict[str, AirspaceZone] = {}
        raw_polygons = []
        for z in current_zones:
            if hasattr(z, "zone_id") and hasattr(z, "polygon"):
                existing_by_id[z.zone_id] = z
            else:
                raw_polygons.append(list(z))

        # 1. Check for expired restrictions in current_zones
        expired_ids: List[str] = []
        still_active_existing: Dict[str, AirspaceZone] = {}
        for zid, z in existing_by_id.items():
            if not z.active_at(eval_time):
                expired_ids.append(zid)
            else:
                still_active_existing[zid] = z

        # 2. Poll external/file/cached provider
        try:
            snap: AirspaceSnapshot = self.provider.fetch()
        except AirspaceDataStale as stale_err:
            return {
                "status": "SOURCE_STALE",
                "available": False,
                "stale": True,
                "changed": bool(expired_ids),
                "safe_to_clear_restrictions": False,
                "error": str(stale_err),
                "retrieved_at": _iso(eval_time),
                "added_zone_ids": [],
                "expired_zone_ids": expired_ids,
                "active_zones": list(still_active_existing.values()) + raw_polygons,
                "confidence": "DEGRADED_STALE",
            }
        except (AirspaceUnavailable, OSError, ValueError) as unavail_err:
            return {
                "status": "SOURCE_UNAVAILABLE",
                "available": False,
                "stale": False,
                "changed": bool(expired_ids),
                "safe_to_clear_restrictions": False,
                "error": str(unavail_err),
                "retrieved_at": _iso(eval_time),
                "added_zone_ids": [],
                "expired_zone_ids": expired_ids,
                "active_zones": list(still_active_existing.values()) + raw_polygons,
                "confidence": "UNAVAILABLE_RETAIN_RESTRICTIONS",
            }

        # 3. Merge active incoming zones and track additions/expirations
        added_ids: List[str] = []
        merged_by_id: Dict[str, AirspaceZone] = dict(still_active_existing)
        zone_records = []
        for z in snap.zones:
            if not z.active_at(eval_time):
                if z.zone_id in merged_by_id:
                    del merged_by_id[z.zone_id]
                    if z.zone_id not in expired_ids:
                        expired_ids.append(z.zone_id)
                continue
            if z.zone_id not in existing_by_id:
                added_ids.append(z.zone_id)
            merged_by_id[z.zone_id] = z
            zone_records.append(
                {
                    "zone_id": z.zone_id,
                    "category": z.category,
                    "floor_m_agl": z.floor_m_agl,
                    "ceiling_m_agl": z.ceiling_m_agl,
                    "effective_from": _iso(z.valid_from),
                    "expires_at": _iso(z.valid_to),
                    "vertex_count": len(z.polygon),
                }
            )

        # 4. Optional DigitalSky intersection validation on remaining route
        ds_info = None
        if self.digitalsky_checker is not None and route_waypoints and len(route_waypoints) >= 2:
            ds_res = self.digitalsky_checker.validate_route_corridor(
                route_waypoints, alt_m_agl=alt_m_agl, start=eval_time
            )
            ds_info = {
                "checked": ds_res.checked,
                "intersects": ds_res.intersects,
                "official_clearance": ds_res.official_clearance,
                "provenance_classification": ds_res.provenance_classification,
                "detail": ds_res.detail,
            }

        changed = bool(added_ids or expired_ids or (snap.content_sha256 != self.last_content_hash and self.last_content_hash is not None))
        self.last_content_hash = snap.content_sha256
        self.active_zone_ids = set(merged_by_id.keys())

        confidence = "HIGH_OFFICIAL" if snap.official and not snap.stale else (
            "DEGRADED_CACHED" if snap.stale else "VERIFIED_LOCAL_OR_EXTERNAL"
        )
        return {
            "status": "OK_CACHED" if snap.stale else "OK_FRESH",
            "available": True,
            "stale": snap.stale,
            "changed": changed,
            "safe_to_clear_restrictions": not snap.stale,
            "provider": snap.provider,
            "source": snap.source,
            "source_classification": snap.provenance_classification,
            "retrieved_at": _iso(snap.retrieved_at),
            "freshness_age_s": round(snap.age_s(eval_time), 2),
            "content_sha256": snap.content_sha256,
            "confidence": confidence,
            "added_zone_ids": added_ids,
            "expired_zone_ids": expired_ids,
            "zone_records": zone_records,
            "digitalsky_validation": ds_info,
            "active_zones": list(merged_by_id.values()) + raw_polygons,
        }

