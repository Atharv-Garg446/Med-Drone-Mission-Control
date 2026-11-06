"""
gdacs_provider.py
=================
Live and cached disaster-event ingestion from the Global Disaster Awareness
and Coordination System (GDACS) REST API, with strict data-provenance tracking.

Official API Contract (verified against https://www.gdacs.org/gdacsapi/swagger/v1/swagger.json):
  - Search endpoint:
      GET https://www.gdacs.org/gdacsapi/api/Events/geteventlist/SEARCH
      Query parameters:
        * eventlist: semicolon-separated event types (FL, EQ, TC, VO, DR, WF)
        * fromDate / toDate: YYYY-MM-DD bounds
        * alertlevel: Green;Orange;Red
        * country: optional country name filter
      Returns:
        * HTTP 200 with a GeoJSON FeatureCollection of disaster events, or
        * HTTP 204 (No Content) when no events match the filter.
  - Footprint geometry endpoint:
      GET https://www.gdacs.org/gdacsapi/api/polygons/getgeometry
      Query parameters:
        * eventtype, eventid, episodeid
      Returns:
        * GeoJSON FeatureCollection containing Point, Polygon, and MultiPolygon
          footprints for the event episode.

Attribution Requirement:
  "Global Disaster Awareness and Coordination System, GDACS."

Scope Boundary (Mandatory Engineering Rule):
  GDACS provides regional disaster-event awareness and geographical footprints
  (floods, earthquakes, tropical cyclones). It is NOT an official drone
  no-fly-zone (NFZ) database and does NOT by itself prove street-level
  landing-pad safety.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from typing import Dict, List, Optional, Sequence, Tuple

from geo_utils import haversine_km, point_in_polygon

Point = Tuple[float, float]

GDACS_ATTRIBUTION = "Global Disaster Awareness and Coordination System, GDACS."
GDACS_SCOPE_DISCLAIMER = (
    "GDACS provides regional disaster awareness and geographical footprints "
    "(FL, EQ, TC, VO, DR, WF). It is NOT an official drone no-fly-zone (NFZ) "
    "database and does NOT prove micro-scale landing-pad clearance."
)

GDACS_SEARCH_URL = "https://www.gdacs.org/gdacsapi/api/Events/geteventlist/SEARCH"
GDACS_GEOMETRY_URL = "https://www.gdacs.org/gdacsapi/api/polygons/getgeometry"


class SourceClassification:
    """Strict provenance categories across all external, cached, simulated, and fixture sources."""

    LIVE_EXTERNAL = "LIVE_EXTERNAL"
    OFFICIAL_API_VALIDATION = "OFFICIAL_API_VALIDATION"
    MAP_VISUALIZATION_ONLY = "MAP_VISUALIZATION_ONLY"
    SIMULATED_SENSOR = "SIMULATED_SENSOR"
    CACHED = "CACHED"
    SYNTHETIC_FIXTURE = "SYNTHETIC_FIXTURE"


class GDACSUnavailable(RuntimeError):
    """Raised when the GDACS endpoint cannot be reached or returns an invalid payload."""


class GDACSDataStale(RuntimeError):
    """Raised when only cached GDACS data older than max_age_s is available."""


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(dt: Optional[datetime]) -> Optional[str]:
    return dt.isoformat() if dt else None


def _parse_gdacs_dt(value: Optional[str]) -> Optional[datetime]:
    if not value:
        return None
    s = str(value).strip()
    if not s:
        return None
    try:
        dt = datetime.fromisoformat(s.replace("Z", "+00:00"))
        return dt if dt.tzinfo else dt.replace(tzinfo=timezone.utc)
    except ValueError:
        for fmt in ("%Y-%m-%dT%H:%M:%S", "%Y-%m-%d %H:%M:%S", "%Y-%m-%d"):
            try:
                return datetime.strptime(s, fmt).replace(tzinfo=timezone.utc)
            except ValueError:
                continue
    return None


def _extract_polygons_from_geometry(geom: Optional[dict]) -> List[List[Point]]:
    """Extract outer polygon rings as [(lat, lon), ...] from GeoJSON Polygon/MultiPolygon/GeometryCollection."""
    if not geom or not isinstance(geom, dict):
        return []
    gtype = geom.get("type")
    rings: List[List[Point]] = []
    if gtype == "Polygon":
        coords = geom.get("coordinates") or []
        if coords:
            ring = [(float(c[1]), float(c[0])) for c in coords[0]]
            if len(ring) > 1 and ring[0] == ring[-1]:
                ring = ring[:-1]
            if len(ring) >= 3:
                rings.append(ring)
    elif gtype == "MultiPolygon":
        for poly in geom.get("coordinates") or []:
            if poly and poly[0]:
                ring = [(float(c[1]), float(c[0])) for c in poly[0]]
                if len(ring) > 1 and ring[0] == ring[-1]:
                    ring = ring[:-1]
                if len(ring) >= 3:
                    rings.append(ring)
    elif gtype == "GeometryCollection":
        for sub in geom.get("geometries") or []:
            rings.extend(_extract_polygons_from_geometry(sub))
    return rings


def parse_gdacs_geometry_collection(doc: dict) -> List[List[Point]]:
    """Extract all polygon footprints from a GDACS /api/polygons/getgeometry FeatureCollection."""
    if not isinstance(doc, dict):
        return []
    rings: List[List[Point]] = []
    if doc.get("type") == "FeatureCollection":
        for feat in doc.get("features") or []:
            if isinstance(feat, dict):
                rings.extend(_extract_polygons_from_geometry(feat.get("geometry")))
    else:
        rings.extend(_extract_polygons_from_geometry(doc))
    return rings


@dataclass
class GDACSEvent:
    """Normalized disaster event record with explicit provenance metadata."""

    event_key: str
    event_type: str
    event_id: int
    episode_id: int
    name: str
    description: str
    alert_level: str
    alert_score: float
    is_current: bool
    country: str
    iso3: str
    centroid: Point  # (lat, lon)
    bbox: Optional[Tuple[float, float, float, float]] = None  # (min_lon, min_lat, max_lon, max_lat)
    polygons: List[List[Point]] = field(default_factory=list)
    from_date: Optional[datetime] = None
    to_date: Optional[datetime] = None
    date_modified: Optional[datetime] = None
    geometry_url: Optional[str] = None
    report_url: Optional[str] = None
    severity_data: Dict = field(default_factory=dict)
    source_org: str = "GDACS"
    provenance: str = SourceClassification.LIVE_EXTERNAL
    attribution: str = GDACS_ATTRIBUTION

    def is_expired(self, when: Optional[datetime] = None, grace_days: int = 14) -> bool:
        """Return True if the event is explicitly marked non-current and past its end window."""
        when = when or _utc_now()
        if self.is_current:
            return False
        if self.to_date is not None:
            return when > (self.to_date + timedelta(days=grace_days))
        return False

    def distance_to_km(self, lat: float, lon: float) -> float:
        return haversine_km(self.centroid, (lat, lon))

    def contains_or_near(self, lat: float, lon: float, radius_km: float = 50.0) -> bool:
        """Check whether (lat, lon) lies inside an event footprint polygon, bbox, or radius_km of centroid."""
        pt = (lat, lon)
        for ring in self.polygons:
            if len(ring) >= 3 and point_in_polygon(pt, ring):
                return True
        if self.bbox is not None:
            min_lon, min_lat, max_lon, max_lat = self.bbox
            if (min_lat <= lat <= max_lat) and (min_lon <= lon <= max_lon):
                return True
        return self.distance_to_km(lat, lon) <= radius_km

    def to_dict(self) -> dict:
        return {
            "event_key": self.event_key,
            "event_type": self.event_type,
            "event_id": self.event_id,
            "episode_id": self.episode_id,
            "name": self.name,
            "description": self.description,
            "alert_level": self.alert_level,
            "alert_score": self.alert_score,
            "is_current": self.is_current,
            "country": self.country,
            "iso3": self.iso3,
            "centroid": list(self.centroid),
            "bbox": list(self.bbox) if self.bbox else None,
            "polygon_count": len(self.polygons),
            "from_date": _iso(self.from_date),
            "to_date": _iso(self.to_date),
            "date_modified": _iso(self.date_modified),
            "geometry_url": self.geometry_url,
            "severity_data": self.severity_data,
            "provenance": self.provenance,
            "attribution": self.attribution,
        }


@dataclass
class GDACSSnapshot:
    """Point-in-time collection of GDACS disaster events with freshness & provenance audit."""

    provider: str
    source_url: str
    events: List[GDACSEvent]
    retrieved_at: datetime
    content_sha256: str
    provenance: str = SourceClassification.LIVE_EXTERNAL
    stale: bool = False
    error: Optional[str] = None
    duplicates_removed: int = 0
    expired_filtered: int = 0
    attribution: str = GDACS_ATTRIBUTION
    scope_disclaimer: str = GDACS_SCOPE_DISCLAIMER

    def age_s(self, now: Optional[datetime] = None) -> float:
        return ((now or _utc_now()) - self.retrieved_at).total_seconds()

    def active_events(self, when: Optional[datetime] = None) -> List[GDACSEvent]:
        return [e for e in self.events if not e.is_expired(when)]

    def events_near(
        self,
        lat: float,
        lon: float,
        radius_km: float = 250.0,
        event_types: Optional[Sequence[str]] = None,
    ) -> List[GDACSEvent]:
        allowed = {t.upper() for t in event_types} if event_types else None
        matched = []
        for ev in self.events:
            if allowed and ev.event_type.upper() not in allowed:
                continue
            if ev.contains_or_near(lat, lon, radius_km=radius_km):
                matched.append(ev)
        matched.sort(key=lambda e: e.distance_to_km(lat, lon))
        return matched

    def assess_site_disaster_context(
        self,
        lat: float,
        lon: float,
        site_suitability: Optional[dict] = None,
        search_radius_km: float = 250.0,
    ) -> dict:
        """Combine macro-level GDACS disaster context with micro-level landing site suitability.

        Never claims that absence of a GDACS alert guarantees a safe landing pad,
        nor that a GDACS polygon is a legal DGCA/AAI no-fly zone.
        """
        nearby = self.events_near(lat, lon, radius_km=search_radius_km)
        local_suit = dict(site_suitability or {})
        local_safe_to_land = local_suit.get("is_safe_for_landing", True)
        surface = local_suit.get("surface_condition", "unknown")
        return {
            "query_point": [lat, lon],
            "gdacs_provenance": self.provenance,
            "gdacs_stale": self.stale,
            "gdacs_retrieved_at": _iso(self.retrieved_at),
            "gdacs_attribution": self.attribution,
            "gdacs_scope_disclaimer": self.scope_disclaimer,
            "nearby_disaster_count": len(nearby),
            "nearby_disasters": [
                {
                    "event_key": e.event_key,
                    "event_type": e.event_type,
                    "name": e.name,
                    "alert_level": e.alert_level,
                    "country": e.country,
                    "distance_km": round(e.distance_to_km(lat, lon), 2),
                }
                for e in nearby[:5]
            ],
            "local_site_suitability": {
                "is_safe_for_landing": local_safe_to_land,
                "surface_condition": surface,
                "recommended_delivery_mode": "land" if local_safe_to_land else "hover",
                "source_classification": SourceClassification.SYNTHETIC_FIXTURE
                if "provenance" not in local_suit
                else local_suit["provenance"],
            },
        }

    def metadata(self) -> dict:
        by_type: Dict[str, int] = {}
        by_alert: Dict[str, int] = {}
        for ev in self.events:
            by_type[ev.event_type] = by_type.get(ev.event_type, 0) + 1
            by_alert[ev.alert_level] = by_alert.get(ev.alert_level, 0) + 1
        return {
            "provider": self.provider,
            "source_url": self.source_url,
            "provenance": self.provenance,
            "retrieved_at": _iso(self.retrieved_at),
            "age_s": round(self.age_s(), 2),
            "content_sha256": self.content_sha256,
            "stale": self.stale,
            "error": self.error,
            "event_count": len(self.events),
            "events_by_type": by_type,
            "events_by_alert_level": by_alert,
            "duplicates_removed": self.duplicates_removed,
            "expired_filtered": self.expired_filtered,
            "attribution": self.attribution,
            "scope_disclaimer": self.scope_disclaimer,
        }


def parse_gdacs_feature_collection(
    doc: dict,
    provenance: str = SourceClassification.LIVE_EXTERNAL,
    filter_expired: bool = False,
    now: Optional[datetime] = None,
) -> Tuple[List[GDACSEvent], int, int]:
    """Parse a GDACS GeoJSON FeatureCollection into deduplicated GDACSEvent objects.

    Returns (events, duplicates_removed, expired_filtered).
    """
    if not isinstance(doc, dict) or doc.get("type") != "FeatureCollection":
        raise ValueError("Expected a GeoJSON FeatureCollection from GDACS")

    dedup: Dict[str, GDACSEvent] = {}
    duplicates_removed = 0
    expired_filtered = 0

    for feat in doc.get("features") or []:
        if not isinstance(feat, dict):
            continue
        props = feat.get("properties") or {}
        geom = feat.get("geometry") or {}

        ev_type = str(props.get("eventtype") or "").strip().upper()
        try:
            ev_id = int(props.get("eventid") or 0)
        except (TypeError, ValueError):
            ev_id = 0
        try:
            ep_id = int(props.get("episodeid") or 0)
        except (TypeError, ValueError):
            ep_id = 0

        if not ev_type or ev_id <= 0:
            continue

        # Extract centroid from Point geometry or bbox
        centroid: Optional[Point] = None
        if geom.get("type") == "Point":
            coords = geom.get("coordinates") or []
            if len(coords) >= 2:
                centroid = (float(coords[1]), float(coords[0]))

        raw_bbox = feat.get("bbox") or props.get("bbox")
        bbox: Optional[Tuple[float, float, float, float]] = None
        if isinstance(raw_bbox, (list, tuple)) and len(raw_bbox) == 4:
            bbox = (
                float(raw_bbox[0]),
                float(raw_bbox[1]),
                float(raw_bbox[2]),
                float(raw_bbox[3]),
            )
            if centroid is None:
                centroid = ((bbox[1] + bbox[3]) / 2.0, (bbox[0] + bbox[2]) / 2.0)

        polygons = _extract_polygons_from_geometry(geom)
        if centroid is None and polygons and polygons[0]:
            ring = polygons[0]
            centroid = (
                sum(p[0] for p in ring) / len(ring),
                sum(p[1] for p in ring) / len(ring),
            )
        if centroid is None:
            continue

        lat, lon = centroid
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            continue

        urls = props.get("url") if isinstance(props.get("url"), dict) else {}
        is_cur_raw = str(props.get("iscurrent", "true")).strip().lower()
        is_current = is_cur_raw in ("true", "1", "yes", "y")

        try:
            alert_score = float(props.get("alertscore") or 0.0)
        except (TypeError, ValueError):
            alert_score = 0.0

        ev_key = f"{ev_type}_{ev_id}_{ep_id}"
        ev = GDACSEvent(
            event_key=ev_key,
            event_type=ev_type,
            event_id=ev_id,
            episode_id=ep_id,
            name=str(props.get("name") or props.get("eventname") or ev_key),
            description=str(props.get("description") or props.get("htmldescription") or ""),
            alert_level=str(props.get("alertlevel") or "Green"),
            alert_score=alert_score,
            is_current=is_current,
            country=str(props.get("country") or ""),
            iso3=str(props.get("iso3") or ""),
            centroid=centroid,
            bbox=bbox,
            polygons=polygons,
            from_date=_parse_gdacs_dt(props.get("fromdate")),
            to_date=_parse_gdacs_dt(props.get("todate")),
            date_modified=_parse_gdacs_dt(props.get("datemodified")),
            geometry_url=urls.get("geometry"),
            report_url=urls.get("report"),
            severity_data=dict(props.get("severitydata") or {})
            if isinstance(props.get("severitydata"), dict)
            else {},
            source_org=str(props.get("source") or "GDACS"),
            provenance=provenance,
            attribution=GDACS_ATTRIBUTION,
        )

        if filter_expired and ev.is_expired(now):
            expired_filtered += 1
            continue

        if ev_key in dedup:
            duplicates_removed += 1
            existing = dedup[ev_key]
            if (
                ev.date_modified
                and existing.date_modified
                and ev.date_modified > existing.date_modified
            ):
                dedup[ev_key] = ev
        else:
            dedup[ev_key] = ev

    return list(dedup.values()), duplicates_removed, expired_filtered


class GDACSProvider:
    """Live HTTP client for the official GDACS Event List and Polygon Geometry endpoints."""

    def __init__(
        self,
        event_types: Sequence[str] = ("FL", "EQ", "TC"),
        alert_levels: Sequence[str] = ("green", "orange", "red"),
        lookback_days: int = 35,
        country: Optional[str] = None,
        timeout_s: float = 12.0,
        fetch_footprints_for: Optional[Sequence[str]] = None,
        filter_expired: bool = False,
        base_url: str = GDACS_SEARCH_URL,
        opener=None,
        clock=_utc_now,
    ):
        self.event_types = tuple(t.upper() for t in event_types)
        self.alert_levels = tuple(a.lower() for a in alert_levels)
        self.lookback_days = lookback_days
        self.country = country
        self.timeout_s = timeout_s
        self.fetch_footprints_for = set(fetch_footprints_for or ())
        self.filter_expired = filter_expired
        self.base_url = base_url
        self._opener = opener or urllib.request.urlopen
        self._clock = clock

    def build_search_url(
        self,
        from_date: Optional[datetime] = None,
        to_date: Optional[datetime] = None,
    ) -> str:
        now = self._clock()
        to_dt = to_date or now
        from_dt = from_date or (to_dt - timedelta(days=self.lookback_days))
        params = {
            "eventlist": ";".join(self.event_types),
            "fromDate": from_dt.strftime("%Y-%m-%d"),
            "toDate": to_dt.strftime("%Y-%m-%d"),
            "alertlevel": ";".join(self.alert_levels),
        }
        if self.country:
            params["country"] = self.country
        return f"{self.base_url}?{urllib.parse.urlencode(params)}"

    def _http_get_bytes(self, url: str) -> Tuple[int, bytes]:
        req = urllib.request.Request(
            url,
            headers={
                "Accept": "application/json",
                "User-Agent": "MedDroneMissionControl/1.0 (Autonomous-UAV-Research)",
            },
        )
        try:
            with self._opener(req, timeout=self.timeout_s) as resp:
                status = getattr(resp, "status", None) or resp.getcode()
                body = resp.read()
                return int(status), body
        except urllib.error.HTTPError as e:
            if e.code == 204:
                return 204, b""
            raise GDACSUnavailable(f"GDACS HTTP {e.code} for {url}: {e.reason}") from e
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise GDACSUnavailable(f"GDACS network error for {url}: {e}") from e

    def fetch_event_Geometry(self, event: GDACSEvent) -> List[List[Point]]:
        """Fetch detailed polygon footprints for a specific GDACSEvent from /api/polygons/getgeometry."""
        url = event.geometry_url
        if not url:
            params = urllib.parse.urlencode(
                {
                    "eventtype": event.event_type,
                    "eventid": event.event_id,
                    "episodeid": event.episode_id,
                }
            )
            url = f"{GDACS_GEOMETRY_URL}?{params}"
        status, raw = self._http_get_bytes(url)
        if status == 204 or not raw.strip():
            return []
        try:
            doc = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            raise GDACSUnavailable(f"Malformed GDACS geometry JSON from {url}: {e}") from e
        polygons = parse_gdacs_geometry_collection(doc)
        event.polygons = polygons
        return polygons

    def fetch(
        self,
        from_date: Optional[datetime] = None,
        to_date: Optional[datetime] = None,
    ) -> GDACSSnapshot:
        url = self.build_search_url(from_date=from_date, to_date=to_date)
        now = self._clock()
        status, raw = self._http_get_bytes(url)

        if status == 204 or not raw.strip():
            empty_doc = {"type": "FeatureCollection", "features": []}
            raw = json.dumps(empty_doc).encode("utf-8")
            doc = empty_doc
        else:
            try:
                doc = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as e:
                raise GDACSUnavailable(f"Malformed GDACS JSON from {url}: {e}") from e

        try:
            events, dups, expired = parse_gdacs_feature_collection(
                doc,
                provenance=SourceClassification.LIVE_EXTERNAL,
                filter_expired=self.filter_expired,
                now=now,
            )
        except ValueError as e:
            raise GDACSUnavailable(f"Invalid GDACS schema from {url}: {e}") from e

        if self.fetch_footprints_for:
            for ev in events:
                if ev.event_key in self.fetch_footprints_for or ev.country in self.fetch_footprints_for:
                    try:
                        self.fetch_event_Geometry(ev)
                    except GDACSUnavailable:
                        pass

        return GDACSSnapshot(
            provider="gdacs_live_api",
            source_url=url,
            events=events,
            retrieved_at=now,
            content_sha256=hashlib.sha256(raw).hexdigest(),
            provenance=SourceClassification.LIVE_EXTERNAL,
            stale=False,
            duplicates_removed=dups,
            expired_filtered=expired,
            attribution=GDACS_ATTRIBUTION,
            scope_disclaimer=GDACS_SCOPE_DISCLAIMER,
        )


class CachedGDACSProvider:
    """Resilient wrapper around GDACSProvider with explicit staleness & optional disk cache.

    Never relabels cached data as LIVE_EXTERNAL: when a fallback snapshot is served,
    ``provenance`` is set to ``SourceClassification.CACHED`` and ``stale`` is ``True``.
    """

    def __init__(
        self,
        inner: GDACSProvider,
        max_age_s: float = 12 * 3600,
        cache_path: Optional[str] = None,
        clock=_utc_now,
    ):
        self.inner = inner
        self.max_age_s = max_age_s
        self.cache_path = cache_path
        self.clock = clock
        self.last_good: Optional[GDACSSnapshot] = None

    def _save_disk_cache(self, snap: GDACSSnapshot) -> None:
        if not self.cache_path:
            return
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.cache_path)), exist_ok=True)
            payload = {
                "provider": snap.provider,
                "source_url": snap.source_url,
                "retrieved_at": _iso(snap.retrieved_at),
                "content_sha256": snap.content_sha256,
                "duplicates_removed": snap.duplicates_removed,
                "expired_filtered": snap.expired_filtered,
                "events": [e.to_dict() for e in snap.events],
            }
            with open(self.cache_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
        except OSError:
            pass

    def _load_disk_cache(self) -> Optional[GDACSSnapshot]:
        if not self.cache_path or not os.path.exists(self.cache_path):
            return None
        try:
            with open(self.cache_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            retrieved_at = _parse_gdacs_dt(data.get("retrieved_at")) or _utc_now()
            events = []
            for item in data.get("events") or []:
                c = item.get("centroid") or [0.0, 0.0]
                b = item.get("bbox")
                events.append(
                    GDACSEvent(
                        event_key=str(item["event_key"]),
                        event_type=str(item["event_type"]),
                        event_id=int(item["event_id"]),
                        episode_id=int(item["episode_id"]),
                        name=str(item.get("name", "")),
                        description=str(item.get("description", "")),
                        alert_level=str(item.get("alert_level", "Green")),
                        alert_score=float(item.get("alert_score", 0.0)),
                        is_current=bool(item.get("is_current", True)),
                        country=str(item.get("country", "")),
                        iso3=str(item.get("iso3", "")),
                        centroid=(float(c[0]), float(c[1])),
                        bbox=(float(b[0]), float(b[1]), float(b[2]), float(b[3])) if b else None,
                        from_date=_parse_gdacs_dt(item.get("from_date")),
                        to_date=_parse_gdacs_dt(item.get("to_date")),
                        date_modified=_parse_gdacs_dt(item.get("date_modified")),
                        geometry_url=item.get("geometry_url"),
                        severity_data=dict(item.get("severity_data") or {}),
                        provenance=SourceClassification.CACHED,
                        attribution=GDACS_ATTRIBUTION,
                    )
                )
            return GDACSSnapshot(
                provider=str(data.get("provider", "gdacs_disk_cache")),
                source_url=str(data.get("source_url", self.cache_path)),
                events=events,
                retrieved_at=retrieved_at,
                content_sha256=str(data.get("content_sha256", "")),
                provenance=SourceClassification.CACHED,
                stale=True,
                duplicates_removed=int(data.get("duplicates_removed", 0)),
                expired_filtered=int(data.get("expired_filtered", 0)),
            )
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            return None

    def fetch(self) -> GDACSSnapshot:
        try:
            snap = self.inner.fetch()
            self.last_good = snap
            self._save_disk_cache(snap)
            return snap
        except (GDACSUnavailable, OSError, ValueError) as exc:
            candidate = self.last_good or self._load_disk_cache()
            if candidate is None:
                raise GDACSUnavailable(f"No live or cached GDACS data available: {exc}") from exc
            age = candidate.age_s(self.clock())
            if age > self.max_age_s:
                raise GDACSDataStale(
                    f"Cached GDACS snapshot is {age:.0f}s old (limit {self.max_age_s:.0f}s); "
                    f"live error: {exc}"
                ) from exc
            cached_events = [
                GDACSEvent(**{**e.__dict__, "provenance": SourceClassification.CACHED})
                for e in candidate.events
            ]
            return GDACSSnapshot(
                provider=candidate.provider,
                source_url=candidate.source_url,
                events=cached_events,
                retrieved_at=candidate.retrieved_at,
                content_sha256=candidate.content_sha256,
                provenance=SourceClassification.CACHED,
                stale=True,
                error=str(exc),
                duplicates_removed=candidate.duplicates_removed,
                expired_filtered=candidate.expired_filtered,
                attribution=GDACS_ATTRIBUTION,
                scope_disclaimer=GDACS_SCOPE_DISCLAIMER,
            )


class GDACSMissionImpactAssessor:
    """Operational pipeline linking GDACS retrieval -> validation -> geometry -> mission impact.

    Strictly distinguishes:
      1. Regional disaster awareness (ADVISORY_REGIONAL_AWARENESS): coarse centroid/bbox
         within relevance_radius_km; retained as advisory without inventing precision.
      2. Validated hazard polygon (VALIDATED_HAZARD_POLYGON): polygon footprint retrieved
         from /api/polygons/getgeometry or Feature geometry.
      3. Legal aviation no-fly zone: GDACS is NEVER a legal aviation NFZ
         (is_legal_aviation_nfz=False).
      4. Physically verified obstacle: GDACS is NEVER a tactical 3D obstacle
         (is_physically_verified_obstacle=False).
      5. Safe landing location: GDACS NEVER proves street-level landing pad clearance
         (proves_safe_landing_location=False). However, when a pending delivery stop
         lies inside a validated GDACS flood (FL), cyclone (TC), or earthquake (EQ)
         hazard polygon, it triggers a conservative site-suitability downgrade from
         ground landing ('land') to 'hover'.
    """

    def __init__(
        self,
        provider,
        relevance_radius_km: float = 250.0,
        fetch_missing_geometries: bool = True,
        avoid_red_hazard_polygons: bool = True,
        clock=_utc_now,
    ):
        self.provider = provider
        self.relevance_radius_km = relevance_radius_km
        self.fetch_missing_geometries = fetch_missing_geometries
        self.avoid_red_hazard_polygons = avoid_red_hazard_polygons
        self.clock = clock
        self.last_content_sha256: Optional[str] = None

    def _inner_gdacs_provider(self) -> Optional[GDACSProvider]:
        if isinstance(self.provider, GDACSProvider):
            return self.provider
        inner = getattr(self.provider, "inner", None)
        if isinstance(inner, GDACSProvider):
            return inner
        return None

    def evaluate_mission_impact(
        self,
        stops: Sequence[object],
        leg_paths: Sequence[Sequence[Point]] = (),
        home: Optional[Point] = None,
        now: Optional[datetime] = None,
    ) -> dict:
        from geo_utils import point_near_polygon, segment_intersects_polygon

        eval_time = now or self.clock()
        try:
            snap: GDACSSnapshot = self.provider.fetch()
        except GDACSDataStale as stale_err:
            return {
                "available": False,
                "status": "STALE",
                "stale": True,
                "error": str(stale_err),
                "attribution": GDACS_ATTRIBUTION,
                "scope_disclaimer": GDACS_SCOPE_DISCLAIMER,
                "semantic_boundaries": {
                    "is_legal_aviation_nfz": False,
                    "is_physically_verified_obstacle": False,
                    "proves_safe_landing_location": False,
                },
                "stop_suitability_updates": [],
                "hazard_avoidance_zones": [],
                "advisory_events": [],
            }
        except (GDACSUnavailable, OSError, ValueError) as unavail_err:
            return {
                "available": False,
                "status": "UNAVAILABLE",
                "stale": False,
                "error": str(unavail_err),
                "attribution": GDACS_ATTRIBUTION,
                "scope_disclaimer": GDACS_SCOPE_DISCLAIMER,
                "semantic_boundaries": {
                    "is_legal_aviation_nfz": False,
                    "is_physically_verified_obstacle": False,
                    "proves_safe_landing_location": False,
                },
                "stop_suitability_updates": [],
                "hazard_avoidance_zones": [],
                "advisory_events": [],
            }

        changed = snap.content_sha256 != self.last_content_sha256
        self.last_content_sha256 = snap.content_sha256

        # Reference coordinate for regional relevance check
        ref_pts: List[Point] = []
        if home:
            ref_pts.append((float(home[0]), float(home[1])))
        for s in stops:
            lat = getattr(s, "lat", None)
            lon = getattr(s, "lon", None)
            if lat is not None and lon is not None:
                ref_pts.append((float(lat), float(lon)))

        active_events = snap.active_events(eval_time)
        inner_http = self._inner_gdacs_provider()

        advisory_events = []
        stop_suitability_updates = []
        hazard_avoidance_zones = []
        route_hazard_advisories = []

        for ev in active_events:
            # Check geographic relevance to any mission point
            relevant = False
            min_dist_km = float("inf")
            for pt in ref_pts:
                d_km = ev.distance_to_km(pt[0], pt[1])
                if d_km < min_dist_km:
                    min_dist_km = d_km
                if ev.contains_or_near(pt[0], pt[1], radius_km=self.relevance_radius_km):
                    relevant = True
            if not relevant and ref_pts:
                continue

            # Retrieve footprint geometry if relevant and not yet populated
            if self.fetch_missing_geometries and not ev.polygons and inner_http is not None:
                try:
                    inner_http.fetch_event_Geometry(ev)
                except Exception:
                    pass

            valid_polys = [ring for ring in ev.polygons if len(ring) >= 3]
            if not valid_polys:
                # Coarse centroid/bbox only -> retain strictly as regional advisory
                advisory_events.append(
                    {
                        "event_key": ev.event_key,
                        "event_type": ev.event_type,
                        "name": ev.name,
                        "alert_level": ev.alert_level,
                        "country": ev.country,
                        "distance_km": round(min_dist_km, 2) if min_dist_km != float("inf") else None,
                        "classification": "ADVISORY_REGIONAL_AWARENESS",
                        "geometry_status": "COARSE_CENTROID_OR_BBOX_ONLY",
                        "planner_action": "ADVISORY_NO_WAYPOINT_MODIFICATION",
                        "provenance": ev.provenance,
                        "attribution": ev.attribution,
                    }
                )
                continue

            # We have 1+ validated hazard polygons for this relevant active event
            for s in stops:
                sid = getattr(s, "id", None)
                slat = getattr(s, "lat", None)
                slon = getattr(s, "lon", None)
                if sid is None or slat is None or slon is None:
                    continue
                for poly in valid_polys:
                    if point_in_polygon((slat, slon), poly) or point_near_polygon((slat, slon), poly, 100.0):
                        surf_map = {"FL": "flooded", "TC": "flooded", "EQ": "debris", "WF": "debris"}
                        inferred_surf = surf_map.get(ev.event_type.upper(), "flooded")
                        existing_suit = getattr(s, "site_suitability", None)
                        existing_clearance = 15.0
                        if isinstance(existing_suit, dict):
                            existing_clearance = float(existing_suit.get("obstacle_clearance_m", 15.0))
                        elif hasattr(existing_suit, "obstacle_clearance_m"):
                            existing_clearance = float(existing_suit.obstacle_clearance_m)
                        stop_suitability_updates.append(
                            {
                                "stop_id": sid,
                                "event_key": ev.event_key,
                                "event_type": ev.event_type,
                                "alert_level": ev.alert_level,
                                "classification": "VALIDATED_HAZARD_POLYGON",
                                "planner_action": "DOWNGRADE_LAND_TO_HOVER",
                                "new_suitability": {
                                    "is_safe_for_landing": False,
                                    "surface_condition": inferred_surf,
                                    "slope_deg": 2.0,
                                    "obstacle_clearance_m": existing_clearance,
                                    "recommended_mode": "hover",
                                    "uncertainty": "uncertain",
                                    "provenance": ev.provenance,
                                    "notes": (
                                        f"Inside validated GDACS {ev.event_type} polygon ({ev.event_key}); "
                                        f"ground landing disabled ({GDACS_ATTRIBUTION})"
                                    ),
                                },
                            }
                        )
                        break

            # Check route leg intersections with validated hazard polygons
            candidate_paths = list(leg_paths or ())
            if not candidate_paths and ref_pts and len(ref_pts) >= 2:
                candidate_paths = [ref_pts + ([ref_pts[0]] if home else [])]
            for leg_idx, path in enumerate(candidate_paths):
                for i in range(len(path) - 1):
                    p1, p2 = path[i], path[i + 1]
                    for poly_idx, poly in enumerate(valid_polys):
                        if segment_intersects_polygon(p1, p2, poly, margin_m=0.0):
                            route_hazard_advisories.append(
                                {
                                    "event_key": ev.event_key,
                                    "event_type": ev.event_type,
                                    "alert_level": ev.alert_level,
                                    "leg_index": leg_idx,
                                    "polygon_index": poly_idx,
                                    "classification": "VALIDATED_HAZARD_POLYGON",
                                    "is_legal_aviation_nfz": False,
                                    "attribution": ev.attribution,
                                }
                            )
                            if self.avoid_red_hazard_polygons and ev.alert_level.strip().lower() == "red":
                                from airspace_provider import AirspaceZone

                                zid = f"GDACS_HAZARD_{ev.event_key}_{poly_idx}"
                                if not any(getattr(z, "zone_id", None) == zid for z in hazard_avoidance_zones):
                                    hazard_avoidance_zones.append(
                                        AirspaceZone(
                                            zone_id=zid,
                                            category="temporary_restriction",
                                            polygon=poly,
                                            name=f"GDACS Red Hazard Footprint {ev.name}",
                                            floor_m_agl=0.0,
                                            ceiling_m_agl=120.0,
                                            valid_from=ev.from_date,
                                            valid_to=ev.to_date,
                                            properties={
                                                "source": "GDACS",
                                                "event_key": ev.event_key,
                                                "classification": "VALIDATED_HAZARD_POLYGON",
                                                "is_legal_aviation_nfz": False,
                                                "provenance": ev.provenance,
                                                "attribution": GDACS_ATTRIBUTION,
                                            },
                                        )
                                    )

        return {
            "available": True,
            "status": "OK_CACHED" if snap.stale else "OK_LIVE",
            "changed": changed,
            "provenance": snap.provenance,
            "stale": snap.stale,
            "retrieved_at": _iso(snap.retrieved_at),
            "freshness_age_s": round(snap.age_s(eval_time), 2),
            "content_sha256": snap.content_sha256,
            "total_active_events": len(active_events),
            "attribution": snap.attribution,
            "scope_disclaimer": snap.scope_disclaimer,
            "semantic_boundaries": {
                "is_legal_aviation_nfz": False,
                "is_physically_verified_obstacle": False,
                "proves_safe_landing_location": False,
            },
            "advisory_events": advisory_events,
            "stop_suitability_updates": stop_suitability_updates,
            "route_hazard_advisories": route_hazard_advisories,
            "hazard_avoidance_zones": hazard_avoidance_zones,
        }

    @staticmethod
    def apply_impact_to_stops_and_zones(
        impact: dict,
        stops: Sequence[object],
        zones: Sequence[object],
    ) -> dict:
        """Idempotently apply GDACS stop suitability updates and hazard avoidance zones.

        Stop objects are mutated in place. Returns a dict with keys
        ``changed``, ``updated_stop_ids``, ``added_zone_ids`` and ``zones`` (merged list).
        Repeating the call with the same impact yields ``changed=False``.
        """
        sids, zids, merged = GDACSMissionImpactAssessor._apply_impact_tuple(impact, stops, zones)
        return {
            "changed": bool(sids or zids),
            "updated_stop_ids": sids,
            "added_zone_ids": zids,
            "zones": merged,
        }

    @staticmethod
    def _apply_impact_tuple(
        impact: dict,
        stops: Sequence[object],
        zones: Sequence[object],
    ) -> Tuple[List[int], List[str], List[object]]:
        from flightstack.payload import SiteSuitability

        merged_zones = list(zones)
        existing_zone_ids = {
            getattr(z, "zone_id", None)
            for z in merged_zones
            if getattr(z, "zone_id", None) is not None
        }
        modified_stop_ids: List[int] = []
        added_hazard_zone_ids: List[str] = []

        if not isinstance(impact, dict) or not impact.get("available", False):
            return modified_stop_ids, added_hazard_zone_ids, merged_zones

        stops_by_id = {getattr(s, "id", None): s for s in stops if getattr(s, "id", None) is not None}
        for sup in impact.get("stop_suitability_updates") or []:
            sid = sup.get("stop_id")
            new_suit_dict = sup.get("new_suitability")
            if sid not in stops_by_id or not new_suit_dict:
                continue
            stop_obj = stops_by_id[sid]
            cur_suit = SiteSuitability.from_any(getattr(stop_obj, "site_suitability", None))
            cand_suit = SiteSuitability.from_any(new_suit_dict)
            if (
                cur_suit.is_safe_for_landing == cand_suit.is_safe_for_landing
                and cur_suit.surface_condition == cand_suit.surface_condition
                and cur_suit.recommended_mode == cand_suit.recommended_mode
                and cur_suit.provenance == cand_suit.provenance
            ):
                continue
            stop_obj.site_suitability = cand_suit
            if sid not in modified_stop_ids:
                modified_stop_ids.append(sid)

        for hz in impact.get("hazard_avoidance_zones") or []:
            zid = getattr(hz, "zone_id", None)
            if zid and zid in existing_zone_ids:
                continue
            merged_zones.append(hz)
            if zid:
                existing_zone_ids.add(zid)
                added_hazard_zone_ids.append(zid)

        return modified_stop_ids, added_hazard_zone_ids, merged_zones

