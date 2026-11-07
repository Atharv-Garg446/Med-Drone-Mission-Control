"""
test_gdacs_and_provenance.py
============================
Unit, fixture-integration, and live external-API tests for:
  - GDACS disaster event provider (gdacs_provider.py)
  - DigitalSky OpenAPI 3.0.0 flight-plan validation schema (airspace_provider.py)
  - Mappls airspace layer classification (MAP_VISUALIZATION_ONLY)
  - Strict provenance separation (LIVE_EXTERNAL, OFFICIAL_API_VALIDATION,
    MAP_VISUALIZATION_ONLY, SIMULATED_SENSOR, CACHED, SYNTHETIC_FIXTURE).
"""

from __future__ import annotations

import base64
import io
import json
import os
import sys
import tempfile
import unittest
import urllib.error
from datetime import datetime, timedelta, timezone

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from airspace_provider import (
    DIGITALSKY_VALIDATE_URL,
    DigitalSkyIntersectionChecker,
    MapplsAirspaceLayerSpec,
    ScenarioProvider,
    SourceClassification,
    build_digitalsky_validate_request,
    default_digitalsky_http_transport,
)
from config import JAIPUR_SINGLE_UAV
from gdacs_provider import (
    GDACS_ATTRIBUTION,
    CachedGDACSProvider,
    GDACSDataStale,
    GDACSEvent,
    GDACSProvider,
    GDACSUnavailable,
    parse_gdacs_feature_collection,
    parse_gdacs_geometry_collection,
)


SAMPLE_GDACS_FEATURE_COLLECTION = {
    "type": "FeatureCollection",
    "features": [
        {
            "type": "Feature",
            "bbox": [75.5, 26.5, 76.2, 27.2],
            "geometry": {"type": "Point", "coordinates": [75.80, 26.89]},
            "properties": {
                "eventtype": "FL",
                "eventid": 1103001,
                "episodeid": 1,
                "name": "Flood in Rajasthan",
                "description": "Flood in Rajasthan (India)",
                "alertlevel": "Orange",
                "alertscore": 2.0,
                "iscurrent": "true",
                "country": "India",
                "iso3": "IND",
                "fromdate": "2026-10-25T00:00:00",
                "todate": "2026-11-05T12:00:00",
                "datemodified": "2026-11-04T08:00:00",
                "source": "GDACS",
                "url": {
                    "geometry": "https://www.gdacs.org/gdacsapi/api/polygons/getgeometry?eventtype=FL&eventid=1103001&episodeid=1",
                    "report": "https://www.gdacs.org/report.aspx?eventid=1103001&episodeid=1&eventtype=FL",
                },
                "severitydata": {
                    "severity": 2.5,
                    "severitytext": "Magnitude 2.5",
                    "severityunit": "km2",
                },
            },
        },
        # Duplicate episode with newer datemodified to test deduplication
        {
            "type": "Feature",
            "bbox": [75.5, 26.5, 76.2, 27.2],
            "geometry": {"type": "Point", "coordinates": [75.80, 26.89]},
            "properties": {
                "eventtype": "FL",
                "eventid": 1103001,
                "episodeid": 1,
                "name": "Flood in Rajasthan (Updated)",
                "description": "Flood in Rajasthan (India) - Episode 1 Updated",
                "alertlevel": "Red",
                "alertscore": 2.5,
                "iscurrent": "true",
                "country": "India",
                "iso3": "IND",
                "fromdate": "2026-10-25T00:00:00",
                "todate": "2026-11-05T18:00:00",
                "datemodified": "2026-11-05T09:00:00",
                "source": "GDACS",
            },
        },
        # Expired historical event
        {
            "type": "Feature",
            "bbox": [80.0, 12.8, 80.4, 13.2],
            "geometry": {"type": "Point", "coordinates": [80.22, 12.98]},
            "properties": {
                "eventtype": "TC",
                "eventid": 1000999,
                "episodeid": 2,
                "name": "Cyclone Historical",
                "description": "Old cyclone",
                "alertlevel": "Green",
                "alertscore": 1.0,
                "iscurrent": "false",
                "country": "India",
                "iso3": "IND",
                "fromdate": "2025-01-01T00:00:00",
                "todate": "2025-01-05T00:00:00",
                "datemodified": "2025-01-05T00:00:00",
            },
        },
    ],
}

SAMPLE_GDACS_FOOTPRINT_GEOJSON = {
    "type": "FeatureCollection",
    "features": [
        {
            "type": "Feature",
            "properties": {"polygontype": "Affected Area"},
            "geometry": {
                "type": "Polygon",
                "coordinates": [
                    [
                        [75.75, 26.85],
                        [75.85, 26.85],
                        [75.85, 26.92],
                        [75.75, 26.92],
                        [75.75, 26.85],
                    ]
                ],
            },
        }
    ],
}


class _FakeHTTPResponse:
    def __init__(self, status: int, body: bytes):
        self.status = status
        self._body = body

    def read(self) -> bytes:
        return self._body

    def getcode(self) -> int:
        return self.status

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return False


class TestGDACSUnitAndFixtures(unittest.TestCase):
    """Unit and deterministic fixture tests for GDACS parsing, provenance, and caching."""

    def test_parse_deduplication_and_expiry_filtering(self):
        now = datetime(2026, 11, 5, 12, 0, tzinfo=timezone.utc)
        events, dups, expired = parse_gdacs_feature_collection(
            SAMPLE_GDACS_FEATURE_COLLECTION,
            provenance=SourceClassification.SYNTHETIC_FIXTURE,
            filter_expired=True,
            now=now,
        )
        self.assertEqual(dups, 1)
        self.assertEqual(expired, 1)
        self.assertEqual(len(events), 1)
        ev = events[0]
        self.assertEqual(ev.event_key, "FL_1103001_1")
        # Kept the newer duplicate ("Red" alert level)
        self.assertEqual(ev.alert_level, "Red")
        self.assertEqual(ev.name, "Flood in Rajasthan (Updated)")
        self.assertEqual(ev.attribution, GDACS_ATTRIBUTION)
        self.assertEqual(ev.provenance, SourceClassification.SYNTHETIC_FIXTURE)

    def test_parse_geometry_collection_and_spatial_containment(self):
        rings = parse_gdacs_geometry_collection(SAMPLE_GDACS_FOOTPRINT_GEOJSON)
        self.assertEqual(len(rings), 1)
        self.assertEqual(len(rings[0]), 4)  # closed duplicate endpoint stripped

        events, _, _ = parse_gdacs_feature_collection(
            SAMPLE_GDACS_FEATURE_COLLECTION,
            filter_expired=True,
            now=datetime(2026, 11, 5, 12, 0, tzinfo=timezone.utc),
        )
        ev = events[0]
        ev.polygons = rings
        # Gopalpura Relief Camp (26.8850, 75.7930) is inside the polygon [26.85..26.92, 75.75..75.85]
        self.assertTrue(ev.contains_or_near(26.8850, 75.7930, radius_km=1.0))
        # Distant point (Delhi: 28.61, 77.20) is outside polygon, bbox, and 50 km radius
        self.assertFalse(ev.contains_or_near(28.61, 77.20, radius_km=50.0))

    def test_gdacs_provider_http_200_and_footprint_fetch(self):
        def fake_opener(req, timeout=10.0):
            url = req.full_url
            if "getgeometry" in url:
                return _FakeHTTPResponse(
                    200, json.dumps(SAMPLE_GDACS_FOOTPRINT_GEOJSON).encode("utf-8")
                )
            return _FakeHTTPResponse(
                200, json.dumps(SAMPLE_GDACS_FEATURE_COLLECTION).encode("utf-8")
            )

        provider = GDACSProvider(
            event_types=("FL", "EQ", "TC"),
            fetch_footprints_for=("FL_1103001_1",),
            filter_expired=True,
            opener=fake_opener,
        )
        snap = provider.fetch()
        self.assertEqual(snap.provenance, SourceClassification.LIVE_EXTERNAL)
        self.assertFalse(snap.stale)
        self.assertEqual(snap.attribution, GDACS_ATTRIBUTION)
        self.assertEqual(len(snap.events), 1)
        self.assertEqual(len(snap.events[0].polygons), 1)

        # Verify site context combines macro GDACS awareness with local site suitability
        stop1 = JAIPUR_SINGLE_UAV.deliveries[0]
        ctx = snap.assess_site_disaster_context(
            stop1.lat, stop1.lon, site_suitability=stop1.site_suitability
        )
        self.assertEqual(ctx["gdacs_provenance"], SourceClassification.LIVE_EXTERNAL)
        self.assertEqual(ctx["gdacs_attribution"], GDACS_ATTRIBUTION)
        self.assertEqual(ctx["nearby_disaster_count"], 1)
        self.assertEqual(ctx["local_site_suitability"]["recommended_delivery_mode"], "hover")

    def test_gdacs_provider_http_204_empty_result(self):
        def fake_opener_204(req, timeout=10.0):
            return _FakeHTTPResponse(204, b"")

        provider = GDACSProvider(opener=fake_opener_204)
        snap = provider.fetch()
        self.assertEqual(len(snap.events), 0)
        self.assertFalse(snap.stale)
        self.assertEqual(snap.provenance, SourceClassification.LIVE_EXTERNAL)

    def test_gdacs_provider_malformed_json_raises_unavailable(self):
        def fake_opener_bad(req, timeout=10.0):
            return _FakeHTTPResponse(200, b"{not valid json")

        provider = GDACSProvider(opener=fake_opener_bad)
        with self.assertRaises(GDACSUnavailable):
            provider.fetch()

    def test_cached_gdacs_provider_outage_and_staleness(self):
        calls = {"n": 0}

        def flaky_opener(req, timeout=10.0):
            calls["n"] += 1
            if calls["n"] == 1:
                return _FakeHTTPResponse(
                    200, json.dumps(SAMPLE_GDACS_FEATURE_COLLECTION).encode("utf-8")
                )
            raise urllib.error.URLError("simulated network timeout")

        now_holder = [datetime(2026, 11, 5, 12, 0, tzinfo=timezone.utc)]
        with tempfile.TemporaryDirectory() as tmpdir:
            cache_file = os.path.join(tmpdir, "gdacs_cache.json")
            inner = GDACSProvider(opener=flaky_opener, clock=lambda: now_holder[0])
            cached = CachedGDACSProvider(
                inner, max_age_s=3600.0, cache_path=cache_file, clock=lambda: now_holder[0]
            )

            # 1st fetch: live success
            snap1 = cached.fetch()
            self.assertEqual(snap1.provenance, SourceClassification.LIVE_EXTERNAL)
            self.assertFalse(snap1.stale)
            self.assertTrue(os.path.exists(cache_file))

            # 2nd fetch (10 min later, network down): serves CACHED snapshot with stale=True
            now_holder[0] += timedelta(minutes=10)
            snap2 = cached.fetch()
            self.assertEqual(snap2.provenance, SourceClassification.CACHED)
            self.assertTrue(snap2.stale)
            self.assertIn("simulated network timeout", snap2.error or "")
            for ev in snap2.events:
                self.assertEqual(ev.provenance, SourceClassification.CACHED)

            # 3rd fetch (2 hours later > 3600s max_age_s): raises GDACSDataStale
            now_holder[0] += timedelta(hours=2)
            with self.assertRaises(GDACSDataStale):
                cached.fetch()


class TestDigitalSkyOpenAPISchemaAndMappls(unittest.TestCase):
    """Tests for the official DigitalSky OpenAPI 3.0.0 contract and Mappls classification."""

    def test_build_digitalsky_validate_request_schema(self):
        poly = [(26.88, 75.78), (26.88, 75.82), (26.92, 75.82), (26.92, 75.78)]
        start = datetime(2026, 11, 5, 10, 0, tzinfo=timezone.utc)
        end = datetime(2026, 11, 5, 10, 30, tzinfo=timezone.utc)
        req = build_digitalsky_validate_request(
            polygon=poly,
            floor_ft=0.0,
            ceiling_ft=196.85,  # 60 m AGL
            start=start,
            end=end,
            source_app="med-drone-test",
        )
        self.assertEqual(req["url"], DIGITALSKY_VALIDATE_URL)
        self.assertEqual(req["headers"]["X-API-VERSION"], "1")
        self.assertEqual(req["headers"]["source"], "med-drone-test")
        self.assertIn("x-correlation-id", req["headers"])

        body = req["body"]
        self.assertEqual(body["minAltitude"], 0.0)
        self.assertAlmostEqual(body["maxAltitude"], 196.85, places=2)
        self.assertFalse(body["isCircle"])
        self.assertNotIn("radius", body)

        # Decode base64 geom and verify RFC 7946 GeoJSON FeatureCollection with closed [lon, lat] ring
        decoded = json.loads(base64.b64decode(body["geom"]).decode("utf-8"))
        self.assertEqual(decoded["type"], "FeatureCollection")
        ring = decoded["features"][0]["geometry"]["coordinates"][0]
        self.assertEqual(ring[0], [75.78, 26.88])
        self.assertEqual(ring[0], ring[-1])

    def test_digitalsky_ceiling_above_400ft_rejected(self):
        poly = [(26.88, 75.78), (26.88, 75.82), (26.92, 75.82)]
        with self.assertRaises(ValueError):
            build_digitalsky_validate_request(polygon=poly, floor_ft=0.0, ceiling_ft=450.0)

    def test_digitalsky_unauthorised_without_token_is_explicit(self):
        checker = DigitalSkyIntersectionChecker(transport=None, token=None)
        res = checker.validate_route_corridor(
            waypoints=[(26.9127, 75.8010), (26.8850, 75.7930)],
            alt_m_agl=60.0,
        )
        self.assertFalse(res.checked)
        self.assertFalse(res.official_clearance)
        self.assertIsNotNone(res.raw)
        self.assertEqual(res.raw["status"], "UNAUTHORISED_NO_PILOT_TOKEN")
        self.assertEqual(res.raw["headers_redacted"]["x-auth"], "<REDACTED_MISSING>")

    def test_digitalsky_official_http_transport_with_mock_opener(self):
        captured = {}

        def fake_ds_opener(req, timeout=10.0):
            captured["url"] = req.full_url
            captured["headers"] = dict(req.header_items())
            captured["body"] = json.loads(req.data.decode("utf-8"))
            return _FakeHTTPResponse(
                200,
                json.dumps({"status": "GREEN", "redZones": [], "yellowZones": []}).encode("utf-8"),
            )

        checker = DigitalSkyIntersectionChecker(
            transport=lambda **kw: default_digitalsky_http_transport(opener=fake_ds_opener, **kw),
            token="test-session-token-do-not-log",
        )
        res = checker.validate_route_corridor(
            waypoints=[(26.9127, 75.8010), (26.8850, 75.7930)],
            alt_m_agl=60.0,
        )
        self.assertTrue(res.checked)
        self.assertFalse(res.intersects)
        self.assertTrue(res.official_clearance)
        self.assertEqual(captured["url"], DIGITALSKY_VALIDATE_URL)
        self.assertLessEqual(captured["body"]["maxAltitude"], 400.0)

    def test_mappls_airspace_layer_spec_is_visualization_only(self):
        spec = MapplsAirspaceLayerSpec()
        self.assertEqual(spec.classification, SourceClassification.MAP_VISUALIZATION_ONLY)
        self.assertFalse(spec.supports_polygon_export)
        self.assertFalse(spec.supports_autonomous_route_clearance)
        self.assertEqual(len(spec.layers), 4)

    def test_scenario_provider_classified_as_synthetic_fixture(self):
        snap = ScenarioProvider(JAIPUR_SINGLE_UAV).fetch()
        self.assertFalse(snap.official)
        self.assertEqual(snap.provenance_classification, SourceClassification.SYNTHETIC_FIXTURE)
        self.assertEqual(
            snap.metadata()["provenance_classification"],
            SourceClassification.SYNTHETIC_FIXTURE,
        )


class TestDigitalSkyErrorSchemasAndDynamicMonitor(unittest.TestCase):
    """Tests for OpenAPI 3.0 error schemas, DynamicAirspaceMonitor, and GDACSMissionImpactAssessor."""

    def test_digitalsky_hal_validation_error_and_500_error_parsing(self):
        from airspace_provider import parse_digitalsky_error_payload

        hal_400 = json.dumps(
            {
                "message": "Validation failed for FlightPlanAreaValidateRequest",
                "_links": {"self": {"href": "/flightplan/validate/airspace", "templated": "false"}},
                "_embeded": {
                    "errors": [
                        {"message": "maxAltitude must not exceed 400 feet"},
                        {"message": "geom must be valid base64 GeoJSON"},
                    ]
                },
            }
        ).encode("utf-8")
        parsed_400 = parse_digitalsky_error_payload(hal_400, 400)
        self.assertIn("HTTP 400", parsed_400)
        self.assertIn("maxAltitude must not exceed 400 feet", parsed_400)

        err_500 = json.dumps({"code": "DS_ERR_500", "message": "Upstream AAI gateway timeout"}).encode("utf-8")
        parsed_500 = parse_digitalsky_error_payload(err_500, 500)
        self.assertIn("DS_ERR_500", parsed_500)
        self.assertIn("Upstream AAI gateway timeout", parsed_500)

    def test_dynamic_airspace_monitor_tracks_additions_expiry_and_outage(self):
        from airspace_provider import (
            AirspaceSnapshot,
            AirspaceUnavailable,
            AirspaceZone,
            DynamicAirspaceMonitor,
        )

        now = datetime(2026, 11, 6, 12, 0, tzinfo=timezone.utc)
        expiring_zone = AirspaceZone(
            zone_id="TFR_EXPIRING",
            category="temporary_restriction",
            polygon=[(26.88, 75.79), (26.88, 75.80), (26.89, 75.80)],
            valid_from=now - timedelta(hours=2),
            valid_to=now - timedelta(minutes=5),  # Already expired at `now`
        )
        new_zone = AirspaceZone(
            zone_id="TFR_NEW_ACTIVE",
            category="temporary_restriction",
            polygon=[(26.89, 75.80), (26.89, 75.81), (26.90, 75.81)],
            valid_from=now - timedelta(minutes=10),
            valid_to=now + timedelta(hours=1),
        )

        class _MutableProvider:
            def __init__(self):
                self.fail = False

            def fetch(self):
                if self.fail:
                    raise AirspaceUnavailable("Simulated provider outage")
                return AirspaceSnapshot(
                    provider="pollable_test_feed",
                    source="memory://test",
                    zones=[new_zone],
                    retrieved_at=now,
                    content_sha256="hash-v2",
                    official=False,
                )

        prov = _MutableProvider()
        mon = DynamicAirspaceMonitor(prov, clock=lambda: now)
        res = mon.check_for_updates([expiring_zone], now=now)
        self.assertTrue(res["available"])
        self.assertTrue(res["changed"])
        self.assertEqual(res["expired_zone_ids"], ["TFR_EXPIRING"])
        self.assertEqual(res["added_zone_ids"], ["TFR_NEW_ACTIVE"])
        self.assertEqual([z.zone_id for z in res["active_zones"]], ["TFR_NEW_ACTIVE"])

        # Now simulate outage: must NOT clear active restrictions
        prov.fail = True
        res_outage = mon.check_for_updates(res["active_zones"], now=now)
        self.assertFalse(res_outage["available"])
        self.assertEqual(res_outage["status"], "SOURCE_UNAVAILABLE")
        self.assertFalse(res_outage["safe_to_clear_restrictions"])
        self.assertEqual(len(res_outage["active_zones"]), 1)

    def test_gdacs_mission_impact_assessor_advisory_vs_validated_polygon(self):
        from config import Location
        from gdacs_provider import GDACSMissionImpactAssessor

        # Event 1: Coarse centroid only (no polygon) near Jaipur
        # Event 2: Validated flood polygon covering Stop 2 (Jhalana Elevated Medical Post)
        search_doc = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": [75.80, 26.90]},
                    "properties": {
                        "eventtype": "EQ",
                        "eventid": 7001,
                        "episodeid": 1,
                        "name": "Regional Tremor Advisory",
                        "alertlevel": "Green",
                        "alertscore": 1.0,
                        "iscurrent": "true",
                        "country": "India",
                        "iso3": "IND",
                    },
                },
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [
                            [
                                [75.810, 26.880],
                                [75.825, 26.880],
                                [75.825, 26.895],
                                [75.810, 26.895],
                                [75.810, 26.880],
                            ]
                        ],
                    },
                    "properties": {
                        "eventtype": "FL",
                        "eventid": 7002,
                        "episodeid": 1,
                        "name": "Jaipur East Basin Inundation",
                        "alertlevel": "Red",
                        "alertscore": 2.5,
                        "iscurrent": "true",
                        "country": "India",
                        "iso3": "IND",
                    },
                },
            ],
        }
        provider = GDACSProvider(
            opener=lambda req, timeout=15.0: _FakeHTTPResponse(
                200, json.dumps(search_doc).encode("utf-8")
            ),
            fetch_footprints_for=(),
        )
        assessor = GDACSMissionImpactAssessor(
            provider,
            relevance_radius_km=250.0,
            fetch_missing_geometries=False,
            avoid_red_hazard_polygons=True,
        )
        stop2 = Location(
            2,
            "Jhalana Elevated Medical Post",
            26.8890,
            75.8180,
            demand_kg=1.5,
            site_suitability={
                "is_safe_for_landing": True,
                "surface_condition": "clear",
                "recommended_mode": "land",
            },
        )
        leg_paths = [[(26.9127, 75.8010), (26.8890, 75.8180)]]
        impact = assessor.evaluate_mission_impact(
            stops=[stop2], leg_paths=leg_paths, home=(26.9127, 75.8010)
        )
        self.assertTrue(impact["available"])
        self.assertEqual(impact["attribution"], GDACS_ATTRIBUTION)
        self.assertFalse(impact["semantic_boundaries"]["is_legal_aviation_nfz"])
        self.assertFalse(impact["semantic_boundaries"]["proves_safe_landing_location"])

        # Event 7001 has no polygon -> retained strictly as ADVISORY_REGIONAL_AWARENESS
        self.assertEqual(len(impact["advisory_events"]), 1)
        self.assertEqual(impact["advisory_events"][0]["event_key"], "EQ_7001_1")
        self.assertEqual(impact["advisory_events"][0]["classification"], "ADVISORY_REGIONAL_AWARENESS")

        # Event 7002 has a validated polygon covering Stop 2 -> triggers DOWNGRADE_LAND_TO_HOVER
        self.assertEqual(len(impact["stop_suitability_updates"]), 1)
        sup = impact["stop_suitability_updates"][0]
        self.assertEqual(sup["stop_id"], 2)
        self.assertEqual(sup["planner_action"], "DOWNGRADE_LAND_TO_HOVER")
        self.assertEqual(sup["new_suitability"]["recommended_mode"], "hover")
        self.assertFalse(sup["new_suitability"]["is_safe_for_landing"])
        self.assertEqual(len(impact["hazard_avoidance_zones"]), 1)


class TestLiveGDACSExternalAPI(unittest.TestCase):
    """Live external API verification against https://www.gdacs.org/gdacsapi."""

    def test_live_gdacs_api_fetch(self):
        provider = GDACSProvider(
            event_types=("FL", "EQ", "TC"),
            lookback_days=35,
            timeout_s=15.0,
        )
        try:
            snap = provider.fetch()
        except GDACSUnavailable as exc:
            self.skipTest(f"Live GDACS endpoint unreachable in current environment: {exc}")

        self.assertEqual(snap.provenance, SourceClassification.LIVE_EXTERNAL)
        self.assertFalse(snap.stale)
        self.assertEqual(snap.attribution, GDACS_ATTRIBUTION)
        self.assertGreater(len(snap.events), 0, "Expected at least 1 global FL/EQ/TC event in 35 days")
        meta = snap.metadata()
        self.assertEqual(meta["provenance"], SourceClassification.LIVE_EXTERNAL)
        self.assertEqual(meta["attribution"], GDACS_ATTRIBUTION)


if __name__ == "__main__":
    unittest.main()
