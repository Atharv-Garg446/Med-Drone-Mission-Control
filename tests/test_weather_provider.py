#!/usr/bin/env python3
"""
tests/test_weather_provider.py
==============================
Unit tests for flightstack/weather_provider.py.
All standard tests use mocked responses or synthetic fixtures (no network required).
Live network test is gated behind RUN_LIVE_WEATHER_TESTS=1 environment flag.
"""

from datetime import datetime, timezone
import json
import math
import os
import unittest
from unittest.mock import MagicMock, patch
import urllib.error

from flightstack.weather_provider import (
    ATTRIBUTION,
    HourlyWeatherRecord,
    WeatherFetchError,
    WeatherForecast,
    bearing_deg,
    fetch_open_meteo_forecast,
    parse_iso_to_epoch,
    parse_open_meteo_response,
)


SAMPLE_OPEN_METEO_JSON = {
    "latitude": 26.9124,
    "longitude": 75.7873,
    "generationtime_ms": 0.05,
    "utc_offset_seconds": 0,
    "timezone": "UTC",
    "timezone_abbreviation": "UTC",
    "elevation": 435.0,
    "hourly_units": {
        "time": "iso8601",
        "wind_speed_10m": "m/s",
        "wind_direction_10m": "°",
        "wind_gusts_10m": "m/s",
        "temperature_2m": "°C",
        "precipitation": "mm"
    },
    "hourly": {
        "time": [
            "2026-11-02T10:00",
            "2026-11-02T11:00",
            "2026-11-02T12:00",
            "2026-11-02T13:00"
        ],
        "wind_speed_10m": [5.0, 7.5, 10.0, 12.0],
        "wind_direction_10m": [90.0, 180.0, 270.0, 0.0],  # E, S, W, N
        "wind_gusts_10m": [8.0, 11.0, 15.0, 18.0],
        "temperature_2m": [28.5, 30.0, 31.2, 29.8],
        "precipitation": [0.0, 0.0, 0.5, 2.1]
    }
}


class TestWeatherProvider(unittest.TestCase):

    def test_parse_iso_to_epoch(self):
        t_epoch = parse_iso_to_epoch("2026-11-02T12:00")
        dt = datetime.fromtimestamp(t_epoch, tz=timezone.utc)
        self.assertEqual(dt.year, 2026)
        self.assertEqual(dt.month, 11)
        self.assertEqual(dt.day, 2)
        self.assertEqual(dt.hour, 12)
        self.assertEqual(dt.minute, 0)

    def test_parse_open_meteo_response(self):
        forecast = parse_open_meteo_response(SAMPLE_OPEN_METEO_JSON, retrieved_epoch_s=1793620800.0)
        self.assertAlmostEqual(forecast.latitude, 26.9124)
        self.assertAlmostEqual(forecast.longitude, 75.7873)
        self.assertIn("Open-Meteo", forecast.attribution)
        self.assertEqual(len(forecast.hourly_records), 4)

        rec0 = forecast.hourly_records[0]
        self.assertEqual(rec0.time_iso, "2026-11-02T10:00")
        self.assertAlmostEqual(rec0.wind_speed_ms, 5.0)
        self.assertAlmostEqual(rec0.wind_direction_deg, 90.0)

    def test_target_time_selection(self):
        forecast = parse_open_meteo_response(SAMPLE_OPEN_METEO_JSON)
        # Query time close to 11:15 UTC -> should select 11:00 record
        t_query = parse_iso_to_epoch("2026-11-02T11:15")
        rec = forecast.get_record_for_time(t_query)
        self.assertIsNotNone(rec)
        self.assertEqual(rec.time_iso, "2026-11-02T11:00")
        self.assertAlmostEqual(rec.wind_speed_ms, 7.5)

        # Query time close to 12:45 UTC -> should select 13:00 record
        t_query2 = parse_iso_to_epoch("2026-11-02T12:45")
        rec2 = forecast.get_record_for_time(t_query2)
        self.assertIsNotNone(rec2)
        self.assertEqual(rec2.time_iso, "2026-11-02T13:00")

    def test_meteorological_wind_conventions_and_components(self):
        """Test headwind and tailwind projection with standard meteorological conventions."""
        # Record at 13:00 has wind_direction = 0.0 (North wind: blowing FROM North towards South)
        # Record wind speed = 12.0 m/s
        forecast = parse_open_meteo_response(SAMPLE_OPEN_METEO_JSON)
        t_1300 = parse_iso_to_epoch("2026-11-02T13:00")

        # Scenario A: Drone flies North (heading 0 deg) into a North wind -> HEADWIND of ~ -12 m/s
        v_along, v_cross, _ = forecast.wind_components(leg_bearing_deg=0.0, target_epoch_s=t_1300)
        self.assertAlmostEqual(v_along, -12.0, delta=0.1)
        self.assertAlmostEqual(v_cross, 0.0, delta=0.1)

        # Scenario B: Drone flies South (heading 180 deg) with a North wind -> TAILWIND of ~ +12 m/s
        v_along, v_cross, _ = forecast.wind_components(leg_bearing_deg=180.0, target_epoch_s=t_1300)
        self.assertAlmostEqual(v_along, 12.0, delta=0.1)
        self.assertAlmostEqual(v_cross, 0.0, delta=0.1)

        # Scenario C: Drone flies East (heading 90 deg) with a North wind -> CROSSWIND
        v_along, v_cross, _ = forecast.wind_components(leg_bearing_deg=90.0, target_epoch_s=t_1300)
        self.assertAlmostEqual(v_along, 0.0, delta=0.1)
        # Wind blowing towards South (180 deg) is from drone's left (port to starboard) -> crosswind
        self.assertNotEqual(v_cross, 0.0)

    def test_effective_ground_speed_and_crabbing(self):
        forecast = parse_open_meteo_response(SAMPLE_OPEN_METEO_JSON)
        t_1000 = parse_iso_to_epoch("2026-11-02T10:00")
        # At 10:00: Wind is 5.0 m/s FROM 90 deg (East wind, blowing towards 270 deg West)
        # Airspeed = 10.0 m/s

        # 1. Drone flying West (270 deg): direct tailwind of 5 m/s -> ground speed ~ 15 m/s
        vg, crab, _ = forecast.effective_ground_speed(airspeed_ms=10.0, leg_bearing_deg=270.0, target_epoch_s=t_1000)
        self.assertAlmostEqual(vg, 15.0, delta=0.2)
        self.assertAlmostEqual(crab, 0.0, delta=0.1)

        # 2. Drone flying East (90 deg): direct headwind of 5 m/s -> ground speed ~ 5 m/s
        vg_head, crab_head, _ = forecast.effective_ground_speed(airspeed_ms=10.0, leg_bearing_deg=90.0, target_epoch_s=t_1000)
        self.assertAlmostEqual(vg_head, 5.0, delta=0.2)
        self.assertAlmostEqual(crab_head, 0.0, delta=0.1)

        # 3. Drone flying North (0 deg): pure crosswind of 5 m/s
        # Forward component: sqrt(10^2 - 5^2) = sqrt(75) ~ 8.66 m/s
        vg_cross, crab_cross, _ = forecast.effective_ground_speed(airspeed_ms=10.0, leg_bearing_deg=0.0, target_epoch_s=t_1000)
        self.assertAlmostEqual(vg_cross, 8.66, delta=0.2)
        self.assertAlmostEqual(abs(crab_cross), 30.0, delta=0.5)

    def test_bearing_deg(self):
        # North from (0, 0) to (1, 0) is 0 deg
        self.assertAlmostEqual(bearing_deg((0.0, 0.0), (1.0, 0.0)), 0.0, delta=0.1)
        # East from (0, 0) to (0, 1) is 90 deg
        self.assertAlmostEqual(bearing_deg((0.0, 0.0), (0.0, 1.0)), 90.0, delta=0.1)
        # South from (1, 0) to (0, 0) is 180 deg
        self.assertAlmostEqual(bearing_deg((1.0, 0.0), (0.0, 0.0)), 180.0, delta=0.1)

    def test_missing_or_sparse_fields_handled_gracefully(self):
        sparse_json = {
            "latitude": 28.0,
            "longitude": 77.0,
            "hourly": {
                "time": ["2026-11-02T10:00"],
                "wind_speed_10m": [None],
                "wind_direction_10m": [None]
            }
        }
        forecast = parse_open_meteo_response(sparse_json)
        self.assertEqual(len(forecast.hourly_records), 1)
        self.assertEqual(forecast.hourly_records[0].wind_speed_ms, 0.0)
        self.assertEqual(forecast.hourly_records[0].wind_direction_deg, 0.0)

    @patch("urllib.request.urlopen")
    def test_fetch_open_meteo_forecast_mocked(self, mock_urlopen):
        mock_resp = MagicMock()
        mock_resp.status = 200
        mock_resp.read.return_value = json.dumps(SAMPLE_OPEN_METEO_JSON).encode("utf-8")
        mock_urlopen.return_value.__enter__.return_value = mock_resp

        forecast = fetch_open_meteo_forecast(26.9124, 75.7873)
        self.assertEqual(len(forecast.hourly_records), 4)
        self.assertEqual(forecast.source, "Open-Meteo Weather Forecast API")

    @patch("urllib.request.urlopen")
    def test_fetch_open_meteo_retry_and_failure(self, mock_urlopen):
        mock_urlopen.side_effect = urllib.error.URLError("Connection refused")
        with self.assertRaises(WeatherFetchError):
            fetch_open_meteo_forecast(26.9124, 75.7873, max_retries=1)

    @unittest.skipUnless(os.environ.get("RUN_LIVE_WEATHER_TESTS"), "Live API test disabled (opt-in only)")
    def test_live_open_meteo_smoke(self):
        # Real API smoke test for Jaipur coordinates
        forecast = fetch_open_meteo_forecast(26.9124, 75.7873, timeout_s=10.0)
        self.assertGreater(len(forecast.hourly_records), 0)
        self.assertIn("open-meteo.com", forecast.attribution.lower())


if __name__ == "__main__":
    unittest.main()

