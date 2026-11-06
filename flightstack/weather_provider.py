#!/usr/bin/env python3
"""
flightstack/weather_provider.py
===============================
Integration with Open-Meteo Weather API for meteorological wind forecasts.

Provides:
  - Open-Meteo forecast fetching with retries, timeout handling, and provenance metadata.
  - Meteorological wind convention conversion (wind-from direction -> along-track headwind/tailwind).
  - Forecast hour interpolation / nearest-hour lookup based on estimated leg flight time.
  - Aerodynamic wind triangle ground speed calculation with crab-angle correction.
  - Attribution: "Weather data by Open-Meteo.com" (CC BY 4.0).
"""

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple
import urllib.error
import urllib.request


ATTRIBUTION = "Weather data by Open-Meteo.com (CC BY 4.0, https://open-meteo.com/)"


class WeatherFetchError(RuntimeError):
    """Raised when weather forecast retrieval fails after retries."""
    pass


@dataclass
class HourlyWeatherRecord:
    """Weather conditions for a specific 1-hour UTC window."""
    time_iso: str
    time_epoch_s: float
    wind_speed_ms: float
    wind_direction_deg: float      # Meteorological convention: direction wind is blowing FROM (0=N, 90=E, 180=S, 270=W)
    wind_gusts_ms: float = 0.0
    temperature_c: float = 20.0
    precipitation_mm: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "time_iso": self.time_iso,
            "time_epoch_s": self.time_epoch_s,
            "wind_speed_ms": self.wind_speed_ms,
            "wind_direction_deg": self.wind_direction_deg,
            "wind_gusts_ms": self.wind_gusts_ms,
            "temperature_c": self.temperature_c,
            "precipitation_mm": self.precipitation_mm,
        }


@dataclass
class WeatherForecast:
    """Full retrieved forecast snapshot with metadata and provenance."""
    latitude: float
    longitude: float
    retrieved_at_utc: str
    retrieved_at_epoch_s: float
    source: str = "Open-Meteo Weather Forecast API"
    attribution: str = ATTRIBUTION
    timezone: str = "UTC"
    elevation_m: float = 0.0
    hourly_records: List[HourlyWeatherRecord] = field(default_factory=list)
    raw_response: Optional[Dict[str, Any]] = None

    def get_record_for_time(self, target_epoch_s: Optional[float] = None) -> Optional[HourlyWeatherRecord]:
        """Find the hourly record closest to target_epoch_s (defaults to retrieval time)."""
        if not self.hourly_records:
            return None
        target = target_epoch_s if target_epoch_s is not None else self.retrieved_at_epoch_s
        return min(self.hourly_records, key=lambda r: abs(r.time_epoch_s - target))

    def wind_components(
        self,
        leg_bearing_deg: float,
        target_epoch_s: Optional[float] = None
    ) -> Tuple[float, float, HourlyWeatherRecord]:
        """Compute along-track and cross-track wind components for a flight heading.

        Args:
            leg_bearing_deg: Flight heading direction in degrees [0, 360).
            target_epoch_s: Target time in epoch seconds for forecast lookup.

        Returns:
            (v_along_ms, v_cross_ms, record):
                v_along_ms: Along-track component (+ is tailwind, - is headwind).
                v_cross_ms: Cross-track component (+ is from port to starboard, - starboard to port).
                record: The HourlyWeatherRecord used for the calculation.
        """
        rec = self.get_record_for_time(target_epoch_s)
        if rec is None:
            return 0.0, 0.0, HourlyWeatherRecord("", 0.0, 0.0, 0.0)

        # Wind direction is meteorological: direction FROM which wind blows.
        # Vector direction TOWARDS which wind blows is:
        wind_towards_deg = (rec.wind_direction_deg + 180.0) % 360.0

        # Angle delta between wind motion and vehicle flight direction
        delta_rad = math.radians(wind_towards_deg - leg_bearing_deg)

        # Along-track (parallel): positive = tailwind, negative = headwind
        v_along = rec.wind_speed_ms * math.cos(delta_rad)
        # Cross-track (perpendicular)
        v_cross = rec.wind_speed_ms * math.sin(delta_rad)

        return round(v_along, 2), round(v_cross, 2), rec

    def effective_ground_speed(
        self,
        airspeed_ms: float,
        leg_bearing_deg: float,
        target_epoch_s: Optional[float] = None,
        min_ground_speed_ms: float = 1.0
    ) -> Tuple[float, float, HourlyWeatherRecord]:
        """Calculate effective ground speed using the aerodynamic wind triangle.

        Accounts for crab angle needed to maintain course in crosswind:
            V_along = V_w * cos(delta)
            V_cross = V_w * sin(delta)
            V_crab_eff = sqrt(max(0, V_air^2 - V_cross^2))
            V_ground = max(min_speed, V_crab_eff + V_along)

        Returns:
            (ground_speed_ms, crab_angle_deg, record)
        """
        v_along, v_cross, rec = self.wind_components(leg_bearing_deg, target_epoch_s)

        if airspeed_ms <= 0.0:
            return min_ground_speed_ms, 0.0, rec

        # Check if crosswind exceeds airspeed (cannot hold heading)
        if abs(v_cross) >= airspeed_ms:
            # Extreme crosswind; clamp crab and forward speed
            crab_angle_deg = math.copysign(89.0, -v_cross)
            v_ground = max(min_ground_speed_ms, v_along)
        else:
            crab_rad = math.asin(-v_cross / airspeed_ms)
            crab_angle_deg = math.degrees(crab_rad)
            v_forward_air = airspeed_ms * math.cos(crab_rad)
            v_ground = max(min_ground_speed_ms, v_forward_air + v_along)

        return round(v_ground, 2), round(crab_angle_deg, 1), rec

    def to_dict(self) -> Dict[str, Any]:
        return {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "retrieved_at_utc": self.retrieved_at_utc,
            "retrieved_at_epoch_s": self.retrieved_at_epoch_s,
            "source": self.source,
            "attribution": self.attribution,
            "timezone": self.timezone,
            "elevation_m": self.elevation_m,
            "records_count": len(self.hourly_records),
            "hourly_records": [r.to_dict() for r in self.hourly_records],
        }


def parse_iso_to_epoch(iso_str: str) -> float:
    """Parse ISO 8601 string (e.g., '2026-11-02T12:00' or with Z) to UTC epoch seconds."""
    clean = iso_str.replace("Z", "")
    if len(clean) == 16:  # 'YYYY-MM-DDTHH:MM'
        dt = datetime.strptime(clean, "%Y-%m-%dT%H:%M").replace(tzinfo=timezone.utc)
    elif len(clean) == 19:  # 'YYYY-MM-DDTHH:MM:SS'
        dt = datetime.strptime(clean, "%Y-%m-%dT%H:%M:%S").replace(tzinfo=timezone.utc)
    else:
        dt = datetime.fromisoformat(clean).replace(tzinfo=timezone.utc)
    return dt.timestamp()


def parse_open_meteo_response(data: Dict[str, Any], retrieved_epoch_s: Optional[float] = None) -> WeatherForecast:
    """Parse raw JSON response from Open-Meteo into WeatherForecast dataclass."""
    now_epoch = retrieved_epoch_s if retrieved_epoch_s is not None else time.time()
    now_iso = datetime.fromtimestamp(now_epoch, tz=timezone.utc).isoformat()

    lat = float(data.get("latitude", 0.0))
    lon = float(data.get("longitude", 0.0))
    elev = float(data.get("elevation", 0.0))
    tz = str(data.get("timezone", "UTC"))

    hourly = data.get("hourly", {})
    times = hourly.get("time", [])
    speeds = hourly.get("wind_speed_10m", [])
    dirs = hourly.get("wind_direction_10m", [])
    gusts = hourly.get("wind_gusts_10m", [0.0] * len(times))
    temps = hourly.get("temperature_2m", [20.0] * len(times))
    precips = hourly.get("precipitation", [0.0] * len(times))

    records: List[HourlyWeatherRecord] = []
    for i, t_str in enumerate(times):
        epoch = parse_iso_to_epoch(t_str)
        spd = float(speeds[i]) if i < len(speeds) and speeds[i] is not None else 0.0
        wdir = float(dirs[i]) if i < len(dirs) and dirs[i] is not None else 0.0
        gst = float(gusts[i]) if i < len(gusts) and gusts[i] is not None else 0.0
        tmp = float(temps[i]) if i < len(temps) and temps[i] is not None else 20.0
        prc = float(precips[i]) if i < len(precips) and precips[i] is not None else 0.0

        records.append(HourlyWeatherRecord(
            time_iso=t_str,
            time_epoch_s=epoch,
            wind_speed_ms=spd,
            wind_direction_deg=wdir,
            wind_gusts_ms=gst,
            temperature_c=tmp,
            precipitation_mm=prc
        ))

    return WeatherForecast(
        latitude=lat,
        longitude=lon,
        retrieved_at_utc=now_iso,
        retrieved_at_epoch_s=now_epoch,
        source="Open-Meteo Weather Forecast API",
        attribution=ATTRIBUTION,
        timezone=tz,
        elevation_m=elev,
        hourly_records=records,
        raw_response=data
    )


def fetch_open_meteo_forecast(
    latitude: float,
    longitude: float,
    timeout_s: float = 8.0,
    max_retries: int = 2,
    forecast_days: int = 2,
) -> WeatherForecast:
    """Fetch live meteorological weather forecast from Open-Meteo API.

    No API key required. Attribution required under CC BY 4.0.
    """
    url = (
        f"https://api.open-meteo.com/v1/forecast?"
        f"latitude={latitude:.4f}&longitude={longitude:.4f}&"
        f"hourly=wind_speed_10m,wind_direction_10m,wind_gusts_10m,temperature_2m,precipitation&"
        f"wind_speed_unit=ms&forecast_days={forecast_days}&timezone=UTC"
    )

    req = urllib.request.Request(
        url,
        headers={"User-Agent": "MedDroneMissionControl/2.0 (Research Emergency Logistics Prototype)"}
    )

    last_error: Optional[Exception] = None
    for attempt in range(max_retries + 1):
        try:
            with urllib.request.urlopen(req, timeout=timeout_s) as resp:
                if resp.status != 200:
                    raise WeatherFetchError(f"Open-Meteo API returned status {resp.status}")
                raw_bytes = resp.read()
                data = json.loads(raw_bytes.decode("utf-8"))
                return parse_open_meteo_response(data)
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, json.JSONDecodeError) as e:
            last_error = e
            if attempt < max_retries:
                time.sleep(0.5 * (attempt + 1))
                continue

    raise WeatherFetchError(f"Failed to fetch Open-Meteo forecast for ({latitude}, {longitude}): {last_error}")


def bearing_deg(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculate geographic forward azimuth from point 1 (lat, lon) to point 2 (lat, lon) in degrees [0, 360)."""
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    az = math.degrees(math.atan2(y, x))
    return (az + 360.0) % 360.0

