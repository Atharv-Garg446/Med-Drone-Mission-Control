# ADR-003: Airspace & Disaster Data Provenance (GDACS, DigitalSky OpenAPI 3.0, and Mappls)

## Status
Accepted (2026-11-05)

## Context
An autonomous medical-delivery system operating in India must distinguish between:
1. Live macro-level disaster intelligence (floods, earthquakes, tropical cyclones).
2. Official regulatory airspace validation (DGCA / AAI DigitalSky).
3. Browser visualization layers (Mappls Airspace Web SDK).
4. Deterministic test fixtures (`JAIPUR_AIRPORT_ZONE`, `CITY_PALACE_ZONE`, `TFR_POPUP_4D_01`).

Conflating local polygons or browser map tiles with official airspace clearance or live disaster data is technically indefensible.

## Decision
1. **Integrate the live public GDACS REST API (`gdacs_provider.py`):**
   - Query `https://www.gdacs.org/gdacsapi/api/Events/geteventlist/SEARCH` and `/api/polygons/getgeometry`, deduplicate episodes, filter expired alerts, cache to `output/gdacs_cache_latest.json` (`CachedGDACSProvider`), and include mandatory attribution (`"Global Disaster Awareness and Coordination System, GDACS."`) and scope boundaries.
2. **Implement the exact DigitalSky OpenAPI 3.0.0 contract (`airspace_provider.py`):**
   - Verified against `docs/external_specs/digitalsky_interactive_maps_api_v1.0.yml` (`POST https://digitalsky.dgca.gov.in/digital-sky/flightplan/validate/airspace`, headers `X-API-VERSION: 1`, `source`, `x-auth`, `x-correlation-id`, base64 GeoJSON `geom`, `maxAltitude <= 400 ft`).
   - Keep live execution strictly gated on `DIGITALSKY_AUTH_TOKEN` / `DIGITALSKY_TOKEN`. When absent, report `checked=False`, `official_clearance=False`, `status="UNAUTHORISED_NO_PILOT_TOKEN"`.
3. **Classify Mappls Airspace (`MapplsAirspaceLayerSpec`) as `MAP_VISUALIZATION_ONLY`:**
   - Document its 4 named Web SDK layers and `supports_polygon_export=False`.
4. **Enforce `SourceClassification` tags across all logs and snapshots:**
   - `LIVE_EXTERNAL`, `OFFICIAL_API_VALIDATION`, `MAP_VISUALIZATION_ONLY`, `SIMULATED_SENSOR`, `CACHED`, and `SYNTHETIC_FIXTURE`.

## Consequences
- Every SITL mission log (`output/sitl_run_<timestamp>.jsonl`) contains an auditable record of live GDACS disaster alerts, DigitalSky validation status, Mappls classification, and synthetic TFR provenance.

