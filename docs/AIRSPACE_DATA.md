# Indian airspace data: what is verifiably available

This was researched in October 2026. Re-verify before relying on it.

## Digital Sky (DGCA / AAI): digitalsky.aai.aero

**What I verified directly** (from the public web client served to anonymous users):

* The portal is an Angular single-page application.
* It calls two public-API paths:
  * `/public-api/v1/hdsbpm/get-token`
  * `/public-api/v1/hdsbpm/check-flight-plan-intersection`
* For the intersection call, the client takes a **user-supplied token**,
  encrypts it client-side, and sends it in custom headers along with a request checksum.
  Request bodies are also encrypted client-side.
* The bundle contains no public endpoint for downloading zone geometry
  (red/yellow/green polygons). The map tiles come from a third-party map provider.

**What I could not verify:**

* An official, publicly downloadable request/response schema (OpenAPI) for the
  intersection API. Secondary sources say it exists for registered users, but
  I couldn't retrieve it without an account.
* Rate limits, terms of use for automated access, and update semantics.
* Any source of temporary flight restrictions in machine-readable form.

**Decision:**

* `DigitalSkyIntersectionChecker` (in `airspace_provider.py`) is
  **gated by credentials and the official spec**. It needs `DIGITALSKY_TOKEN` and a `transport`
  callable that implements the officially documented request. Without both, it
  returns `checked=False`, and callers must treat that as "airspace unverified".
* I did **not** reimplement the web client's client-side encryption or headers.
  Doing that would mean copying an undocumented interface and its access
  controls, which the project rules forbid.
* The intersection check only answers "does this flight area intersect a
  restricted zone?" It can validate a mission, but A* still needs polygons.

## Polygons for A*

There is currently no authorised, documented source of Digital Sky zone geometry
available to this project. The planner therefore takes polygons from:

| Provider | Category label | Official? |
|---|---|---|
| `ScenarioProvider` (config.py boxes) | `scenario` | No: simplified stand-ins |
| `GeoJSONFileProvider` | from file | Only if the file is from an authorised source and `official=True` is set deliberately |
| Mid-flight demo zone | `scenario` / "simulated_temporary_restriction" | No |

Every snapshot records the provider, source path/URL, retrieval time, the
source's update time when it supplies one, a SHA-256 of the content, the version and the
staleness. `CachedAirspaceProvider` returns the last-known-good data marked
`stale=True` during an outage, and raises `AirspaceDataStale` once it is older
than the configured limit.

## Category policy

`AirspaceSnapshot.planning_polygons()` blocks `red`, `yellow`,
`temporary_restriction` and `scenario` by default. Yellow is blocked because
the planner has no record of ATC permission. `green` never blocks. Zones
with `valid_from`/`valid_to` are only applied inside their validity window.

## Path to official integration

1. Register on Digital Sky and get API access and the official spec.
2. Implement `transport` from that spec, with the token from the environment
   (never committed to the repo).
3. Record a real response as a test fixture, after checking the terms of use allow it.
4. Run the check before upload, and again after every re-plan. Refuse to launch, or require
   operator confirmation, when the result is `checked=False`.
