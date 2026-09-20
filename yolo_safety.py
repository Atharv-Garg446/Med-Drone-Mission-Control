"""
yolo_safety.py
==============
Landing-zone safety check: before a drone commits to landing, analyzes a photo 
of the drop zone to ensure it's clear of people/obstacles. Uses a pretrained 
YOLOv8 model (yolov8n.pt).

Split into two layers:
  1. `classify_detections()` -- Pure decision logic. Given a list of detections, 
     decides SAFE/UNSAFE. Independent of YOLO and fully unit-testable offline.
  2. `run_yolo_on_image()` -- The model call. Loads the pretrained checkpoint 
     via `ultralytics` and runs inference.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional

# COCO classes we treat as "landing zone is NOT clear". `person` is the
# obvious one (never land on/near a person); the others are physical
# obstructions a small drone genuinely cannot land through.
UNSAFE_CLASSES = {"person", "car", "motorcycle", "bicycle", "dog", "truck", "bus"}
CONFIDENCE_THRESHOLD = 0.35  # detections below this are treated as noise, not a real object


@dataclass
class SafetyResult:
    location_name: str
    is_safe: bool
    reason: str
    detections_considered: List[Tuple[str, float]]


def classify_detections(location_name: str, detections: List[Tuple[str, float]]) -> SafetyResult:
    """Pure decision logic, no YOLO/model dependency at all.

    `detections` is a list of (class_name, confidence) tuples -- exactly
    the shape YOLOv8 produces per-box, just already unpacked so this
    function doesn't need to know anything about ultralytics' Results
    object. That's what makes it trivially unit-testable (see
    tests/test_core_logic.py: FAKE_DETECTIONS_CLEAR / FAKE_DETECTIONS_BLOCKED).
    """
    considered = [(cls, conf) for cls, conf in detections if conf >= CONFIDENCE_THRESHOLD]
    flagged = [(cls, conf) for cls, conf in considered if cls in UNSAFE_CLASSES]

    if flagged:
        worst = max(flagged, key=lambda t: t[1])
        reason = f"detected '{worst[0]}' in landing zone (confidence {worst[1]:.2f})"
        return SafetyResult(location_name, is_safe=False, reason=reason, detections_considered=considered)

    return SafetyResult(location_name, is_safe=True, reason="landing zone clear", detections_considered=considered)


def run_yolo_on_image(image_path: str, location_name: str,
                       model_path: str = "yolov8n.pt") -> Optional[SafetyResult]:
    """Run the real pretrained YOLOv8 model on a real image file.

    Returns None (with an explanatory print) if `ultralytics` isn't
    installed, or if loading the model fails (e.g. no internet to fetch
    the checkpoint the first time). Every caller in this project treats
    None as "safety check unavailable -- flag for manual visual
    confirmation before landing", never as "assumed safe".
    """
    try:
        from ultralytics import YOLO
    except ImportError:
        print(f"[info] ultralytics not installed -- skipping the automated safety "
              f"check for '{location_name}' (`pip install ultralytics` to enable it). "
              f"Recommend a manual visual check before landing here.")
        return None

    try:
        model = YOLO(model_path)  # auto-downloads yolov8n.pt on first run if missing
        results = model(image_path, verbose=False)

        detections: List[Tuple[str, float]] = []
        for result in results:
            names = result.names
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                detections.append((names[class_id], confidence))

        return classify_detections(location_name, detections)

    except Exception as e:
        print(f"[warning] YOLOv8 inference failed for '{location_name}' ({e}). "
              f"This usually means the pretrained checkpoint couldn't be "
              f"downloaded (no internet). Recommend a manual visual check "
              f"before landing here.")
        return None


def check_landing_zones(routes, locations, out_dir_hint: str = "sample_data/") -> List[SafetyResult]:
    """Convenience wrapper for main.py: for every location with a
    `landing_zone_image` set in config.py, run the safety check and
    collect results. Locations without an image configured are skipped
    (nothing to check), not flagged unsafe."""
    results = []
    all_ids_in_routes = {node for route in routes for node in route}

    for loc in locations:
        if loc.id not in all_ids_in_routes or loc.landing_zone_image is None:
            continue
        image_path = out_dir_hint.rstrip("/") + "/" + loc.landing_zone_image
        result = run_yolo_on_image(image_path, loc.name)
        if result is not None:
            results.append(result)
    return results
