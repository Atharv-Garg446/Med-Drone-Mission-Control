"""
generate_sample_images.py
==========================
Creates two tiny placeholder "landing zone" photos using only Pillow (no
network, no YOLO): one showing an empty patch of ground (should be judged
SAFE), and one with a crude human-silhouette shape standing in the middle
of it (should be judged UNSAFE). These stand in for real drone camera
stills so the pipeline has something to point yolo_safety.py at.

These are obviously not photorealistic -- they exist to make the pipeline
runnable end-to-end without a real drone camera, not to actually fool or
validate YOLOv8's detection accuracy. See README for how yolo_safety.py's
DECISION LOGIC is separately unit-tested against known detection inputs,
which is the part of the safety check we're actually vouching for.
"""

import os
from PIL import Image, ImageDraw


def _draw_ground(draw, size):
    w, h = size
    draw.rectangle([0, 0, w, h], fill=(120, 150, 90))       # grass-green field
    draw.ellipse([w * 0.3, h * 0.35, w * 0.7, h * 0.65], fill=(160, 160, 150))  # landing pad


def generate_clear_landing_zone(path: str, size=(320, 240)):
    img = Image.new("RGB", size)
    draw = ImageDraw.Draw(img)
    _draw_ground(draw, size)
    img.save(path)
    return path


def generate_blocked_landing_zone(path: str, size=(320, 240)):
    img = Image.new("RGB", size)
    draw = ImageDraw.Draw(img)
    _draw_ground(draw, size)
    w, h = size
    # crude person silhouette: head + body, standing on the landing pad
    cx, cy = w * 0.5, h * 0.5
    draw.ellipse([cx - 10, cy - 45, cx + 10, cy - 25], fill=(40, 30, 30))       # head
    draw.rectangle([cx - 14, cy - 25, cx + 14, cy + 35], fill=(40, 30, 30))     # torso
    img.save(path)
    return path


if __name__ == "__main__":
    out_dir = os.path.dirname(os.path.abspath(__file__))
    clear_path = generate_clear_landing_zone(os.path.join(out_dir, "default_clear_landing.jpg"))
    blocked_path = generate_blocked_landing_zone(os.path.join(out_dir, "sanganer_clinic_landing.jpg"))
    print(f"Wrote {clear_path}")
    print(f"Wrote {blocked_path}")
