# Custom Perception Training & Dataset Specification

## 1. Overview
Med-Drone Mission Control uses **Ultralytics YOLOv8** for real-time aerial obstacle detection and landing zone screening. This document specifies the dataset schema, annotation guidelines, and training workflow for fine-tuning custom models on aerial drone imagery.

---

## 2. Dataset Schema & Structure
The dataset follows standard YOLO Darknet format:

```text
datasets/med_drone_aerial/
├── data.yaml
├── images/
│   ├── train/       # Training images (.jpg, .png)
│   ├── val/         # Validation images
│   └── test/        # Test images (optional)
└── labels/
    ├── train/       # Normalized bounding box text files (.txt)
    ├── val/
    └── test/
```

### Label File Format
For each image `image_0001.jpg`, a matching `image_0001.txt` contains one row per object:
```text
<class_id> <x_center> <y_center> <width> <height>
```
*Coordinates are normalized between `0.0` and `1.0` relative to image dimensions.*

### Class Dictionary
| Class ID | Class Name | Description | Flight Controller Response |
|---|---|---|---|
| `0` | `person` | Ground pedestrian, flood victim, responder | Inhibit landing, hover dwell |
| `1` | `vehicle` | Car, ambulance, boat, tractor | Inhibit landing if within 10m pad |
| `2` | `construction_crane` | Tower crane, boom crane, tall rig | Tactical LOITER, 50m A* reroute |
| `3` | `communication_tower` | Cell tower, radio mast, guy wires | Tactical LOITER, 50m A* reroute |
| `4` | `power_line_pylon` | High-voltage transmission pylon | Climb +20m or A* reroute |
| `5` | `uncooperative_uav` | Rogue drone, unauthorized quadcopter | Immediate LOITER / emergency RTL |
| `6` | `emergency_landing_pad` | Prepared tarp, painted H, rooftop pad | Engage ArUco / visual precision land |
| `7` | `flood_debris` | Floating debris, submerged obstacle | Flag unsuitable landing zone |

---

## 3. Recommended Open-Source Datasets
To train custom weights without flying a physical drone:
1. **VisDrone-DET**: 10,209 aerial images captured by drones across 14 cities in China, covering pedestrians, cars, vans, trucks, and tricycles from top-down angles.
   - Download: `https://github.com/VisDrone/VisDrone-Dataset`
2. **UAVVaste**: Waste and flood debris from low-altitude drone flights.
3. **SeaDronesSee**: Search and rescue dataset for maritime and flood response.

---

## 4. Annotation Instructions
1. **Tool**: Use [CVAT (Computer Vision Annotation Tool)](https://www.cvat.ai/) or [Roboflow](https://roboflow.com/).
2. **Bounding Box Bounds**: Box must tightly bound the physical extents of the obstacle, including crane booms and masts.
3. **Occlusions**: Label partially occluded objects if $\ge 30\%$ of the structure is visible.
4. **Resolution**: Minimum recommended input image size: $1280 \times 720$.

---

## 5. Training Commands
To start training on Apple Silicon GPU (MPS) or NVIDIA CUDA:

```bash
# Train for 50 epochs with batch size 16 at 640x640 resolution
python3 perception/train_yolo.py --data perception/data_sample.yaml --epochs 50 --batch 16 --imgsz 640
```

### Hardware & Resource Requirements
- **Apple Silicon (M1/M2/M3 with 8 GB RAM)**: Supported using `device="mps"`. Recommended batch size $\le 16$ to avoid Out-Of-Memory thrashing. Expected training speed: ~15-20 minutes for 50 epochs on 1,000 images.
- **NVIDIA GPU (CUDA, 8 GB+ VRAM)**: Supported on Linux / Cloud (Colab, RunPod). Recommended batch size 32 or 64.

---

## 6. Model Evaluation
To measure mAP, precision, recall, and streaming latency:

```bash
python3 perception/evaluate_model.py --weights yolov8n.pt --data perception/data_sample.yaml
```

To deploy newly trained weights into Med-Drone Mission Control:
1. Copy `runs/train_aerial/med_drone_yolov8/weights/best.pt` to `flightstack/models/aerial_yolov8.pt`.
2. Update `perception_pipeline.py` or pass `--weights flightstack/models/aerial_yolov8.pt`.
