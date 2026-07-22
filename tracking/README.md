# Tracking — NanoTrack + LightTrack (NCNN)

| Binary | Models | Cmd | MJPEG |
|--------|--------|-----|-------|
| `nanotrack_fc` | `models/nanotrack_*.{param,bin}` | `:12347` | `:5003` |
| `lighttrack_fc` | `models/lighttrack/*` | `:12349` | `:5005` |

## Build

```bash
./build.sh
# ./build.sh --clean
```

Requires OpenCV + ncnn (`NCNN_DIR` in `CMakeLists.txt`, default `/root/ncnn-install`).

## Run

```bash
./build/lighttrack_fc --camera /dev/cam_usb2 --models ./models/lighttrack
./build/nanotrack_fc --camera /dev/cam_usb2 --models ./models
```

Camera frames come from `orch_daemon` SHM. Init bbox manually over UDP — no detector process.

## Benchmark (compare all trackers)

`benchmark_fc` runs all available tracker backends on a dataset and outputs metrics
(IoU, AUC, FPS, latency, robustness, precision@20px).

### Build

```bash
./build.sh   # builds benchmark_fc alongside the other binaries
```

### Dataset format (OTB / UAV123 / LaSOT compatible)

```
dataset_root/
  seq_name1/
    img/
      frame_000001.jpg
      frame_000002.jpg
      ...
    groundtruth.txt     # one bbox per line: cx,cy,w,h (normalized) OR x,y,w,h (absolute pixels)
  seq_name2/
    ...
```

### Usage

```bash
# Run all backends on a dataset
./build/benchmark_fc --dataset /path/to/OTB2015 --backends all --verbose

# Run specific backends
./build/benchmark_fc --backends "nanov3 csrt kcf tctrack" --models ./models

# Save report and visualizations
./build/benchmark_fc --backends all --output report.csv --viz ./viz_out/
```

### Metrics

| Metric | Meaning |
|--------|---------|
| IoU | Intersection over Union (mean) |
| AUC | Area Under Success curve (IoU thresholds 0.0–1.0) |
| Prec@20 | % frames with center error < 20px |
| FPS | Frames per second |
| Lat(avg/med) | Average/median inference latency |
| Robustness | 1 − (lost frames / total frames) |
| LostEv | Number of LOST events (track loss) |

### Visualize results

```bash
# Requires matplotlib + numpy
python3 ../deploy/benchmark_report.py report.csv --output-dir ./benchmark_viz

# Opens benchmark_report.html for interactive charts
```