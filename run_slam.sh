#!/bin/bash

echo "[*] Running SLAM simulation..."
python3 src/video_loader.py
python3 src/run_orb_slam.py
python3 src/visualize_map.py
