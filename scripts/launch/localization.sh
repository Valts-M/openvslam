#!/bin/bash

cd ~/openvslam
./build/run_slambox -c example/realsense/realsense-T265.yaml -v vocab/orb_vocab.fbow -s /home/xavier/Desktop/reports/ -l
python3 report_generator/report_generator.py --config /home/xavier/Desktop/reports/latest/latest/config.json
