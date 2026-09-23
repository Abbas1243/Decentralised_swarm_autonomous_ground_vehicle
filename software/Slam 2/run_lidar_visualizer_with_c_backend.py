"""
run_lidar_visualizer_with_c_backend.py
=========================================
Runs lidar_visualizer.py UNMODIFIED, with correlative_match replaced by
the ctypes-wrapped C implementation instead of the pure-Python reference
-- same sys.modules swap trick as run_slam_selftest_with_c_backend.py,
applied here so it happens before lidar_visualizer.py's own
`from slam import SlamState` triggers slam.py's `import correlative_match`.

Usage: identical to lidar_visualizer.py itself, e.g.:
    python3 run_lidar_visualizer_with_c_backend.py --port /dev/ttyUSB0
    python3 run_lidar_visualizer_with_c_backend.py --port /dev/ttyUSB0 --matplotlib
"""
import sys
import correlative_match_ctypes

sys.modules['correlative_match'] = correlative_match_ctypes

import lidar_visualizer

if __name__ == "__main__":
    lidar_visualizer.main()
