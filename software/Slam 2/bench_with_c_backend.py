import sys
import correlative_match_ctypes
sys.modules['correlative_match'] = correlative_match_ctypes
import bench_full_pipeline  # noqa
