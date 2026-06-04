import sys
import os
from data_logger import DataLogger


if __name__ == '__main__':
    files = sys.argv[1:]
    if not files:
        print("Usage: python3 plot_results.py res/gazebo/run_*/session_*.csv")
        sys.exit(1)

    for path in files:
        logger = DataLogger.from_csv(path)
        png_path = os.path.splitext(path)[0] + '.png'
        logger.save_plot(png_path, show=True)
        print(f"Loaded {path} — {len(logger._rows)} iterations → saved {png_path}")
