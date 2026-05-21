import sys
import os
from data_logger import DataLogger


if __name__ == '__main__':
    files = sys.argv[1:]
    if not files:
        results_dir = os.path.join(os.path.dirname(__file__), 'res')
        files = sorted(
            os.path.join(root, f)
            for root, _, fs in os.walk(results_dir)
            for f in fs if f.endswith('.csv')
        )
    if not files:
        print("No CSV files found. Usage: python3 plot_results.py res/gazebo/run_*/session_*.csv")
        sys.exit(1)

    for path in files:
        logger = DataLogger.from_csv(path)
        png_path = os.path.splitext(path)[0] + '.png'
        logger.save_plot(png_path, show=True)
        print(f"Loaded {path} — {len(logger._rows)} iterations → saved {png_path}")
