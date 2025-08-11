from pynput import mouse
import csv
import time
import sys


def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <output_filename.csv>")
        sys.exit(1)

    filename = sys.argv[1]

    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["timestamp", "x", "y"])
        start_time = time.time()

        def on_move(x, y):
            timestamp = time.time() - start_time
            writer.writerow([timestamp, x, y])
            file.flush()

        with mouse.Listener(on_move=on_move) as listener:
            print("Tracking mouse movement... Press CTRL+C to stop.")
            try:
                listener.join()
            except KeyboardInterrupt:
                print("\nStopped tracking.")


if __name__ == "__main__":
    main()
