# Wearable Mouse Project

In this repository, we have implemented a wearable mouse using three different scheduling approaches and sensor fusion algorithms. The project uses an ESP32 with BLE, interfaced with inertial sensors (ADXL345 accelerometer, two MPU6050 IMUs for gyroscope and additional acceleration). It supports movement tracking and click detection (single/double clicks) via BLE. Scripts for scheduling analysis, performance tracking, and histogram plotting are also provided.

Repository available at: [GitHub Repo](https://github.com/Farbod82/WEARABLE_MOUSE_CPS_FINAL_PROJECT)

## Hardware Overview
- **Sensors**: ADXL345 (primary accel), MPU6050 (0x68: gyro), MPU6050 (0x69: secondary accel for clicks).
- **Microcontroller**: ESP32 with BLE.
- **Libraries**: Wire (I2C), BleMouse (BLE HID), mabutrace (tracing in time-triggered version).
- **Features**: Calibration for offsets, threshold-based clicks (debounce 50ms, double-click window 300ms).

## Algorithms
All versions share similar calibration and click detection but differ in sensor fusion for mouse movement.

### Calibration
Averages 500 sensor samples (accel/gyro) while assuming the device is stationary. Offsets accel by gravity (az -1g) and scales gyro ( /131). Time-triggered version adds stability check (accel variance <0.05g).

### Click Detection
Threshold-based on secondary MPU6050 accel magnitude (>40,000 units). Debounces triggers (50ms) and detects single/double clicks within a 300ms window. State machines track timings for clicks.

### Sensor Fusion for Movement
- **Accel-Based Smoothing (used in `intrrupt.cpp`)**: Focuses on acceleration with exponential moving average (EMA) low-pass filter (alpha=0.2) for smoothing ax/ay. Applies deadzones (<0.03g raw, <0.015g smoothed). Checks stationarity (accel ~1g, gyro <5 DPS) to reset smoothing. Gyro used only for stationarity, not fusion. Movement: dx/dy = smoothed accel * 40 (linear scaling).
- **Complementary Filter (used in `interrupt_preemptive.cpp` and `time_triggered.cpp`)**: Fuses accel and gyro for orientation. Computes roll/pitch angles from accel (arctan). Integrates gyro over dt (from millis/micros) and fuses via complementary filter (alpha=0.98 gyro, 0.02 accel). Deadzone (>0.2° ignored). Movement: dx/dy = pitch/roll * 40 (angular scaling).

## Mouse Implementations
### 1. Interrupt-Driven (`intrrupt.cpp`)
Non-preemptive interrupts enqueue events (sensor reads via data-ready pins/timers, movement every 10ms) in FIFO queue, processed in loop. Uses accel-based fusion.

### 2. Preemptive Interrupt-Driven (`interrupt_preemptive.cpp`)
Similar interrupts/queue, but dequeue prioritizes movement events (scans/shifts queue). Adds debug prints. Uses complementary filter with millis dt.

### 3. Time-Triggered (`time_triggered.cpp`)
Cyclic executive: 8 frames (1250μs each, busy-wait), polling sensors in fixed sequence. Includes WiFi/mabutrace for tracing, calibration stability. Uses complementary filter with micros dt.

## Comparison
| Aspect                | Interrupt-Driven          | Preemptive Interrupt      | Time-Triggered            |
|-----------------------|---------------------------|---------------------------|---------------------------|
| **Scheduling**       | Interrupt + FIFO queue   | Interrupt + preemptive queue | Cyclic frames (1250μs, 8-loop) |
| **Prioritization**   | None                     | Movement first            | Fixed sequence            |
| **Fusion Algo**      | Accel smoothing          | Complementary filter      | Complementary filter      |
| **Movement Basis**   | Smoothed accel           | Fused roll/pitch          | Fused roll/pitch          |
| **Timing**           | 10ms timers              | 10ms timers + millis dt   | Micros frames + dt        |
| **Extras**           | Basic                    | Debug prints              | WiFi, tracing, stability  |
| **Pros**             | Simple, low overhead     | Responsive movement       | Deterministic, traceable  |
| **Cons**             | Queue overflow risk      | Complex dequeue           | Polling overhead          |

## Scripts
These Python scripts aid in scheduling, tracking, and analysis. Run with Python 3 (install deps like pandas, matplotlib if needed).

- **find_frame_size.py**: Computes valid frame sizes from tasks CSV (WCET, periods, deadlines), generates static schedule, visualizes Gantt chart, exports to CSV. Usage: `python find_frame_size.py <tasks.csv>`.
- **plot_motion.py**: Plots mouse position over time from 'drift_motion.csv' to visualize drift. Usage: `python plot_motion.py`.
- **interval_calculator.sql**: SQL query to compute time intervals between 'click' events in traces (e.g., for latency analysis). Run in SQLite or similar on trace DB.
- **histogram_plotter.py**: Plots histogram of move intervals (in %) from 'delay_intr2.csv' for timing distribution. Usage: `python histogram_plotter.py`.
- **track_mouse.py**: Tracks real mouse movement via pynput, saves timestamps/x/y to CSV. Usage: `python track_mouse.py <output.csv>` (CTRL+C to stop).
- **smoothness_calculator.py**: Computes/prints RMS jerk and plots jerk over time from multiple CSVs (resamples at 0.01s). Measures smoothness. Usage: `python smoothness_calculator.py` (input files at prompt).
- 
# First Demo of Physical Implementation 
![Board_Demo](https://github.com/user-attachments/assets/e72364a1-a57a-4208-8578-a90a3e8e97b6)
