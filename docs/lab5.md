---
layout: page
title: Lab 5 - Linear PID and Linear Interpolation
date: 2026-03-14
---

## 1. Prelab

For PID tuning I set up a system to adjust gains and collect data without re-uploading firmware. I added these BLE commands to the Artemis:

- `START_PID` starts the PID controller with a hard stop timer as a safety net
- `STOP_PID` kills motors immediately
- `SEND_PID_DATA` sends back logged arrays (timestamp, raw ToF, extrapolated distance, motor command) to Jupyter
- `SET_PID_GAINS` updates Kp/Kd/Ki on the fly without re-flashing
- `SET_SETPOINT`, `SET_PID_DURATION`, `SET_MAX_PWM`, `SET_CALIBRATION` for adjusting everything else from Python

The controller logs up to 1500 samples per run. After it finishes I pull the data back and plot it in Jupyter, which made it pretty quick to try different gains.

```python
# Set gains and run a trial from Jupyter
ble.send_command(CMD.SET_PID_GAINS, f"{KP}|{KD}|{KI}")
ble.send_command(CMD.SET_SETPOINT, "304")
ble.send_command(CMD.SET_MAX_PWM, "120")
ble.send_command(CMD.START_PID, "")
time.sleep(6)
ble.send_command(CMD.SEND_PID_DATA, "")
```

I also wrote a sensor diagnostic that collects both ToF sensors for 5 seconds. This ended up being really useful because I found out my front and side sensors were actually swapped. Sensor 1 was reading the side and sensor 2 was the one facing forward. Once I fixed the PID loop to read from the right sensor things started working.

---

## 2. Lab Tasks

I went with a **PD controller** (no integral term). The task is just position control with a fixed target so proportional handles steady-state fine, and derivative damps overshoot. There's nothing persistently pushing the car off the setpoint so I didn't need an integrator.

I started with P-only at Kp=0.05 and worked up:

| Config | Kp | Kd | Overshoot | Settling (<30mm) | Notes |
|--------|----|----|-----------|-------------------|-------|
| P-only (low) | 0.05 | 0 | 83mm | didn't settle | Too sluggish, car at constant deadband PWM, couldn't brake in time |
| P-only (higher) | 0.15 | 0 | 31mm | 0.84s | Better approach but still overshoots |
| PD (final) | 0.10 | 0.05 | **5mm** | **0.50s** | Smooth approach, minimal overshoot |

The ToF gives distances in the range 0-4000mm. With a setpoint of 304mm the max error is about 1700mm when starting 2m away. At Kp=0.10 that gives a PID output of 170 which gets clamped to MAX_PWM=120. As the car gets closer the output drops proportionally.

Kd=0.05 was enough to cut overshoot from 31mm down to 5mm. I tried Kd=0.5 first but that was way too aggressive and the motor command just flipped back and forth between forward and reverse.

For the deadband I set a floor of 60 PWM, so anything the PID outputs between 0 and 60 gets bumped up to 60 (the minimum that actually moves the car from Lab 4). Within ±15mm of the setpoint I just set motors to 0 so it doesn't jitter.

I used Short distance mode on the VL53L1X for faster updates. It caps the range at ~1.3m but that's fine since we're always within 2m of the wall. Sensor gives about 11 Hz of new readings.

The PID loop runs at about 388 Hz, roughly 36x faster than the sensor. Between new ToF readings the controller uses linear extrapolation to estimate distance (more on that below).

**Final controller: Kp=0.10, Kd=0.05, MAX_PWM=120, Setpoint=304mm**

<div class="lab-media">
  <p><a href="{{ 'images/lab5_tof_vs_time.png' | relative_url }}"><img src="{{ 'images/lab5_tof_vs_time.png' | relative_url }}" alt="ToF distance vs time" style="max-width:100%;" /></a></p>
</div>

<div class="lab-media">
  <p><a href="{{ 'images/lab5_motor_vs_time.png' | relative_url }}"><img src="{{ 'images/lab5_motor_vs_time.png' | relative_url }}" alt="Motor command vs time" style="max-width:100%;" /></a></p>
</div>

<div class="lab-media">
  <p><a href="{{ 'images/lab5_error_vs_time.png' | relative_url }}"><img src="{{ 'images/lab5_error_vs_time.png' | relative_url }}" alt="Error vs time" style="max-width:100%;" /></a></p>
</div>

Max speed from the ToF data was about 110 mm/s (0.11 m/s), with an average approach speed around 70 mm/s.

<video width="100%" controls>
  <source src="{{ 'images/lab5_starting_distance_2000mm_video.mp4' | relative_url }}" type="video/mp4">
</video>

<video width="100%" controls>
  <source src="{{ 'images/lab5_starting_distance_30mm_with_perturbation_test_video.mp4' | relative_url }}" type="video/mp4">
</video>

The controller recovers from perturbations. If I push the car toward the wall it backs up, if I pull it away it drives forward. The error just flips sign and the PD handles it.

<video width="100%" controls>
  <source src="{{ 'images/lab5_perturbation_test_video.mp4' | relative_url }}" type="video/mp4">
</video>

<video width="100%" controls>
  <source src="{{ 'images/lab5_diagonal_line_with_different_wall_surface_video.mp4' | relative_url }}" type="video/mp4">
</video>

---

## 3. Extrapolation

The ToF only gives new data at ~11 Hz but the PID loop runs at ~388 Hz, so there's about 36 loops between each sensor update.

I use linear extrapolation from the last two ToF readings. When a new reading comes in I compute the slope:

```cpp
tof_slope = (tof_current - tof_prev) / (tof_current_time - tof_prev_time);
```

Then between readings I estimate distance:

```cpp
distance_est = tof_current + tof_slope * (millis() - tof_current_time);
```

You can see this in the ToF plot where the blue line smoothly fills in between the red dots instead of just being a staircase.

| Component | Rate |
|-----------|------|
| ToF sensor | ~11 Hz |
| PID control loop | ~388 Hz |
| Ratio | ~36x faster |

I also put a low-pass filter on the derivative term:

```cpp
pid_filtered_derivative = alpha * raw_derivative + (1 - alpha) * pid_filtered_derivative;
```

with alpha=0.05. This keeps the derivative from spiking when the ToF reading jumps around.

---
