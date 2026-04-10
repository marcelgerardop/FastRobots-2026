---
layout: page
title: Lab 8 - Stunts!
date: 2026-04-09
---

## Task B: Drift

I chose the drift stunt. The car drives fast toward a wall, initiates a 180 degree turn when it gets close, then drives back.

---

## Approach

I built a state machine on the Artemis with 5 states: APPROACH, BRAKE, DRIFT, RETURN, DONE. All parameters are sent over BLE so I could tune without re-uploading.

- **APPROACH**: Drive forward at set L/R motor PWM values. The front ToF sensor polls for distance. When a real ToF reading drops below the threshold, transition to BRAKE.
- **BRAKE**: Short reverse burst to kill forward momentum before turning.
- **DRIFT**: Yaw PD controller (reused from Lab 6) turns the car toward the 180 degree target using DMP yaw tracking.
- **RETURN**: Drive forward for a set duration.

```cpp
switch (stunt_state) {
    case 0:  // APPROACH
        analogWrite(LEFT_FWD, stunt_approach_left);
        analogWrite(RIGHT_FWD, stunt_approach_right);
        if (tof_new_data && tof_current > 0 && tof_current < stunt_distance_threshold) {
            stunt_state = 1;  // BRAKE
        }
        break;
    case 1:  // BRAKE
        drive(-stunt_brake_pwm, -stunt_brake_pwm);
        if (millis() - stunt_state_time > stunt_brake_duration) {
            stunt_state = 2;  // DRIFT
            yaw_setpoint = stunt_turn_angle;
            current_yaw = 0.0;
        }
        break;
    case 2:  // DRIFT
        if (dmp_data_ready) {
            stunt_motor = compute_yaw_pid(gyrZ);
            // direct analogWrite for turning
            if (abs(yaw_err) < stunt_turn_margin)
                stunt_state = 3;  // RETURN
        }
        break;
    case 3:  // RETURN
        analogWrite(LEFT_FWD, stunt_return_left);
        analogWrite(RIGHT_FWD, stunt_return_right);
        if (millis() - stunt_state_time > stunt_return_duration)
            stunt_state = 4;  // DONE
        break;
}
```

---

## Tuning and Debugging

The biggest challenge was getting the car to not hit the wall. I ran into a few issues during tuning:

**Motor calibration clipping**: My left motor is much weaker than the right. The `drive()` function multiplies the left PWM by a calibration factor (3.5), but at any approach speed above ~72 PWM both motors clip at 255 and the calibration does nothing. I switched to setting left and right motor PWM independently via BLE, bypassing the calibration entirely.

**ToF extrapolation false triggers**: The distance estimate between sensor readings sometimes spiked wildly, triggering the brake too early. I changed the trigger to only fire on actual ToF readings (`tof_new_data && tof_current < threshold`), not the extrapolated estimate.

**Brake state**: At full approach speed the car had too much momentum to turn in time. I added a short reverse burst (200 PWM for 150ms) between approach and drift to kill the forward momentum before starting the turn.

**Yaw PD tuning**: Reused gains from Lab 6 (Kp=3.0, Kd=1.0) but the turn was too slow at 3.4 seconds. Bumped Kp to 4.0 and yaw_max_pwm from 150 to 255, which brought the 180 turn down to ~0.6 seconds.

Final parameters:

| Parameter | Value |
|-----------|-------|
| Approach L/R | 255 / 70 PWM |
| Distance threshold | 500mm |
| Brake | 200 PWM, 150ms |
| Yaw Kp / Kd | 4.0 / 1.0 |
| Yaw max PWM | 255 |
| Return L/R | 255 / 70 PWM |
| Return duration | 1000ms |

---

## Results

Here are 3 successful drift runs:

### Trial 1

<div class="lab-media">
<video src="{{ 'images/lab8_stunt_video1.mp4' | relative_url }}" controls style="max-width:100%; border-radius:8px;"></video>
<p><a href="{{ 'images/lab8_stunt_video1.mp4' | relative_url }}">Open video</a> if it doesn't play above.</p>
</div>

### Trial 2

<div class="lab-media">
<video src="{{ 'images/lab8_stunt_video2.mp4' | relative_url }}" controls style="max-width:100%; border-radius:8px;"></video>
<p><a href="{{ 'images/lab8_stunt_video2.mp4' | relative_url }}">Open video</a> if it doesn't play above.</p>
</div>

### Trial 3

<div class="lab-media">
<video src="{{ 'images/lab8_stunt_video3.mp4' | relative_url }}" controls style="max-width:100%; border-radius:8px;"></video>
<p><a href="{{ 'images/lab8_stunt_video3.mp4' | relative_url }}">Open video</a> if it doesn't play above.</p>
</div>

---

## Sensor Data

Here's the 4-panel plot from one of the successful runs showing ToF distance, yaw angle, motor commands, and state transitions over time.

<div class="lab-media">
  <p><a href="{{ 'images/lab8_stunt_data.png' | relative_url }}"><img src="{{ 'images/lab8_stunt_data.png' | relative_url }}" alt="Stunt data plot" style="max-width:100%;" /></a></p>
</div>

##### Distance, yaw, motor commands, and state for a successful drift run. Green = approach, red = brake, yellow = drift, blue = return.
