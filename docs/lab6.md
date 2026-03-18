---
layout: page
title: Lab 6 - Orientation Control
date: 2026-03-18
---

## Prelab: BLE Setup

I added BLE commands to tune and run the yaw PID. Like last lab, I have separate gains, setpoint, duration, and calibration for orientation control.

```python
ble.send_command(CMD.SET_YAW_GAINS, f"{KP}|{KD}")
ble.send_command(CMD.SET_YAW_SETPOINT, "90")
ble.send_command(CMD.SET_YAW_DURATION, "5000")
ble.send_command(CMD.START_YAW_PID, "")
time.sleep(6)
collected_data.clear()
ble.send_command(CMD.SEND_YAW_PID_DATA, "")
```

The controller logs timestamp, yaw angle, setpoint, and motor command at the DMP update rate (~56 Hz). I pull it back over BLE and plot in Jupyter.

---

## PID Input Signal

I used the ICM-20948's DMP (Digital Motion Processor) for yaw instead of integrating the raw gyroscope. This avoids the drift problem that comes with manually integrating gyro readings over time.

I had to enable DMP by uncommenting `#define ICM_20948_USE_DMP` in the SparkFun library header. Setup in the Arduino sketch:

```cpp
myICM.initializeDMP();
myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0); // max rate
myICM.enableFIFO();
myICM.enableDMP();
myICM.resetDMP();
myICM.resetFIFO();
```

Here I rotated the robot in a full circle by hand with the PID off. I ran this test twice and the second time it didn't reset, so the yaw starts around 280 degrees. But you can see it tracks the hand rotation smoothly.

<div class="lab-media">
  <p><a href="{{ 'images/lab6_dmp_hand_rotation.png' | relative_url }}"><img src="{{ 'images/lab6_dmp_hand_rotation.png' | relative_url }}" alt="DMP hand rotation test" style="max-width:100%;" /></a></p>
</div>

##### DMP yaw during a hand rotation test. No PID, just verifying the sensor tracks correctly.

One thing I ran into was the DMP FIFO backing up. I had to drain it completely each loop with a while loop, otherwise the data would pile up and the readings got delayed. I also had to make sure `current_yaw` resets to 0 when starting the PID so the setpoint is always relative to where the robot is facing at that moment.

---

## Derivative Term

The lab asks if it makes sense to take the derivative of a signal that's already the integral of another. It does. The raw gyroscope reading (`gyrZ`) literally is angular velocity, which is the derivative of yaw. So instead of differentiating the DMP yaw (noisy, susceptible to derivative kick), I just use the gyro directly for the D-term.

I put a low-pass filter on gyrZ (alpha=0.1) to smooth out noise.

---

## PD Controller

I went with PD. Proportional handles the main rotation and derivative damps oscillation so it doesn't overshoot.

```cpp
int compute_yaw_pid(float gyrZ_raw) {
    float error = yaw_setpoint - current_yaw;
    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;

    yaw_filtered_gyrZ = 0.1 * gyrZ_raw + 0.9 * yaw_filtered_gyrZ;
    float output = yaw_kp * error + yaw_kd * (-yaw_filtered_gyrZ);

    int motor_cmd = constrain((int)output, -150, 150);
    if (motor_cmd > 0 && motor_cmd < 70) motor_cmd = 70;
    if (motor_cmd < 0 && motor_cmd > -70) motor_cmd = -70;
    if (abs(error) < 5.0) motor_cmd = 0;
    return motor_cmd;
}
```

The deadband bump to 70 PWM is the minimum to actually spin the wheels. My battery was dying out so had to amp up the 70PWM :). Within 5 degrees of the setpoint I kill the motors and let friction hold position.

I also bypassed the straight-line `drive()` function and used direct `analogWrite` with a separate yaw calibration factor. Had to do this, because the calibration was a bit funky when rotating left and right.

---

## Tuning

I started P-only and added D:

| Kp | Kd | Result |
|----|-----|--------|
| 3.0 | 0.0 | Reaches 90 degrees, jitters at setpoint |
| 3.0 | 0.1 | Less jitter, slight overshoot |
| 3.0 | 1.0 | Smooth approach, settles within 5 degrees |

---

## Range and Sampling

The DMP outputs at about 56 Hz. I initially logged every loop iteration (~1250 Hz) which filled my 1500-sample array in under a second. Had to find this out using Jupyter because my data collected was not effective for plotting. Turned out 89% of the samples were duplicates. I fixed this by only logging when new DMP data arrives, which gives about 27 seconds of recording at 56 Hz.

The ICM-20948 gyroscope default range is ±250 dps. I thought that was plenty since the robot doesn't spin faster than maybe 100 dps.

---

## Results

Final gains: **Kp=3.0, Kd=1.0**, dead zone=5 degrees, deadband=70 PWM, max PWM=150.

### 90 Degree Step Response

<div class="lab-media">
  <p><a href="{{ 'images/lab6_yaw_vs_setpoint_90_degrees.png' | relative_url }}"><img src="{{ 'images/lab6_yaw_vs_setpoint_90_degrees.png' | relative_url }}" alt="Yaw vs setpoint 90 degrees" style="max-width:100%;" /></a></p>
</div>

##### Yaw angle vs time for 90 degree setpoint. Settles within 5 degrees in about 0.9s.

<div class="lab-media">
  <p><a href="{{ 'images/lab6_motor_cmd_90_degrees.png' | relative_url }}"><img src="{{ 'images/lab6_motor_cmd_90_degrees.png' | relative_url }}" alt="Motor command 90 degrees" style="max-width:100%;" /></a></p>
</div>

##### Motor command for 90 degree trial. Starts at max PWM, drops through deadband, then cuts to 0 inside the dead zone.

<div class="lab-media">
<video src="{{ 'images/lab6_clean_90_degree_spin_video.mp4' | relative_url }}" controls style="max-width:100%; border-radius:8px;"></video>
<p><a href="{{ 'images/lab6_clean_90_degree_spin_video.mp4' | relative_url }}">Open video</a> if it doesn't play above.</p>
</div>

### 180 Degree Step Response

<div class="lab-media">
  <p><a href="{{ 'images/lab6_yaw_vs_setpoint_180_degrees.png' | relative_url }}"><img src="{{ 'images/lab6_yaw_vs_setpoint_180_degrees.png' | relative_url }}" alt="Yaw vs setpoint 180 degrees" style="max-width:100%;" /></a></p>
</div>

##### Yaw angle vs time for 180 degree setpoint. Reaches about 165 degrees with about 15 degree steady-state offset.

<div class="lab-media">
  <p><a href="{{ 'images/lab6_motor_cmd_180_degrees.png' | relative_url }}"><img src="{{ 'images/lab6_motor_cmd_180_degrees.png' | relative_url }}" alt="Motor command 180 degrees" style="max-width:100%;" /></a></p>
</div>

##### Motor command for 180 degree trial. Briefly reverses when it overshoots, then holds at deadband trying to close the remaining gap.

### Kick / Disturbance Test

<div class="lab-media">
<video src="{{ 'images/lab6_kick_test_video.mp4' | relative_url }}" controls style="max-width:100%; border-radius:8px;"></video>
<p><a href="{{ 'images/lab6_kick_test_video.mp4' | relative_url }}">Open video</a> if it doesn't play above.</p>
</div>

##### Robot holding heading at 0 degrees while I push it. It corrects back after each disturbance.
