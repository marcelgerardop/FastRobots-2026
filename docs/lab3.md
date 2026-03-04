---
layout: page
title: Lab 3 - ToF
date: 2026-03-03
---

## 1. Prelab

### I2C Sensor Address

The VL53L1X datasheet lists the default I2C address as 0x52 (8-bit write address). But, the arduino uses 7-bit addressing, so the I2C scan reports 0x29. The IMU (ICM-20948) appears at 0x69.

### Approach to Using 2 ToF Sensors

I chose the **XSHUT pin method**: at boot, I pulled XSHUT LOW on sensor 2 to shut it down, change sensor 1's I2C address to 0x32, then release XSHUT to bring sensor 2 back online at 0x29. This made it so both sensors have unique addresses and can be read independently. I decided to use this method instead of continuously toggling sensors on/off for each read, which wouldve given me a smaller sampling rate and add initialization latency.

```cpp
// ToF dual-sensor init via XSHUT
pinMode(XSHUT_PIN, OUTPUT);
digitalWrite(XSHUT_PIN, LOW);   // shut down sensor 2
delay(10);
distanceSensor1.begin();
distanceSensor1.setI2CAddress(0x32);  // change sensor 1 address
digitalWrite(XSHUT_PIN, HIGH);  // bring sensor 2 back up
delay(10);
distanceSensor2.begin();         // sensor 2 at default 0x29
```

**Lab Soldering Note:** During testing, the second ToF sensor had a bad solder joint on the QWIIC wires, causing an I2C short (all addresses appeared in scans). I The diagnostic sketch confirmed only one sensor was functional, so the lab was completed with a single ToF sensor. The wiring and code for dual sensors is in place for when the second sensor is re-soldered.

### Sensor Placement and Missed Obstacles

I plan to mount sensor 1 facing forward on the front of the chassis to detect obstacles ahead, and sensor 2 on one side for wall-following and detecting obstacles during turns. The robot will miss obstacles approaching from behind, from the uncovered side, objects below the beam height, and thin objects that fall between the sensor's field of view. I used long QWIIC cables for both sensors since they mount on the chassis edges, while the IMU stays near the Artemis with a short cable.

<div class="lab-media">
  <p><a href="{{ 'images/lab3_wire_diagram.jpeg' | relative_url }}"><img src="{{ 'images/lab3_wire_diagram.jpeg' | relative_url }}" alt="Wiring diagram" style="max-width:100%;" /></a></p>
</div>

---

## Lab Tasks

### Battery Power + Soldering

I soldered the JST connector to the 650mAh LiPo battery (cutting wires one at a time to avoid shorting), applied heat shrink, and verified polarity. The Artemis powers on via battery and communicates over BLE untethered I confirmed with a PING/PONG test from Jupyter.

<div class="lab-media">
  <p><a href="{{ 'images/lab3_tof_setup.jpeg' | relative_url }}"><img src="{{ 'images/lab3_tof_setup.jpeg' | relative_url }}" alt="ToF sensor connected to QWIIC breakout board" style="max-width:100%;" /></a></p>
</div>

### I2C Scan

Ran the Apollo3 Example05_Wire_I2C scanner. The scan found two devices:

- **0x29** — VL53L1X ToF sensor (matches datasheet)
- **0x69** — ICM-20948 IMU (with AD0_VAL = 1)

<div class="lab-media">
  <p><a href="{{ 'images/lab3_i2c_scan.png' | relative_url }}"><img src="{{ 'images/lab3_i2c_scan.png' | relative_url }}" alt="I2C scan showing 0x29 and 0x69" style="max-width:100%;" /></a></p>
</div>

### Distance Mode

The VL53L1X has three ranging modes:

1. **Short** (1.3 m max)
2. **Medium** (3 m max)
3. **Long** (4 m, max) - is slower at ~100ms

I chose Long mode because a fast-moving robot needs maximum detection range to have enough stopping distance. The ~100ms measurement time is acceptable since the robot's control loop can still react.

<div class="lab-media">
  <p><a href="{{ 'images/lab3_tof_read_distance.png' | relative_url }}"><img src="{{ 'images/lab3_tof_read_distance.png' | relative_url }}" alt="ToF read distance serial output" style="max-width:100%;" /></a></p>
</div>

### Two ToF Sensors + IMU

When I was testing the dual-sensor, the I2C scanner showed all 127 addresses (0x01-0x7E) when both sensors were connected. After some reaserch I found that I had an I2C short caused by a bad solder joint on the second sensor's QWIIC wires. This caused me a big delay on this lab and a bunch of headaches. I confirmed this by running a diagnostic sketch. The XSHUT wire was on the only working sensor (which I had already soldered). After disconnecting the faulty sensor, the bus returned to normal. The dual-sensor XSHUT code I implemented is ready for when the second sensor is re-soldered.

The working sensor and IMU operate correctly on the shared I2C bus.

### Speed Test Discussion

Looking at the serial output, we see the main loop executes in approximately **1 ms per iteration**. ToF data only appears every ~100 ms when the sensor has new data available (Long mode timing). This is the limiting factor in the ToF sensor's ranging time, not the loop speed. I used `checkForDataReady()` instead of a blocking read which made sure the loop didn't hang.

```cpp
// Non-blocking ToF read in loop
if (distanceSensor.checkForDataReady()) {
    int dist = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
    Serial.print("D:"); Serial.print(dist); Serial.print(" mm");
}
```

<div class="lab-media">
  <p><a href="{{ 'images/lab3_speed_test.png' | relative_url }}"><img src="{{ 'images/lab3_speed_test.png' | relative_url }}" alt="Speed test showing fast loop with periodic ToF data" style="max-width:100%;" /></a></p>
</div>

### ToF Distance vs Time

I collected the ToF data for 10 seconds on the Artimus using a non-blocking flag based approach (which is the same I used in lab 2). Basically I stored using arrays then sent them over BLE to jupyter for plotting. Below is the code:

```cpp
if (tof_collecting && tof_index < TOF_ARRAY_SIZE) {
    if (distanceSensor.checkForDataReady()) {
        tof_time[tof_index] = millis();
        tof_data[tof_index] = distanceSensor.getDistance();
        distanceSensor.clearInterrupt();
        tof_index++;
    }
}
```

<div class="lab-media">
  <p><a href="{{ 'images/lab3_tof_distance_vs_time.png' | relative_url }}"><img src="{{ 'images/lab3_tof_distance_vs_time.png' | relative_url }}" alt="ToF distance vs time plot" style="max-width:100%;" /></a></p>
</div>

### IMU Angle vs Time

I collected the IMU data for 5 seconds while tilting/rotating the board, then sent over BLE. The plot shows accelerometer pitch/roll and gyroscope-integrated pitch/roll/yaw. The gyro roll shows significant drift over time (expected without complementary filter correction), while accelerometer values remain bounded and reflect actual tilt. i temporarly paused the ToF ranging during the IMU collection to prevent I2C bus contention between the two sensors sharing the same bus.

<div class="lab-media">
  <p><a href="{{ 'images/lab3_imu_angle_vs_time.png' | relative_url }}"><img src="{{ 'images/lab3_imu_angle_vs_time.png' | relative_url }}" alt="IMU angle vs time plot" style="max-width:100%;" /></a></p>
</div>
