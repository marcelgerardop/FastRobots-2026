---
layout: page
title: Lab 10 - Localization (sim)
date: 2026-04-22
---

## Setup

I cloned `FastRobots-sim-release` into my ECE4160 folder and installed `pygame`, `PyQt6`, `pyqtgraph`, `Box2D`, `pyyaml`, `ipywidgets`, and `colorama` into my `FastRobots_ble` venv. I had to pin PyQt6 to 6.6.1 (6.11 hit a Windows DLL load error) and downgrade IPython to 8.30 (9.10 spammed UnicodeDecodeErrors from its new source scanner). I ran `inClassDemo.ipynb` first as a smoke test.

<div class="lab-media">
  <p><a href="{{ 'images/lab10_setup.png' | relative_url }}"><img src="{{ 'images/lab10_setup.png' | relative_url }}" alt="Simulator and plotter running" style="max-width:100%;" /></a></p>
</div>

##### Simulator and plotter running next to the notebook.

---

## 1. compute_control and odom_motion_model

`compute_control` decomposes two poses into the odometry format: turn to face the destination, drive to it, turn to the final heading.

```python
def compute_control(cur_pose, prev_pose):
    dx = cur_pose[0] - prev_pose[0]
    dy = cur_pose[1] - prev_pose[1]
    delta_rot_1 = mapper.normalize_angle(math.degrees(math.atan2(dy, dx)) - prev_pose[2])
    delta_trans = math.hypot(dx, dy)
    delta_rot_2 = mapper.normalize_angle(cur_pose[2] - prev_pose[2] - delta_rot_1)
    return delta_rot_1, delta_trans, delta_rot_2
```

`odom_motion_model` treats rot1, trans, rot2 as three independent Gaussians and multiplies them. The big trap is angle normalization: a raw residual of 340 is really 20 degrees, and without wrapping the Gaussian returns nearly zero for two angles that are basically the same.

```python
def odom_motion_model(cur_pose, prev_pose, u):
    u_hat_r1, u_hat_t, u_hat_r2 = compute_control(cur_pose, prev_pose)
    p1 = loc.gaussian(mapper.normalize_angle(u_hat_r1 - u[0]), 0, loc.odom_rot_sigma)
    p2 = loc.gaussian(u_hat_t - u[1],                          0, loc.odom_trans_sigma)
    p3 = loc.gaussian(mapper.normalize_angle(u_hat_r2 - u[2]), 0, loc.odom_rot_sigma)
    return p1 * p2 * p3
```

A matching test pair gave 1.0 and a way-off pair gave 2.3e-16, which is the separation I wanted.

---

## 2. Prediction step

A naive prediction loops 1944 prior states against 1944 current states, so 3.7 million `odom_motion_model` calls per step. Too slow. I skipped any prior cell with belief below 0.0001, so once the belief concentrates the outer loop only touches a few cells.

```python
def prediction_step(cur_odom, prev_odom):
    actual_u = compute_control(cur_odom, prev_odom)
    loc.bel_bar = np.zeros_like(loc.bel)
    NX, NY, NA = mapper.MAX_CELLS_X, mapper.MAX_CELLS_Y, mapper.MAX_CELLS_A
    for pcx in range(NX):
      for pcy in range(NY):
        for pca in range(NA):
            pb = loc.bel[pcx, pcy, pca]
            if pb < 0.0001: continue
            pp = mapper.from_map(pcx, pcy, pca)
            for cx in range(NX):
              for cy in range(NY):
                for ca in range(NA):
                    cp = mapper.from_map(cx, cy, ca)
                    loc.bel_bar[cx, cy, ca] += odom_motion_model(cp, pp, actual_u) * pb
    loc.bel_bar /= np.sum(loc.bel_bar)
```

After the first update, each prediction runs in 2-3 seconds.

---

## 3. Update step

I skipped the `sensor_model` helper and vectorized against `mapper.obs_views` (shape (12, 9, 18, 18), one precached ray set per cell). Broadcasting turns the whole step into one numpy call.

```python
def update_step():
    z = loc.obs_range_data.flatten()                     # (18,)
    diffs = mapper.obs_views - z                         # (12, 9, 18, 18)
    probs = loc.gaussian(diffs, 0, loc.sensor_sigma)     # same shape
    likelihood = np.prod(probs, axis=3)                  # (12, 9, 18)
    loc.bel = likelihood * loc.bel_bar
    loc.bel /= np.sum(loc.bel)
```

Runs in milliseconds.

---

## Results

From a uniform prior at (0, 0, 0), the first update alone collapsed belief to 99.996% on cell (5, 4, 9), one cell off from the true (6, 4, 9). A single 360 scan is enough to localize from scratch.

Over 16 trajectory steps the belief probability was essentially 1.0 on the top cell every time (only step 0 was 0.97). Five steps matched ground truth exactly and the rest were off by exactly one cell in x, y, or yaw. Max position error stayed near 0.3m (one grid cell).

<div class="lab-media">
<video src="{{ 'images/lab10_sim_trajectory.mp4' | relative_url }}" controls style="max-width:100%; border-radius:8px;"></video>
<p><a href="{{ 'images/lab10_sim_trajectory.mp4' | relative_url }}">Open video</a> if it doesn't play above.</p>
</div>

##### Green = GT, red = odometry, blue = belief. Odometry drifts badly, belief stays within one cell of GT.

| Step | GT index | Belief index | Match |
|------|----------|--------------|-------|
| 0 | (6, 3, 7) | (6, 4, 6) | off by 1 y, 1 a |
| 1 | (7, 2, 6) | (7, 2, 6) | exact |
| 3 | (7, 1, 4) | (7, 1, 4) | exact |
| 10 | (10, 7, 16) | (10, 7, 16) | exact |
| 15 | (3, 3, 0) | (3, 3, 0) | exact |

Prior belief (after prediction) was often 10-30% spread across a few cells, and then the update pulled it back to 100% on one. So the sensor model is doing almost all the work here, and prediction mostly keeps belief from going too narrow between updates. By step 5 the odometry had drifted far from GT, but belief never followed it away.

Where it would fail: a symmetric map (like a square room) would have cells with identical ray fingerprints and belief could lock onto the wrong one. This map has enough interior obstacles that each cell is distinct.
