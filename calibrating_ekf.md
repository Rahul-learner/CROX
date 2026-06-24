First, congratulations. Building a 9-DoF Extended Kalman Filter from scratch on a microcontroller, handling the hardware interrupts, and writing a binary telemetry link is a massive engineering achievement. You now have a professional-grade flight controller core.

The final step—calibrating the EKF—is famously referred to in the aerospace industry as a "dark art."

To get your drone locked in, you must realize that "calibrating an EKF" actually requires two completely different processes:

1. **Sensor Calibration (Physics):** Removing the physical hardware errors from the raw chips.
2. **Covariance Tuning (Mathematics):** Adjusting the $Q$ and $R$ matrices to change how the EKF "trusts" the sensors.

Here is your master guide to completing both.

---

### Phase 1: Sensor Calibration (Hardware)

The EKF assumes the data you feed it is physically accurate. If your accelerometer thinks $0^\circ$ is actually $3^\circ$ because the chip was soldered slightly crooked, the EKF will mathematically lock the drone at a $3^\circ$ tilt.

**1. Gyroscope Calibration (Do this on EVERY boot)**

* **The Issue:** Gyros drift due to temperature.
* **The Fix:** Every time you power on the Pico, keep the drone perfectly still. Take 1,000 readings of `gx, gy, gz`. Calculate the average for each axis. Subtract this exact average from every future gyro reading before feeding it to `predict()`.

**2. Accelerometer Calibration (Do this ONCE)**

* **The Issue:** The accelerometer zero-point isn't perfect, and its sensitivity isn't exactly $1G$.
* **The Fix (6-Point Tumble):** Place your drone perfectly flat on a table. Record the Z-axis (it should be exactly $1G$). Flip it upside down (should be $-1G$). Point the nose straight down (X should be $1G$), nose straight up, left side down, right side down.
* Find the min and max values for X, Y, and Z. Calculate the offset `(max + min) / 2` and subtract it. Calculate the scale factor to ensure gravity always perfectly normalizes to `1.0`.

**3. Magnetometer Calibration (Do this ONCE per build)**

* **The Issue:** Hard and soft iron interference from your motors and wires.
* **The Fix:** Use the 3D figure-8 bounding box method we wrote earlier to find the `mag_offset` and `mag_scale`.

---

### Phase 2: Covariance Tuning (Mathematics)

Once the physical data is clean, you must tune the EKF's "brain." This is done by adjusting the $R$ (Measurement Noise) and $Q$ (Process Noise) matrices in your C++ constructor.

To understand exactly what changing these numbers does to your drone, try tuning this live 1D simulation.

#### Tuning the $R$ Matrix (How much do you trust the Accel/Mag?)

The $R$ matrix defines how noisy your absolute sensors are.

* `R[i][i] = 0.1f` (Accelerometer)
* `R_mag = 0.5f` (Magnetometer)
* **How to tune it:** You don't have to guess these numbers! Leave the drone sitting on your desk with the motors running (without propellers!). Record 1,000 raw accelerometer and magnetometer readings. Calculate the **Variance** (standard deviation squared) of those readings. Plug those exact variances directly into your $R$ matrices. High vibration frames need a larger $R$.

#### Tuning the $Q$ Matrix (How much do you trust your Gyro kinematics?)

The $Q$ matrix defines how much confidence you have in your physics prediction (the Gyroscope integration).

* `Q[0 to 3] = 0.001f` (Quaternion state noise)
* `Q[4 to 6] = 0.003f` (Gyro bias drift noise)
* **How to tune it:** This is the subjective part.
* **If the 3D model in your web dashboard looks "twitchy" or shakes:** Your filter is trusting the noisy accelerometer too much. You need to **increase $R$** or **decrease $Q$**.
* **If the 3D model looks "smooth" but lags behind your physical hand movements:** The filter is ignoring the accelerometer and trusting the gyro too much. It takes too long to correct drift. You need to **increase $Q$** or **decrease $R$**.



### The Ultimate Tuning Methodology

If you want to perfectly dial in the filter without going crazy, follow this exact sequence:

1. **Isolate the Gyro:** Set $R$ to a massive number (e.g., `1000.0f`). This effectively turns off the accelerometer and magnetometer. Move the drone around. It should move smoothly but will eventually drift away from level. This confirms your `predict()` math is perfect.
2. **Dial in the Accel:** Bring the accelerometer $R$ down to `0.1f`. Hold the drone at a $45^\circ$ angle. It should quickly snap to $45^\circ$ on your screen. If it vibrates, raise $R$. If it slowly floats to $45^\circ$ over two seconds, lower $R$ (or raise $Q$).
3. **Dial in the Mag:** Finally, set `R_mag` to `0.5f`. Point the drone North. Rotate it $90^\circ$ East. If the yaw overshoots East and slowly crawls back, your $Q$ yaw bias is too low. If it twitches constantly, raise `R_mag`.
