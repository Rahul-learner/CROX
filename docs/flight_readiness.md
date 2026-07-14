# Flight Readiness & Pre-Flight Checks

Before taking your CROX flight controller to the field for its maiden flight (or after significant hardware changes), you must ensure the physical drone and the software are fully synced. A perfectly tuned EKF and PID loop cannot compensate for badly unbalanced props or uncalibrated ESCs.

Follow this checklist to ensure your drone is flight-ready.

---

## 1. ESC Calibration (Crucial)

Electronic Speed Controllers (ESCs) need to know the exact PWM pulse widths that correspond to "0% throttle" and "100% throttle". If your 4 ESCs interpret the throttle signal differently, the drone will immediately flip on takeoff.

In `config/config.h`, the flight controller uses the following defaults for the ESC protocol (standard PWM):
```c
#define ESC_MIN_PULSE_US    1000
#define ESC_MAX_PULSE_US    2000
```

### Calibration Procedure (Standard PWM ESCs):
> ⚠️ **REMOVE ALL PROPELLERS BEFORE DOING THIS.**

1. Ensure the drone is completely powered down (no USB, no battery).
2. Modify your `main.cpp` temporarily (or use a dedicated calibration script) to send `ESC_MAX_PULSE_US` (2000µs) to all motors immediately upon boot.
3. Power the flight controller and ESCs using the LiPo battery.
4. The ESCs will beep to acknowledge the max throttle signal.
5. Quickly command the flight controller to drop the signal to `ESC_MIN_PULSE_US` (1000µs).
6. The ESCs will beep again to confirm the low throttle signal and then play their initialization tone.
7. Disconnect the battery. Your ESCs are now calibrated to exactly 1000-2000µs.

---

## 2. Motor Direction & Layout

The CROX flight controller assumes a standard **Quadcopter X** configuration. 

```text
    Front
  M1(CW)  M2(CCW)
      \  /
       \/
       /\
      /  \
  M3(CCW) M4(CW)
     Rear
```

### Verification:
> ⚠️ **NO PROPELLERS.**

1. Connect the battery and arm the drone.
2. Give a tiny bit of throttle so the motors spin at idle.
3. Lightly touch the side of each motor bell with your finger to feel the rotation direction.
   - **M1 (Front-Left):** Clockwise (CW)
   - **M2 (Front-Right):** Counter-Clockwise (CCW)
   - **M3 (Rear-Left):** Counter-Clockwise (CCW)
   - **M4 (Rear-Right):** Clockwise (CW)
4. If a motor is spinning the wrong way, swap **any two** of the three wires connecting that specific motor to its ESC.

---

## 3. Propeller Balancing

Vibrations are the enemy of the EKF. If your propellers are unbalanced, the accelerometer will be flooded with high-frequency noise, which will ruin the drone's stability and cause the motors to overheat.

### How to Balance:
1. Buy a magnetic propeller balancer.
2. Mount the propeller on the balancer. The heavy blade will drop.
3. Apply a small piece of clear tape to the **lighter** blade, or lightly sand the trailing edge of the **heavier** blade.
4. Repeat until the propeller sits perfectly horizontal on the balancer and doesn't favor any side.

---

## 4. Center of Gravity (CG)

The drone should be physically balanced around the center where the flight controller is mounted.

1. Hold the drone by the center of the frame (or the center of the flight controller).
2. The drone should balance level. 
3. If it dips forward or backward, move the LiPo battery to compensate until the drone balances perfectly. 
4. A bad CG forces the PID controllers to constantly work harder on one axis, eating up your battery and reducing flight performance.

---

## 5. Software Sanity Checks

Before you attach the props, verify the software state on the bench.

1. **IMU Orientation:** Connect via USB and open the UART telemetry in your ground station (as described in the [EKF Calibration Guide](calibrating_ekf.md)). Tilt the drone forward. The 3D model must pitch forward. Tilt right, model rolls right. Yaw right, model yaws right.
2. **Radio Channels:** Turn on your transmitter. Verify that when you push the stick forward, the `rc_pitch` value increases (or decreases, depending on your mapping), and that `rc_throttle` drops below `1050` when the stick is at the absolute bottom.
3. **Failsafe Testing:** With the drone armed (NO PROPS) and motors spinning at a low idle, turn off your transmitter. The motors **must** immediately stop. Ensure your receiver's failsafe is set to output low throttle (e.g., 900µs-1000µs) when signal is lost.

---

## 6. First Hover

1. Go to an open field with soft grass.
2. Attach the balanced propellers. Double-check that CW props are on CW motors, and CCW props on CCW motors.
3. Step back at least 5 meters.
4. Arm the drone.
5. Smoothly increase throttle. Don't linger on the ground (ground effect causes turbulence). Pop it up to about 1 meter high.
6. If the drone immediately tries to flip, **disarm immediately**. Re-check your motor layout, propeller directions, and IMU orientation.
7. If it hovers, you are ready to begin [PID Calibration](pid_calibration.md) over the NRF24 radio link!
