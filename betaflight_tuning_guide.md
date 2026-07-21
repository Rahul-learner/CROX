# Betaflight PID Tuning Guide

Betaflight is one of the most advanced open-source flight controller firmware for multirotors. Over the years, its PID loop has evolved significantly. Modern versions (Betaflight 4.3 and later) employ a highly sophisticated, slider-based tuning system designed to be accessible for beginners while retaining granular control for experts. 

This document details how Betaflight handles PID tuning, including its core components and recommended tuning workflow.

---

## 1. The Core Components of the Betaflight PID Loop

While Betaflight's PID loop uses the standard Proportional, Integral, and Derivative terms, it adds several powerful features on top, specifically **Feedforward (FF)** and dynamic scaling mechanisms.

### P-Term (Proportional)
The P-term dictates how hard the flight controller works to correct an error between the drone's current angle/rate and the pilot's requested angle/rate. 
- **High P:** The drone feels locked in and responsive. If too high, the drone will oscillate rapidly (high-frequency shakes).
- **Low P:** The drone feels mushy, drifts easily, and struggles to hold its attitude against wind or prop wash.

### I-Term (Integral)
The I-term accumulates error over time and corrects long-term drift. It keeps the drone pointing exactly where you left it.
- **High I:** The quad feels robotic and rigidly holds its angle, even against strong wind. Too high, and the drone can feel stiff or exhibit slow, low-frequency wobbles.
- **Low I:** The drone will drift slowly over time, especially during forward flight or when external forces act on it.

### D-Term (Derivative)
The D-term looks at the rate of change of the error and acts as a dampener to the P-term. It prevents the P-term from overshooting the target.
- **High D:** Smooths out the flight and eliminates bounce-back after quick flips or rolls. Too high, and the motors will overheat rapidly due to amplifying high-frequency noise from the gyro.
- **Low D:** Causes bounce-back (overshoot) at the end of sharp movements and increases susceptibility to prop wash oscillation.

### Feedforward (FF)
Feedforward is the most significant modern addition to the Betaflight PID loop. Unlike the P-term (which reacts to *error*), **Feedforward reacts directly to the *movement of the sticks***.
- **The Role of FF:** When you move the stick, FF gives the motors an immediate "kick" to start the movement *before* an error even occurs. This makes the drone feel incredibly snappy and responsive.
- **Independent from PIDs:** Because FF handles stick responsiveness, you no longer need dangerously high P-gains to get a snappy quad. P and D can be tuned purely for stabilization and prop wash handling, while FF is tuned purely for stick feel.

---

## 2. Advanced Tuning Features

Betaflight implements several dynamic algorithms to manage PID behavior across different flight conditions:

### D-Min
D-term causes motor heat. D-Min is a feature that keeps the D-term low during smooth, straight flight (to keep motors cool) but dynamically boosts the D-term during sharp maneuvers or prop wash events (to prevent bounce-back).

### TPA (Throttle PID Attenuation)
As throttle increases, mechanical vibrations and aerodynamic forces increase, often causing oscillations if PIDs are too high. TPA automatically scales down (attenuates) your P and D gains as the throttle goes past a certain breakpoint, preventing high-throttle oscillations without ruining your low-throttle tune.

### Anti-Gravity
When rapidly punching the throttle, the I-term can struggle to keep up, causing the drone's nose to dip (pitch up or down). Anti-Gravity detects rapid throttle changes and temporarily boosts the I-term to keep the drone perfectly level during aggressive throttle pumps.

---

## 3. The Modern Tuning Workflow (Sliders)

Starting with Betaflight 4.3, the developers introduced a **Slider-based tuning system**. Instead of typing in arbitrary numbers for P, I, D, and FF, the sliders mathematically adjust the values in relation to one another based on community-tested presets.

> [!IMPORTANT]  
> Always ensure your drone is mechanically sound (tight bolts, undamaged props, secure motors) before tuning. A bad tune is often a hardware issue in disguise.

### Step 1: Filter Tuning First
Before tuning PIDs, you must tune your filters. Less filtering reduces latency (delay) in the PID loop, which massively improves prop wash handling.
1. Move the Gyro and D-term filter sliders to the right (less filtering) in small increments.
2. Do short test flights and check motor temperatures.
3. Stop moving the sliders right when motors get warm. If they get hot to the touch, back the sliders off immediately.

### Step 2: Master Multiplier & PI (Tracking)
1. **Master Multiplier:** This slider moves P, I, and D up or down together. Increase it until you hear high-frequency oscillations (P-term flutter) or motors get hot, then back it down by 10-20%.
2. **PI Tracking:** This slider adjusts the ratio of P and I. If the drone struggles to hold its angle against the wind, increase the I-term slider.

### Step 3: D-Term (Damping)
- Do a series of sharp flips and rolls (snap rolls).
- Watch the end of the maneuver. If the drone "bounces" back when you center the sticks, increase the D-term damping slider.
- **Caution:** Always check motor temps after increasing D-term.

### Step 4: Feedforward (Stick Response)
- Once the drone is stable (P, I, and D are tuned), adjust how it feels on the sticks.
- If the drone feels sluggish or lags behind your stick inputs, increase the Feedforward slider.
- If the drone feels jerky, twitchy, or overshoots the target heavily, decrease the Feedforward slider.

---

## 4. Blackbox Logging and PIDToolbox

For professional or perfectionist tuning, Betaflight uses **Blackbox Logging**. The flight controller saves all gyro, PID, and stick data to an onboard flash chip or SD card at a high frequency (e.g., 2kHz).

Pilots use third-party software like **PIDToolbox** or the **Blackbox Explorer** to visualize this data:
- **Gyro vs. Setpoint:** You can overlay the drone's actual movement (Gyro) over the pilot's requested movement (Setpoint). A perfect tune will show the Gyro line perfectly tracking the Setpoint line without delays or overshoots.
- **Step Response:** Tools can mathematically analyze the logs to show exactly how fast the quad reacts to inputs, removing all guesswork from the tuning process.
