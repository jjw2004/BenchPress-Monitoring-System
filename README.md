# Bench Press Monitoring System 4th Year Final project
## BarTone: A Clip-On IoT Bench Press Monitor with Audio Feedback

#### This project will build a camera-free, 3D-printed clip-on IMU device that monitors the bench press, streaming data over Bluetooth to a phone app and cloud dashboard. Features include bar path and tilt tracking, sonified headphone feedback mapping tilt and speed to sound so lifters learn form by ear, paused-bench competition calls, spotter alerts on velocity loss, a pinned-bar alarm, uneven-plate checks, bar-drop logging, speed-loss set termination, daily readiness scores, sticking-point analysis, and load-velocity 1RM estimation.

## Features

### Form
- **Bar Path & Tilt Tracking** – Tracks the bar's movement through each rep and detects if one side of the bar dips lower than the other.
- **Audio Form Feedback** – Turns bar tilt and speed into sound through headphones, so lifters can hear when their form drifts and learn to correct it by ear.
- **Paused Bench Competition Calls** – Detects when the bar settles on the chest and plays "Press!" and "Rack!" commands, just like a powerlifting judge.
- **Sticking-Point Analysis** – Shows where in the rep the bar slows down or stalls and suggests accessory exercises to strengthen that range.

### Safety
- **Spotter Alerts** – Warns the spotter when bar speed drops towards the lifter's failure point, so they're ready to help before the rep fails.
- **Pinned-Bar Alarm** – Detects when the bar is stuck on the lifter's chest and sounds an alarm, escalating to an emergency contact if no one responds.
- **Uneven-Plate Check** – Reads the bar's tilt while it's racked to flag plates that may be loaded unevenly before the lift begins.
- **Bar-Drop Logging** – Detects sudden impacts, such as the bar being dropped or hitting the safeties, and logs them in the session history.

### Training
- **1RM Estimation** – Estimates the lifter's one-rep max from bar speed at lighter weights, with no need to test a true max.
- **Speed-Loss Set Termination** – Tells the lifter to end the set once bar speed drops by a target amount, to manage fatigue.
- **Daily Readiness Score** – Compares today's warm-up speed with the lifter's usual speed to show whether they're fresher or more fatigued than normal.
