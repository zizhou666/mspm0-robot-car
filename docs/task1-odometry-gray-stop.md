# TASK1 Odometry and Finish-Line Stop

- Entering TEST1 opens the TASK1 page in idle state.
- Before starting, left/right changes the stop-distance gate by 1 cm.
- Holding left/right uses the existing 500 ms delay and 100 ms repeat.
- The default gate is 610 cm and the adjustable range is 550 to 700 cm.
- Center starts the timer and a 1-second linear ramp from 0 to 40 cm/s.
- The car follows the 12-channel infrared line at a maximum of 40 cm/s.
- The gyro is not required and does not participate in TASK1 completion.
- The car stops immediately only when traveled distance reaches the gate and
  at least five infrared channels are active in the same control update.
- Pausing freezes the timer; continuing performs a new 1-second speed ramp
  while preserving the original distance origin and elapsed time.
