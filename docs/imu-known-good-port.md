# ICM42688 Known-Good Port

## Goal

Make the current project's ICM42688 yaw angle behavior match the archive project
that has been verified on the same replacement car board and the same horizontal
sensor installation.

## Scope

- Replace the current ICM42688 driver behavior with the verified reference
  behavior.
- Restore the verified `500 dps` gyro range and matching `65.5 LSB/(deg/s)`
  scale.
- Restore the verified `1.543` yaw-rate multiplier in the data path.
- Restore the reference yaw output filter time constant.
- Calculate integration time from the last successful IMU sample so a transient
  failed read does not silently discard elapsed rotation time.
- Preserve the current mount mode because the reference project was verified
  with the same physical installation.

## Compatibility

The following current-project behavior must remain unchanged:

- Task pages and Task 1 behavior
- OLED startup animation and UI
- NRF24L01 communication and remote-control pages
- Motor, encoder, line-following, battery, button, buzzer, and RGB behavior
- Existing IMU initialization status and elapsed-time display

The reference project's separate `imu_task.c/.h` files will not be imported.
Only their successful-sample timing behavior will be adapted to the current
main-loop structure.

## Verification

1. Compare the resulting ICM42688 driver and relevant constants with the
   verified reference project.
2. Review the diff to ensure unrelated current features are untouched.
3. Rebuild the complete Keil target.
4. On hardware, test slow and fast left/right 90-degree turns several times.

Hardware angle testing remains the final acceptance check because it cannot be
reproduced by the host-side build.
