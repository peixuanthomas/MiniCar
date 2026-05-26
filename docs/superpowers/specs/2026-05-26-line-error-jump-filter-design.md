# Line Error Jump Filter Design

## Goal

Prevent a single discontinuous line sensor sample from causing an oversized PID correction.

## Design

Add a small line error filter between `calc_sensor_error()` and the PID state update in `App/src/UsrTimer.c`. The filter tracks the last accepted non-lost-line error. If the current raw error differs from the last accepted error by more than `4.0`, the first frame is ignored and the previous accepted error is reused. If the jump repeats for `2` consecutive control ticks, the new error is accepted so real line movement, turns, or sustained offset still take control.

Ignored samples do not update `saved_error`, `integral`, `last_error`, or the derivative filter. This keeps a one-frame sensor twitch from polluting PID memory. Lost-line handling remains unchanged and continues to use the existing `black_count == 0` branch.

## Files

- `App/inc/LineErrorFilter.h`: filter state and API.
- `App/src/LineErrorFilter.c`: pure filter implementation.
- `App/src/UsrTimer.c`: call the filter before PID state updates.
- `tests/line_error_filter_test.c`: host unit test for one-frame ignore and sustained-jump acceptance.
- `MDK-ARM/MiniCar103.uvprojx`: include the new source in the Keil project.

## Testing

Use a host C test for the pure filter logic. Verify that:

- the first sample initializes the accepted error;
- a single large jump is ignored and reuses the previous accepted error;
- a repeated large jump is accepted on the second consecutive tick;
- a normal small change is accepted and clears pending jump state;
- reset clears all filter memory.
