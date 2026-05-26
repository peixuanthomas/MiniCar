# Line Error Jump Filter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ignore isolated discontinuous line sensor error samples before they enter the PID correction loop.

**Architecture:** Put the jump detection in a small pure C module so it can be host-tested without STM32 HAL dependencies. `UsrTimer.c` owns the PID loop and calls the filter once after line sensor error calculation; ignored samples reuse the last accepted error and skip PID memory updates for that tick.

**Tech Stack:** STM32F103 C firmware, existing Keil MDK project, host `gcc` unit tests.

---

### File Structure

- Create: `App/inc/LineErrorFilter.h` - public filter state, result type, reset/update APIs, tuning constants.
- Create: `App/src/LineErrorFilter.c` - jump threshold and consecutive-confirmation logic.
- Create: `tests/line_error_filter_test.c` - host unit tests for the pure filter.
- Modify: `App/src/UsrTimer.c` - initialize/reset the filter and gate PID updates.
- Modify: `MDK-ARM/MiniCar103.uvprojx` - add `LineErrorFilter.c` to the Application/Tasks group.

### Task 1: Filter Unit

**Files:**
- Create: `tests/line_error_filter_test.c`
- Create: `App/inc/LineErrorFilter.h`
- Create: `App/src/LineErrorFilter.c`

- [ ] **Step 1: Write the failing test**

Create `tests/line_error_filter_test.c` with checks for initialization, single jump ignore, repeated jump acceptance, normal-change acceptance, and reset.

- [ ] **Step 2: Run test to verify it fails**

Run: `gcc -std=c99 -Wall -Wextra -I App/inc tests/line_error_filter_test.c App/src/LineErrorFilter.c -o tests/line_error_filter_test.exe`

Expected before implementation: compile failure because `LineErrorFilter.h` and functions do not exist.

- [ ] **Step 3: Write minimal implementation**

Implement `LineErrorFilter_Reset()` and `LineErrorFilter_Update()` with threshold `4.0f` and confirmation count `2`.

- [ ] **Step 4: Run test to verify it passes**

Run: `gcc -std=c99 -Wall -Wextra -I App/inc tests/line_error_filter_test.c App/src/LineErrorFilter.c -o tests/line_error_filter_test.exe; ./tests/line_error_filter_test.exe`

Expected: `line error filter tests passed`.

### Task 2: PID Integration

**Files:**
- Modify: `App/src/UsrTimer.c`
- Modify: `MDK-ARM/MiniCar103.uvprojx`

- [ ] **Step 1: Include and reset filter state**

Add `#include "LineErrorFilter.h"`, a static `LineErrorFilter_t line_error_filter`, and reset it in `reset_line_control_state()`.

- [ ] **Step 2: Gate PID state updates**

After `black_count == 8` handling, call `LineErrorFilter_Update()`. If `accepted` is false, publish the reused error and previous motor speeds, then return before integral, derivative, correction, and `saved_error` updates.

- [ ] **Step 3: Add source to Keil project**

Add `LineErrorFilter.c` under the `Application/Tasks` group in `MDK-ARM/MiniCar103.uvprojx`.

- [ ] **Step 4: Run host regression tests**

Run the new filter test and existing host tests that do not require Keil.
