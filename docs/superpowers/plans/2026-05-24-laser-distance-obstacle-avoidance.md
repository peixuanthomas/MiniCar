# Laser Distance Obstacle Avoidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add ATK-MS53L0M UART distance measurement on USART3/PB10/PB11, show `dist: xx mm` on the OLED, and add a separate obstacle-avoidance control layer that bypasses rectangular obstacles under 150 mm without editing the existing line-following algorithm body.

**Architecture:** Keep the laser sensor as a BSP module that owns the USART3 query/parse state and exposes the latest valid distance in millimeters. Keep obstacle avoidance as an application control module called before the existing line-following code in the TIM1 callback; when inactive, the original line-following logic runs unchanged. TaskMain only schedules sensor polling and OLED text updates.

**Tech Stack:** STM32F103C8T6, STM32 HAL UART/TIM/GPIO, existing FIFO UART wrapper, existing OLED API, Keil MDK project file, host C parser tests where possible.

---

### File Structure

- Create: `Bsp/inc/laser_distance.h` - public sensor API, distance validity, query task, frame parser test hook.
- Create: `Bsp/src/laser_distance.c` - ATK-MS53L0M query command, USART3 FIFO byte consumption, frame sync/checksum, stale/error handling.
- Create: `App/inc/ObstacleAvoidance.h` - public obstacle avoidance API called by the timer control loop.
- Create: `App/src/ObstacleAvoidance.c` - small tunable state machine for side bypass maneuver.
- Modify: `App/src/TaskMain.c` - include laser header, replace UART3 echo task with laser task, replace top OLED `FLAG` row with `dist: xx mm`, keep sensor/battery rows.
- Modify: `App/inc/TaskMain.h` - update `TASK_MAX` only if a new task entry is added.
- Modify: `App/src/UsrTimer.c` - include obstacle avoidance and call it before existing line-following code; return early only when avoidance owns motor output.
- Modify: `Bsp/src/Board.c` and `Bsp/inc/Board.h` - include and initialize laser module.
- Modify: `README.md` - change UART3 description from Bluetooth to laser distance module on PB10/PB11.
- Modify: `MDK-ARM/MiniCar103.uvprojx` - add new C source files to the Keil project.

### Task 1: Sensor Parser Unit

**Files:**
- Create: `Bsp/inc/laser_distance.h`
- Create: `Bsp/src/laser_distance.c`

- [ ] **Step 1: Write parser-focused test fixture**

Create a small host test source under a temporary build directory that includes `laser_distance.c` with `LASER_DISTANCE_TEST` enabled and feeds:

```c
static const uint8_t valid_frame[] = {0x55,0x0A,0x00,0x01,0x00,0x00,0x05,0x02,0x01,0x20,0x00,0x88};
static const uint8_t bad_checksum[] = {0x55,0x0A,0x00,0x01,0x00,0x00,0x05,0x02,0x01,0x20,0x00,0x89};
```

Expected behavior: valid frame parses to `288` mm; bad checksum and `State:0 , Range Valid` bytes do not produce a valid distance.

- [ ] **Step 2: Run the test and verify it fails before implementation**

Run: `gcc -DLASER_DISTANCE_TEST -I Bsp/inc -I Core/Inc -I Drivers/STM32F1xx_HAL_Driver/Inc -I Drivers/CMSIS/Device/ST/STM32F1xx/Include -I Drivers/CMSIS/Include <test.c> Bsp/src/FIFO.c -o <test.exe>`

Expected: compile/link failure because `LaserDistance_TestParseByte` and distance status APIs do not exist.

- [ ] **Step 3: Implement minimal laser sensor module**

Implement:

```c
void LaserDistance_Init(void);
void LaserDistance_Task(void);
uint8_t LaserDistance_HasValidDistance(void);
uint16_t LaserDistance_GetDistanceMm(void);
uint8_t LaserDistance_IsStale(void);
uint8_t LaserDistance_TestParseByte(uint8_t byte, uint16_t *distance_mm);
```

Rules: query command is `51 0A 00 01 00 05 02 00 63`; frame header is `0x55`; frame length is `frame[1] + 2`; checksum is 16-bit sum over all bytes before checksum; accept only status `0x00`, function `0x05`, data length `0x02`; reject zero distance and stale data older than 1000 ms.

- [ ] **Step 4: Run parser test and verify pass**

Expected: valid frame returns `288`, invalid checksum returns no parse, ASCII status text returns no parse.

### Task 2: OLED And Task Integration

**Files:**
- Modify: `App/src/TaskMain.c`
- Modify: `Bsp/src/Board.c`
- Modify: `Bsp/inc/Board.h`

- [ ] **Step 1: Replace UART3 echo task with laser polling**

Rename the scheduled task hook from `Uart3Func` to `LaserDistanceFunc`, call `LaserDistance_Task()` every 100 ms, and stop echoing USART3 bytes because USART3 is now owned by the laser module.

- [ ] **Step 2: Replace FLAG row with distance**

Display exactly:

```text
dist: xx mm
```

Use `OLED_Printf(0, 0, OLED_8X16, "dist:%4u mm", distance_mm)` when data is valid. Use `OLED_ShowString(0, 0, "dist: ---- mm", OLED_8X16)` when no valid or stale data exists, clearing the row first with `OLED_ClearArea(0, 0, 128, 16)`.

- [ ] **Step 3: Keep lower OLED rows intact**

Keep `SEN:` at y=16, `BAT:` at y=32, and debug `e=`/`c=` at y=48. Remove only `FLAG:` and its `runFlag` top-row display.

### Task 3: Obstacle Avoidance Layer

**Files:**
- Create: `App/inc/ObstacleAvoidance.h`
- Create: `App/src/ObstacleAvoidance.c`
- Modify: `App/src/UsrTimer.c`

- [ ] **Step 1: Add a state machine that owns motor output only during bypass**

Use default tunables:

```c
#define OBSTACLE_TRIGGER_MM 150U
#define OBSTACLE_CLEAR_MM 220U
#define OBSTACLE_SIDE_SPEED 360
#define OBSTACLE_FORWARD_SPEED 380
#define OBSTACLE_TURN_TICKS 250
#define OBSTACLE_PASS_TICKS 450
#define OBSTACLE_RETURN_TICKS 250
```

At 2 ms per TIM1 tick, this gives about 0.5 s turn, 0.9 s pass, 0.5 s return.

- [ ] **Step 2: Preserve line-following algorithm body**

In `HAL_TIM_PeriodElapsedCallback`, after the `runFlag == 0` stop block and before `use_sensor_state_control`, call:

```c
if (ObstacleAvoidance_Update2ms()) {
    return;
}
```

No PID constants, sensor weighting, lost-line logic, or sensor-state control logic should be modified.

- [ ] **Step 3: Reset avoidance when stopped**

When `runFlag == 0`, call `ObstacleAvoidance_Reset()` before returning so the next run starts cleanly.

### Task 4: Documentation And Project Wiring

**Files:**
- Modify: `README.md`
- Modify: `MDK-ARM/MiniCar103.uvprojx`

- [ ] **Step 1: Update README hardware description**

Change `UART1转USB+UART3转蓝牙调测端口` to `UART1转USB调试口+UART3连接ATK-MS53L0M激光测距模块`.

Change the peripherals table row for UART3 to:

```markdown
| 7  | UART3 | PB10/PB11 | ATK-MS53L0M 激光测距模块 |
```

- [ ] **Step 2: Add sources to Keil project**

Add `..\Bsp\src\laser_distance.c` under the BSP group and `..\App\src\ObstacleAvoidance.c` under the Application/Tasks group.

### Task 5: Build And ST-LINK Validation

**Files:**
- No source edits expected unless verification finds a defect.

- [ ] **Step 1: Build with available local toolchain**

First try the existing Keil/CMSIS build path if available. If not available from shell, at least verify source references and project XML consistency.

- [ ] **Step 2: Flash/debug through ST-LINK if command-line tools are available**

Try OpenOCD/ST-LINK CLI only if installed locally. Validate that the target enumerates, flash succeeds, and OLED shows `dist:` row.

- [ ] **Step 3: Runtime checks**

Expected behavior: distance updates continuously; unplugged or invalid sensor data shows `dist: ---- mm`; when valid distance is below 150 mm while running, avoidance owns motor output for one bypass sequence, then control returns to the existing line follower.

### Self-Review

- Spec coverage: covers USART3/PB10/PB11 laser module, README update, continuous measurement, OLED distance row replacing flag row, error/stale data handling, separate obstacle avoidance layer, original line-following body preservation, and ST-LINK verification attempt.
- Placeholder scan: no TBD/TODO placeholders remain.
- Type consistency: public APIs use `uint8_t` validity flags and `uint16_t` millimeter distances consistently across BSP, OLED, and control modules.
