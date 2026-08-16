# STM32 bring-up guide: Nucleo-F401RE PIL firmware

Turns the code in `stm32/main_pil_loop.c` plus the vendored GNC logic
in `c/` into a flashed, running firmware. Written for STM32CubeIDE on
Windows 10 (CubeIDE is cross-platform; these steps are identical on
Linux/macOS).

## 1. Create the CubeMX project

1. **File -> New -> STM32 Project**
2. Board selector tab -> search **NUCLEO-F401RE** -> select it -> Next
3. Project name: `lifting-body-pil-firmware`
   Targeted language: **C**. Finish.
4. CubeMX will ask to initialize all peripherals to their default
   mode -- **Yes**. This gives you a working clock config and the
   Nucleo's default USART2 routing (to the ST-LINK VCP) for free.

## 2. Verify/configure USART2 (should already be enabled by the default)

In the `.ioc` file's **Connectivity -> USART2** panel:
- Mode: **Asynchronous**
- Baud Rate: **115200**
- Word Length: **8 bits**
- Parity: **None**
- Stop Bits: **1**

This matches `python/scripts/serial_ping_test.py` and `pil_driver.py`'s
defaults (`--baud 115200`). If you change it here, change it on the
PC side too.

## 3. Increase the stack size (do this even though Phase 4a's fix
   dropped peak usage to ~1.3 KB -- cheap insurance, not required)

In `.ioc` -> **Project Manager -> Advanced Settings**, or directly in
the generated linker script (`STM32F401RETX_FLASH.ld`), confirm:

```
_Min_Stack_Size = 0x800;   /* 2 KB is comfortable headroom */
```

The F401RE has 96 KB SRAM total; 2 KB for stack is a small, safe
fraction and leaves ample room for the vendored code's `.bss` (the
`static` matrices from the Phase 4a stack-usage fix) plus HAL/UART
buffers.

## 4. Enable the DWT cycle counter access (usually needs no explicit
   CubeMX setting -- `main_pil_loop.c`'s `dwt_init()` enables it in
   software). Just confirm the project's core is Cortex-M4 with FPU
   (CubeMX sets this automatically for the F401RE) so `CoreDebug` and
   `DWT` register definitions resolve -- they come from CMSIS, already
   included via `main.h`.

## 5. Add the source files to the project

Copy into the CubeIDE project (drag-and-drop into the Project Explorer,
or copy on disk and refresh):

```
Core/Inc/  <-  c/include/quat_math.h
               c/include/matlib.h
               c/include/eskf.h
               c/include/control.h
               c/include/pil_protocol.h
               c/include/pil_core.h

Core/Src/  <-  c/src/quat_math.c
               c/src/matlib.c
               c/src/eskf.c
               c/src/control.c
               c/src/pil_protocol.c
               c/pil/pil_core.c
               stm32/main_pil_loop.c
```

(Flattening into `Core/Inc`/`Core/Src` is the path of least resistance
with CubeIDE's default include paths; if you'd rather preserve the
`c/include`/`c/src` structure, add those as additional include paths
under **Project -> Properties -> C/C++ Build -> Settings -> Include
paths** instead of flattening.)

**Do NOT copy** `c/pil/host_sim_main.c` or `c/test/test_main.c` --
those are host-only (they use `stdio.h`/`clock_gettime`, which don't
exist in the embedded build).

## 6. Wire `main_pil_loop.c` into the generated `main.c`

Open the CubeMX-generated `Core/Src/main.c`. Add near the top:

```c
/* USER CODE BEGIN Includes */
#include "pil_core.h"
void pil_loop_init(void);
void pil_loop_run_forever(void);
/* USER CODE END Includes */
```

In `int main(void)`, after all the `MX_*_Init()` calls:

```c
/* USER CODE BEGIN 2 */
pil_loop_init();
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
pil_loop_run_forever();   /* never returns */
/* USER CODE END WHILE */
```

Leave the rest of CubeMX's generated `main.c` untouched -- only add
inside the `USER CODE BEGIN/END` markers, so regenerating the project
later (e.g. after changing a CubeMX peripheral setting) won't wipe
your changes.

## 7. Build

**Project -> Build Project** (or the hammer icon). Expect a clean
build with the same "zero warnings under -Wall -Wextra" bar the host
build holds -- if CubeIDE's default project doesn't pass `-Wextra`,
add it under **Project Properties -> C/C++ Build -> Settings ->
MCU GCC Compiler -> Warnings**, matching `c/Makefile`'s flags, for an
apples-to-apples comparison with the host build.

## 8. Flash

Plug in the Nucleo via USB. **Run -> Debug** (or the bug icon) uses
the onboard ST-LINK automatically -- no external programmer needed.
First run may prompt to switch to the Debug perspective; accept it,
then either let it run or just flash-and-reset if you don't need to
step through code.

## 9. Verify: find the COM port and run the smoke test

**Windows**: Device Manager -> Ports (COM & LPT) -> look for
"STMicroelectronics STLink Virtual COM Port (COMx)".

**Linux**: `ls /dev/ttyACM*`.

From the PC side (no cross-repo dependency needed for this first
check):

```
pip install pyserial
cd python/scripts
python3 serial_ping_test.py --port COM3        # Windows
python3 serial_ping_test.py --port /dev/ttyACM0  # Linux
```

Expect 10/10 steps `OK`, `theta_est` near the baked-in trim value
(`-0.02369` rad), and a real per-step round-trip time (should be low
single-digit milliseconds dominated by the 115200-baud transfer time
for ~127+114 bytes, not by computation -- the DWT `cycles` field in
each line is the actual on-target compute time, and should be a small
fraction of that).

## 10. Run the full closed-loop PIL demo against real hardware

Requires `lifting-body-gnc` cloned as a sibling directory (see
`python/scripts/pil_driver.py`'s docstring):

```
cd python/scripts
PYTHONPATH=../../lifting-body-gnc/python python3 pil_driver.py \
    --backend serial --port COM3
```

This runs the identical 30 s scenario used in the host-simulated PIL
(`docs/demo_pil_hostsim.png` in lifting-body-gnc) against the real
board, producing `docs/demo_pil_serial.png` here. Compare the two
plots directly -- they should be visually and numerically very close;
any meaningful divergence is worth investigating (candidates: UART
timing/framing issues, the double vs. the F401's single-precision FPU
interacting with compiler optimization flags, or a HAL configuration
difference from the host build).

## Troubleshooting

- **CubeIDE doesn't detect the board during flash**: check the
  ST-LINK solder-bridge jumpers are in their default (factory)
  position -- see the Nucleo-F401RE user manual (UM1724), section on
  ST-LINK/target power and SWD routing.
- **`serial_ping_test.py` times out / no response**: confirm the COM
  port number is correct and no other program (a terminal emulator,
  the CubeIDE debugger console) has it open.
- **Garbled bytes / wrong checksum on every packet**: almost always a
  baud rate or word-length/parity mismatch between the `.ioc` USART2
  config and the Python script's `--baud` argument.
- **Stack overflow / hard fault**: shouldn't occur after the Phase 4a
  fix (peak usage ~1.3 KB), but if you're debugging one, CubeIDE's
  Debug perspective shows the stack pointer and a live memory view --
  check it against `_Min_Stack_Size` in the linker script.
