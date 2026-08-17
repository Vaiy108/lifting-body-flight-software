/* main_pil_loop.c -- STM32 Nucleo-F401RE firmware for the lifting-body
 * GNC PIL demo.
 *
 * This file is NOT a complete CubeIDE project by itself -- it is the
 * application logic to drop into main.c's USER CODE sections after
 * generating the CubeMX project skeleton. See ../BRINGUP.md for the
 * full step-by-step CubeMX configuration and integration instructions.
 *
 * Architecture: identical to c/pil/host_sim_main.c (which this file
 * intentionally mirrors function-for-function), except:
 *   - I/O is HAL_UART_Receive/Transmit instead of stdin/stdout
 *   - Timing uses the DWT cycle counter instead of clock_gettime()
 *   - There is no "process" to exit; this runs forever in main()'s loop
 *
 * pil_core_step() itself -- the actual GNC logic -- is untouched,
 * vendored unmodified from lifting-body-gnc, and already verified via
 * SIL cross-validation (c/test/test_main.c) and host-simulated PIL
 * (c/pil/host_sim_main.c + python/scripts/pil_driver.py --backend
 * hostsim) before ever reaching this file. What's being verified HERE,
 * for the first time, is only the platform glue: does real UART
 * hardware, real interrupt/DMA timing, and the real Cortex-M4F FPU
 * (single-precision-capable, computing in double here) reproduce the
 * same behavior as the host build.
 */

#include "main.h"          /* CubeMX-generated: HAL handles, pin defs */
#include "pil_protocol.h"
#include "pil_core.h"
#include <string.h>

/* CubeMX generates this in main.c; declared here for this file's
 * standalone reference -- in the real project this comes from the
 * generated main.c, not from this file. */
extern UART_HandleTypeDef huart2;   /* USART2 = ST-LINK VCP on Nucleo boards */

/* ---------------------------------------------------------------------
 * DWT cycle counter: gives cycle-accurate timing without a hardware
 * timer peripheral. Must be enabled once at startup.
 * --------------------------------------------------------------------- */
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t dwt_cycles(void) {
    return DWT->CYCCNT;
}

/* ---------------------------------------------------------------------
 * Blocking receive of exactly one PilInputPacket. HAL_UART_Receive
 * blocks (with timeout) rather than using interrupts/DMA -- correct
 * choice for a lockstep protocol (the board legitimately has nothing
 * else to do while waiting for the next PC command). Interrupt/DMA-
 * driven reception is a documented upgrade path if this firmware ever
 * needs to do anything concurrently with waiting for input.
 * --------------------------------------------------------------------- */
static HAL_StatusTypeDef pil_receive_packet(PilInputPacket *pkt,
                                            uint32_t timeout_ms) {
    return HAL_UART_Receive(&huart2, (uint8_t *)pkt, sizeof(*pkt), timeout_ms);
}

static HAL_StatusTypeDef pil_send_packet(const PilOutputPacket *pkt) {
    return HAL_UART_Transmit(&huart2, (const uint8_t *)pkt, sizeof(*pkt),
                             100);
}

/* ---------------------------------------------------------------------
 * Called once from main(), after all CubeMX-generated HAL_*_Init()
 * calls, in the USER CODE BEGIN 2 section.
 * --------------------------------------------------------------------- */
void pil_loop_init(void) {
    dwt_init();
    pil_core_init();
}

/* ---------------------------------------------------------------------
 * Called in an infinite loop from main()'s USER CODE BEGIN WHILE
 * section (replacing CubeMX's default `while (1) { }` body).
 * --------------------------------------------------------------------- */
void pil_loop_run_forever(void) {
    PilInputPacket in;
    PilOutputPacket out;

    for (;;) {
        HAL_StatusTypeDef rx = pil_receive_packet(&in, HAL_MAX_DELAY);
        if (rx != HAL_OK) {
            /* UART framing/timeout error: nothing meaningful to
             * respond with (we don't have a valid `in.seq` to echo),
             * so just resynchronize by waiting for the next packet.
             * A production link would add a resync/framing-recovery
             * strategy here; out of scope for this PIL demo. */
            continue;
        }

        uint32_t t0 = dwt_cycles();
        int rc = pil_core_step(&in, &out);
        uint32_t t1 = dwt_cycles();

        if (rc != 0) {
            /* checksum failure on the input packet */
            memset(&out, 0, sizeof(out));
            out.magic = PIL_OUTPUT_MAGIC;
            out.seq = in.seq;
            out.cycle_count = 0xFFFFFFFFu;
            pil_output_set_checksum(&out);
        } else {
            out.cycle_count = t1 - t0;   /* real hardware cycle count */
            pil_output_set_checksum(&out);
        }

        pil_send_packet(&out);
    }
}

/* ---------------------------------------------------------------------
 * Reference for where these hook into CubeMX's generated main.c:
 *
 *   int main(void) {
 *     HAL_Init();
 *     SystemClock_Config();
 *     MX_GPIO_Init();
 *     MX_USART2_UART_Init();
 *     // USER CODE BEGIN 2
 *     pil_loop_init();
 *     // USER CODE END 2
 *     // USER CODE BEGIN WHILE
 *     pil_loop_run_forever();   // never returns
 *     // USER CODE END WHILE
 *   }
 * --------------------------------------------------------------------- */
