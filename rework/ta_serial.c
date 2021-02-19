/*
    Embedded Audiometri - Copyright (C) 2020 Achmadi
    github.com/mekatronik-achmadi/ or mekatronik-achmadi@gmail.com
    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at
        http://www.apache.org/licenses/LICENSE-2.0
    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    ta_serial.c
 * @brief   Communcation code.
 *
 * @addtogroup Communication
 * @{
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "ta_serial.h"

/*******************************************
 * Serial Command Callback
 *******************************************/

/**
 * @brief Test command callback
 * @details Enumerated and not called directly by any normal thread
 */
static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argc;
    (void) argv;

    chprintf(chp,"Serial Working\r\n");
}

/**
 * @brief Shell command and it's callback enumeration
 * @details Extending from internal shell's callback
 */
static const ShellCommand commands[] = {
    {"test",cmd_test},
    {NULL, NULL}
};

/*******************************************
 * Serial Peripheral Setup
 *******************************************/

/**
 * @brief UART Shell Console pointer
 */
static thread_t *shelltp_uart = NULL;

/**
 * @brief Shell Driver Config
 * @details Serial Interface using UART0 (SD1)
 */
static const ShellConfig shell_uart_cfg = {
  (BaseSequentialStream *)&SERIAL_UART,
  commands
};

/**
 * @brief Initiate UART Driver, its pin, and Shell thread
 */
void TA_SerialInit(void){
    sdStart(&SERIAL_UART, NULL);
    palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7) | PAL_STM32_OSPEED_HIGHEST);

    shellInit();
}

/**
 * @brief Keeping Shell thread alive and check it periodically
 * @details Should invoked in main loop or other thread.
 */
void TA_SerialLoop(void){
    if (!shelltp_uart) {
            shelltp_uart = shellCreate(&shell_uart_cfg, SHELL_WA_SIZE, NORMALPRIO);
        }
        else {
            if (chThdTerminatedX(shelltp_uart)) {
                chThdRelease(shelltp_uart);
                shelltp_uart = NULL;
            }
        }
}
/** @} */
