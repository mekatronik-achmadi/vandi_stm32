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
 * @file    ta_serial.h
 * @brief   Communcation header.
 *
 * @addtogroup Communication
 * @{
 */

#ifndef TA_SERIAL_H
#define TA_SERIAL_H

/*
 * @brief Serial Driver to be used
 * @details Check mcuconf.h and it's implementation
 */
#define SERIAL_UART SD2

/*
 * @brief Serial Driver Baudrate
 * @details This macro used in halconf.
 * @details When using HC05, keep at 9600
 */
#define SERIAL_BAUD 9600

/*
 * @brief Serial Shell Thread Working size
 */
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(1024)

void TA_SerialInit(void);
void TA_SerialLoop(void);

#endif // TA_SERIAL_H
/** @} */
