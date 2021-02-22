This Rework based on Demo RT-STM32F07-Discovery in ChibiOS/RT 3.0.6

### Pin List

##### UART2-BTHC05
- PA2 (TX) - HC05 (RX)
- PA3 (RX) - HC05 (TX)

##### Motor 1
- PE4 (GPIO_Input)
- PC12 (GPIO_EXT12)
- PA0 (TIM5_CH1)
- PA1 (TIM5_CH2)

##### Motor 2
- PE11 (GPIO_Input)
- PE9 (GPIO_EXT19)
- PC6 (TIM8_CH1)
- PC7 (TIM8_CH2)

##### Motor 3
- PE5 (GPIO_Input)
- PD0 (GPIO_EXT10)
- PB0 (TIM3_CH3)
- PB1 (TIM3_CH4)

##### I2C
- PB7 (I2C1_SDA)
- PB6 (I2C1_SCL)

##### RCC
- PH0.. (RCC_OSC_IN)
- PH1.. (RCC_OSC_OUT)

### Special Config

### To Do
Currently done or implemented:
- [ ] Demo Clean-Up
  - [x] Remove Unused
  - [ ] Default Pinout
  - [ ] Default Clock
- [ ] Communication
  - [ ] Serial UART Shell
    - [x] Command: Test
