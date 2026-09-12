#include "ti_msp_dl_config.h"
#include "vehicle_config.h"

static const DL_TimerG_ClockConfig pwm_clock_config = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 3U
};

static const DL_TimerG_PWMConfig pwm_config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 1000U,
    .startTimer = DL_TIMER_START
};

static const DL_UART_Main_ClockConfig uart_clock_config = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config uart_config = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

static void init_power(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(MOTOR_PWM_INST);
    DL_UART_Main_reset(WIRELESS_UART_INST);
    DL_UART_Main_reset(GYRO_UART_INST);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(MOTOR_PWM_INST);
    DL_UART_Main_enablePower(WIRELESS_UART_INST);
    DL_UART_Main_enablePower(GYRO_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

static void init_gpio(void)
{
    DL_GPIO_initPeripheralOutputFunction(MOTOR_PWM_LEFT_IOMUX, MOTOR_PWM_LEFT_FUNC);
    DL_GPIO_initPeripheralOutputFunction(MOTOR_PWM_RIGHT_IOMUX, MOTOR_PWM_RIGHT_FUNC);
    DL_GPIO_enableOutput(GPIOB, MOTOR_PWM_LEFT_PIN | MOTOR_PWM_RIGHT_PIN);

    DL_GPIO_initPeripheralOutputFunction(WIRELESS_UART_TX_IOMUX, WIRELESS_UART_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(WIRELESS_UART_RX_IOMUX, WIRELESS_UART_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(GYRO_UART_TX_IOMUX, GYRO_UART_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GYRO_UART_RX_IOMUX, GYRO_UART_RX_FUNC);

    DL_GPIO_initDigitalOutput(MOTOR_LEFT_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_LEFT_IN2_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_RIGHT_IN1_IOMUX);
    DL_GPIO_initDigitalOutput(MOTOR_RIGHT_IN2_IOMUX);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN |
        MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN);
    DL_GPIO_enableOutput(MOTOR_DIR_PORT, MOTOR_LEFT_IN1_PIN | MOTOR_LEFT_IN2_PIN |
        MOTOR_RIGHT_IN1_PIN | MOTOR_RIGHT_IN2_PIN);

    DL_GPIO_initDigitalInputFeatures(ENCODER_LEFT_A_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_LEFT_B_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_RIGHT_A_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_RIGHT_B_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_setLowerPinsPolarity(ENCODER_LEFT_PORT,
        DL_GPIO_PIN_0_EDGE_RISE | DL_GPIO_PIN_1_EDGE_RISE);
    DL_GPIO_setLowerPinsPolarity(ENCODER_RIGHT_PORT,
        DL_GPIO_PIN_6_EDGE_RISE | DL_GPIO_PIN_7_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(ENCODER_LEFT_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODER_RIGHT_PORT, ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);
    DL_GPIO_enableInterrupt(ENCODER_LEFT_PORT, ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN);
    DL_GPIO_enableInterrupt(ENCODER_RIGHT_PORT, ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN);

    DL_GPIO_initDigitalInput(LINE_L4_IOMUX);
    DL_GPIO_initDigitalInput(LINE_L3_IOMUX);
    DL_GPIO_initDigitalInput(LINE_L2_IOMUX);
    DL_GPIO_initDigitalInput(LINE_L1_IOMUX);
    DL_GPIO_initDigitalInput(LINE_R1_IOMUX);
    DL_GPIO_initDigitalInput(LINE_R2_IOMUX);
    DL_GPIO_initDigitalInput(LINE_R3_IOMUX);
    DL_GPIO_initDigitalInput(LINE_R4_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_SPI_MOSI_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SPI_CLOCK_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_RESET_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_DATA_COMMAND_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_CHIP_SELECT_IOMUX);
    DL_GPIO_clearPins(OLED_SPI_PORT, OLED_SPI_MOSI_PIN | OLED_SPI_CLOCK_PIN | OLED_DATA_COMMAND_PIN);
    DL_GPIO_setPins(OLED_SPI_PORT, OLED_RESET_PIN | OLED_CHIP_SELECT_PIN);
    DL_GPIO_enableOutput(OLED_SPI_PORT, OLED_SPI_ALL_PINS_MASK);

    DL_GPIO_initDigitalInputFeatures(BOARD_KEY_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MENU_BUTTON_UP_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MENU_BUTTON_DOWN_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MENU_BUTTON_BACK_IOMUX, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static void init_pwm(void)
{
    DL_TimerG_setClockConfig(MOTOR_PWM_INST, (DL_TimerG_ClockConfig *)&pwm_clock_config);
    DL_TimerG_initPWMMode(MOTOR_PWM_INST, (DL_TimerG_PWMConfig *)&pwm_config);
    DL_TimerG_setCounterControl(MOTOR_PWM_INST, DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND, DL_TIMER_CLC_CCCTL0_LCOND);
    DL_TimerG_setCaptureCompareOutCtl(MOTOR_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareOutCtl(MOTOR_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(MOTOR_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(MOTOR_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, 0U, DL_TIMER_CC_1_INDEX);
    DL_TimerG_enableClock(MOTOR_PWM_INST);
    DL_TimerG_setCCPDirection(MOTOR_PWM_INST, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
}

static void init_uart(UART_Regs *uart, uint32_t baud)
{
    DL_UART_Main_setClockConfig(uart, (DL_UART_Main_ClockConfig *)&uart_clock_config);
    DL_UART_Main_init(uart, (DL_UART_Main_Config *)&uart_config);
    DL_UART_Main_configBaudRate(uart, CPUCLK_FREQ, baud);
    DL_UART_Main_setRXFIFOThreshold(uart, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(uart, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(uart);
}

SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    init_power();
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_enableMFCLK();
    init_gpio();
    init_pwm();
    init_uart(WIRELESS_UART_INST, WIRELESS_UART_BAUD_RATE);
    init_uart(GYRO_UART_INST, G356_UART_BAUD_RATE);
    NVIC_SetPriority(GPIOA_INT_IRQn, 0U);
    NVIC_SetPriority(GPIOB_INT_IRQn, 0U);
    NVIC_SetPriority(GYRO_UART_INT_IRQN, 1U);
    NVIC_SetPriority(WIRELESS_UART_INT_IRQN, 2U);
}

SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void) { return true; }
SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void) { return true; }
