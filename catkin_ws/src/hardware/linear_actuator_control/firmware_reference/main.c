/**
 ******************************************************************************
 * @file    main.c
 * @brief   STM32F103RCT6: Actuator + Feetech ST3215 ID1/ID2
 *
 * ThinkPad/ROS1 <-> CP2102 <-> USART1 (PA9/PA10), 115200 8N1
 * STM32 USART2 (PA2/PA3), UART <-> Waveshare Bus Servo Adapter (A)
 * H-bridge: PB6 -> PWML1, PB7 -> PWML2, PB8 -> EN
 *
 * Waveshare Adapter UART mode:
 *   PA2 (USART2_TX) -> Adapter TX
 *   PA3 (USART2_RX) -> Adapter RX
 *   GND             -> Adapter GND
 *   jumper          -> A
 *
 * Servo IDs used by this firmware:
 *   ID 1 = PAN
 *   ID 2 = TILT
 *
 * PC console commands (servo commands need Enter):
 *   PING
 *   READ
 *   PAN <0..360>
 *   TILT <0..360>
 *   PT <pan_deg> <tilt_deg>
 *   BAUD <rate>  (switch UART2 rate until next reset)
 *   HELP
 *   ID2 is intentionally DISABLED; IDs are already configured.
 *
 * Linear actuator immediate commands:
 *   U/u = extend (must be refreshed; 500ms watchdog)
 *   D/d = retract (must be refreshed; 500ms watchdog)
 *   S/s = brake stop
 *   F/f = coast
 ******************************************************************************
 */

#include <stdint.h>

/* ========================= Clock =========================
 * HSI 8MHz / 2 * 16 = 64MHz SYSCLK
 * APB1 = 32MHz (USART2)
 * APB2 = 64MHz (USART1)
 */
#define SYSCLK_HZ              64000000U
#define PCLK1_HZ               32000000U
#define PCLK2_HZ               64000000U

#define PC_BAUDRATE            115200U
#define SERVO_DEFAULT_BAUD      1000000U

#define ACTUATOR_TIMEOUT_MS    500U
#define REVERSE_DELAY_MS       100U

#define SERVO_PAN_ID           1U
#define SERVO_TILT_ID          2U

/* Conservative starting motion settings */
#define SERVO_DEFAULT_SPEED    200U
#define SERVO_DEFAULT_ACC      20U

/* ========================= Registers ========================= */

#define RCC_CR          (*(volatile uint32_t *)0x40021000U)
#define RCC_CFGR        (*(volatile uint32_t *)0x40021004U)
#define RCC_APB2ENR     (*(volatile uint32_t *)0x40021018U)
#define RCC_APB1ENR     (*(volatile uint32_t *)0x4002101CU)

#define FLASH_ACR       (*(volatile uint32_t *)0x40022000U)

#define GPIOA_CRL       (*(volatile uint32_t *)0x40010800U)
#define GPIOA_CRH       (*(volatile uint32_t *)0x40010804U)

#define GPIOB_CRL       (*(volatile uint32_t *)0x40010C00U)
#define GPIOB_CRH       (*(volatile uint32_t *)0x40010C04U)
#define GPIOB_BSRR      (*(volatile uint32_t *)0x40010C10U)
#define GPIOB_BRR       (*(volatile uint32_t *)0x40010C14U)

/* USART1: PC / CP2102 */
#define USART1_SR       (*(volatile uint32_t *)0x40013800U)
#define USART1_DR       (*(volatile uint32_t *)0x40013804U)
#define USART1_BRR      (*(volatile uint32_t *)0x40013808U)
#define USART1_CR1      (*(volatile uint32_t *)0x4001380CU)

/* USART2: Waveshare servo adapter */
#define USART2_SR       (*(volatile uint32_t *)0x40004400U)
#define USART2_DR       (*(volatile uint32_t *)0x40004404U)
#define USART2_BRR      (*(volatile uint32_t *)0x40004408U)
#define USART2_CR1      (*(volatile uint32_t *)0x4000440CU)

#define USART_SR_RXNE   (1U << 5)
#define USART_SR_ORE    (1U << 3)
#define USART_SR_FE     (1U << 1)
#define USART_SR_NE     (1U << 2)
#define USART_SR_TC     (1U << 6)
#define USART_SR_TXE    (1U << 7)

#define USART_CR1_RE    (1U << 2)
#define USART_CR1_TE    (1U << 3)
#define USART_CR1_UE    (1U << 13)

#define SYST_CSR        (*(volatile uint32_t *)0xE000E010U)
#define SYST_RVR        (*(volatile uint32_t *)0xE000E014U)
#define SYST_CVR        (*(volatile uint32_t *)0xE000E018U)

/* ========================= H-bridge pins ========================= */

#define PWML1_PIN       6U
#define PWML2_PIN       7U
#define EN_PIN          8U

/* ========================= Feetech protocol ========================= */

#define STS_INST_PING           0x01U
#define STS_INST_READ           0x02U
#define STS_INST_WRITE          0x03U
#define STS_INST_SYNC_WRITE     0x83U

#define STS_ADDR_TORQUE_ENABLE  0x28U
#define STS_ADDR_ACC            0x29U
#define STS_ADDR_GOAL_POSITION  0x2AU
#define STS_ADDR_GOAL_SPEED     0x2EU
#define STS_ADDR_PRESENT_POS    0x38U

#define STS_BROADCAST_ID        0xFEU

/* ========================= State ========================= */

typedef enum
{
    ACTUATOR_STOPPED = 0,
    ACTUATOR_EXTENDING,
    ACTUATOR_RETRACTING,
    ACTUATOR_COASTING
} ActuatorState;

static volatile uint32_t g_ms = 0U;
static volatile ActuatorState g_actuator_state = ACTUATOR_STOPPED;
static volatile uint32_t g_last_actuator_command_ms = 0U;
static uint32_t g_servo_baud = SERVO_DEFAULT_BAUD;
static uint8_t g_last_tx[32];
static uint8_t g_last_tx_len = 0U;

/* ========================= Clock ========================= */

static void Clock_Init_64MHz(void)
{
    RCC_CR |= (1U << 0);
    while ((RCC_CR & (1U << 1)) == 0U)
    {
    }

    /* Prefetch + 2 wait states */
    FLASH_ACR = (1U << 4) | 2U;

    /* APB1 /2, APB2 /1, PLL = HSI/2 *16 = 64MHz */
    RCC_CFGR = 0U;
    RCC_CFGR |= (4U << 8);
    RCC_CFGR |= (14U << 18);

    RCC_CR |= (1U << 24);
    while ((RCC_CR & (1U << 25)) == 0U)
    {
    }

    RCC_CFGR = (RCC_CFGR & ~0x3U) | 0x2U;
    while (((RCC_CFGR >> 2) & 0x3U) != 0x2U)
    {
    }
}

/* ========================= SysTick ========================= */

void SysTick_Handler(void)
{
    g_ms++;
}

static void SysTick_Init(void)
{
    SYST_RVR = (SYSCLK_HZ / 1000U) - 1U;
    SYST_CVR = 0U;
    SYST_CSR = 0x07U;
}

static void Delay_ms(uint32_t delay)
{
    uint32_t start = g_ms;
    while ((uint32_t)(g_ms - start) < delay)
    {
    }
}

/* ========================= GPIO ========================= */

static void Pin_High(uint32_t pin)
{
    GPIOB_BSRR = (1U << pin);
}

static void Pin_Low(uint32_t pin)
{
    GPIOB_BRR = (1U << pin);
}

static void GPIO_Init(void)
{
    RCC_APB2ENR |= (1U << 0) | (1U << 2) | (1U << 3);

    /* Safe H-bridge state before output mode */
    Pin_High(PWML1_PIN);
    Pin_High(PWML2_PIN);
    Pin_High(EN_PIN);

    /* PB6/PB7/PB8: push-pull 2MHz */
    GPIOB_CRL &= ~(0xFU << 24);
    GPIOB_CRL |=  (0x2U << 24);

    GPIOB_CRL &= ~(0xFU << 28);
    GPIOB_CRL |=  (0x2U << 28);

    GPIOB_CRH &= ~(0xFU << 0);
    GPIOB_CRH |=  (0x2U << 0);

    /* PA9 USART1_TX: AF push-pull, 2MHz */
    GPIOA_CRH &= ~(0xFU << 4);
    GPIOA_CRH |=  (0xAU << 4);

    /* PA10 USART1_RX: floating input */
    GPIOA_CRH &= ~(0xFU << 8);
    GPIOA_CRH |=  (0x4U << 8);

    /* PA2 USART2_TX: AF push-pull, 10MHz */
    GPIOA_CRL &= ~(0xFU << 8);
    GPIOA_CRL |=  (0x9U << 8);

    /* PA3 USART2_RX: floating input */
    GPIOA_CRL &= ~(0xFU << 12);
    GPIOA_CRL |=  (0x4U << 12);
}

/* ========================= UART1: PC ========================= */

static void UART1_Init(void)
{
    RCC_APB2ENR |= (1U << 14);
    USART1_CR1 = 0U;
    USART1_BRR = (PCLK2_HZ + (PC_BAUDRATE / 2U)) / PC_BAUDRATE;
    USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void UART1_WriteChar(char c)
{
    while ((USART1_SR & USART_SR_TXE) == 0U)
    {
    }
    USART1_DR = (uint32_t)(uint8_t)c;
}

static void UART1_WriteString(const char *s)
{
    while (*s != '\0')
    {
        UART1_WriteChar(*s++);
    }
}

static int UART1_ReadChar(void)
{
    if ((USART1_SR & USART_SR_RXNE) != 0U)
    {
        return (int)(USART1_DR & 0xFFU);
    }
    return -1;
}

static void UART1_WriteUInt(uint32_t v)
{
    char buf[11];
    int i = 0;

    if (v == 0U)
    {
        UART1_WriteChar('0');
        return;
    }

    while ((v > 0U) && (i < 10))
    {
        buf[i++] = (char)('0' + (v % 10U));
        v /= 10U;
    }

    while (i > 0)
    {
        UART1_WriteChar(buf[--i]);
    }
}

/* ========================= UART2: servo adapter ========================= */

static void UART2_SetBaud(uint32_t baud)
{
    USART2_CR1 = 0U;
    USART2_BRR = (PCLK1_HZ + (baud / 2U)) / baud;
    USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    g_servo_baud = baud;
}

static void UART2_Init(void)
{
    RCC_APB1ENR |= (1U << 17);
    UART2_SetBaud(SERVO_DEFAULT_BAUD);
}

static void UART2_WriteByte(uint8_t b)
{
    while ((USART2_SR & USART_SR_TXE) == 0U)
    {
    }
    USART2_DR = b;
}

static void UART2_WaitTC(void)
{
    while ((USART2_SR & USART_SR_TC) == 0U)
    {
    }
}

static void UART2_FlushRx(void)
{
    uint32_t sr;
    uint32_t dummy;
    do
    {
        sr = USART2_SR;
        if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) == 0U)
            break;
        dummy = USART2_DR;
        (void)dummy;
    } while (1);
}

static int UART2_TryReadByte(void)
{
    uint32_t sr = USART2_SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) == 0U)
        return -1;

    /* On STM32F1 reading SR then DR also clears receive error flags. */
    uint8_t b = (uint8_t)(USART2_DR & 0xFFU);
    if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U)
        return -1;
    return (int)b;
}

/* ========================= H-bridge ========================= */

static void Driver_Brake(void)
{
    Pin_Low(EN_PIN);
    Pin_High(PWML1_PIN);
    Pin_High(PWML2_PIN);
}

static void Driver_Coast(void)
{
    Pin_High(PWML1_PIN);
    Pin_High(PWML2_PIN);
    Pin_High(EN_PIN);
}

static void Driver_Retract(void)
{
    Pin_Low(EN_PIN);
    Pin_Low(PWML1_PIN);
    Pin_High(PWML2_PIN);
}

static void Driver_Extend(void)
{
    Pin_Low(EN_PIN);
    Pin_High(PWML1_PIN);
    Pin_Low(PWML2_PIN);
}

static void Actuator_Stop(void)
{
    Driver_Brake();
    g_actuator_state = ACTUATOR_STOPPED;
}

static void Actuator_Coast(void)
{
    Driver_Coast();
    g_actuator_state = ACTUATOR_COASTING;
}

static void Actuator_Extend(void)
{
    if (g_actuator_state == ACTUATOR_RETRACTING)
    {
        Driver_Brake();
        Delay_ms(REVERSE_DELAY_MS);
    }

    Driver_Extend();
    g_actuator_state = ACTUATOR_EXTENDING;
    g_last_actuator_command_ms = g_ms;
}

static void Actuator_Retract(void)
{
    if (g_actuator_state == ACTUATOR_EXTENDING)
    {
        Driver_Brake();
        Delay_ms(REVERSE_DELAY_MS);
    }

    Driver_Retract();
    g_actuator_state = ACTUATOR_RETRACTING;
    g_last_actuator_command_ms = g_ms;
}

/* ========================= Feetech packet layer ========================= */

static uint8_t STS_Checksum(uint8_t id,
                            uint8_t length,
                            uint8_t instruction,
                            const uint8_t *params,
                            uint8_t param_count)
{
    uint16_t sum = (uint16_t)id + length + instruction;
    uint8_t i;

    for (i = 0U; i < param_count; i++)
    {
        sum += params[i];
    }

    return (uint8_t)(~((uint8_t)sum));
}

static void STS_SendPacket(uint8_t id,
                           uint8_t instruction,
                           const uint8_t *params,
                           uint8_t param_count)
{
    uint8_t length = (uint8_t)(param_count + 2U);
    uint8_t checksum = STS_Checksum(id, length, instruction, params, param_count);
    uint8_t i;

    /* Remember exactly what we sent so an adapter's echo is not a reply. */
    if ((uint32_t)param_count + 6U <= sizeof(g_last_tx))
    {
        g_last_tx_len = (uint8_t)(param_count + 6U);
        g_last_tx[0] = 0xFFU;
        g_last_tx[1] = 0xFFU;
        g_last_tx[2] = id;
        g_last_tx[3] = length;
        g_last_tx[4] = instruction;
        for (i = 0U; i < param_count; ++i)
            g_last_tx[5U + i] = params[i];
        g_last_tx[5U + param_count] = checksum;
    }
    else
    {
        g_last_tx_len = 0U;
    }

    UART2_FlushRx();

    /* RX is not needed during TX. Disabling it prevents local TX echo
       and receive overrun on this adapter's two-wire UART interface. */
    USART2_CR1 &= ~USART_CR1_RE;
    UART2_WriteByte(0xFFU);
    UART2_WriteByte(0xFFU);
    UART2_WriteByte(id);
    UART2_WriteByte(length);
    UART2_WriteByte(instruction);
    for (i = 0U; i < param_count; ++i)
        UART2_WriteByte(params[i]);
    UART2_WriteByte(checksum);
    UART2_WaitTC();
    USART2_CR1 |= USART_CR1_RE;
}

static int STS_ReadStatus(uint8_t expected_id,
                          uint8_t expected_params,
                          uint8_t *error,
                          uint8_t *params,
                          uint32_t timeout_ms)
{
    uint8_t frame[32];
    uint8_t count = 0U;
    uint8_t frame_total = 0U;
    uint32_t started = g_ms;

    while ((uint32_t)(g_ms - started) < timeout_ms)
    {
        int c = UART2_TryReadByte();
        uint8_t j;
        uint8_t sum = 0U;
        int is_echo = 0;
        if (c < 0)
            continue;

        if (count == 0U)
        {
            if (c == 0xFF)
                frame[count++] = (uint8_t)c;
            continue;
        }
        if (count == 1U)
        {
            if (c == 0xFF)
                frame[count++] = (uint8_t)c;
            else
                count = 0U;
            continue;
        }

        frame[count++] = (uint8_t)c;
        if (count == 4U)
        {
            /* LENGTH counts ERROR + CHECKSUM + parameters, not header. */
            if (frame[3] < 2U || frame[3] > 26U)
            {
                count = 0U;
                continue;
            }
            frame_total = (uint8_t)(frame[3] + 4U);
        }
        if (frame_total == 0U || count < frame_total)
            continue;

        for (j = 2U; j < frame_total - 1U; ++j)
            sum = (uint8_t)(sum + frame[j]);

        if (frame_total == g_last_tx_len)
        {
            is_echo = 1;
            for (j = 0U; j < frame_total; ++j)
            {
                if (frame[j] != g_last_tx[j])
                {
                    is_echo = 0;
                    break;
                }
            }
        }

        if (!is_echo &&
            frame[2] == expected_id &&
            frame[3] == (uint8_t)(expected_params + 2U) &&
            frame[frame_total - 1U] == (uint8_t)(~sum))
        {
            *error = frame[4];
            for (j = 0U; j < expected_params; ++j)
                params[j] = frame[5U + j];
            return 1;
        }

        count = 0U;
        frame_total = 0U;
    }
    return 0;
}

static int STS_Ping(uint8_t id, uint8_t *servo_error)
{
    uint8_t ignored = 0U;
    STS_SendPacket(id, STS_INST_PING, 0, 0U);
    return STS_ReadStatus(id, 0U, servo_error, &ignored, 30U);
}

static void STS_DrainReply(uint32_t wait_ms)
{
    uint32_t start = g_ms;

    while ((uint32_t)(g_ms - start) < wait_ms)
    {
        if ((USART2_SR & USART_SR_RXNE) != 0U)
        {
            (void)USART2_DR;
        }
    }
}

static void STS_WriteByteNoCheck(uint8_t id, uint8_t address, uint8_t value)
{
    uint8_t p[2];

    p[0] = address;
    p[1] = value;

    STS_SendPacket(id, STS_INST_WRITE, p, 2U);
    STS_DrainReply(3U);
}

static int STS_ReadPosition(uint8_t id, uint16_t *position)
{
    uint8_t req[2];
    uint8_t error = 0U;
    uint8_t data[4];
    int n;

    req[0] = STS_ADDR_PRESENT_POS;
    req[1] = 2U;

    STS_SendPacket(id, STS_INST_READ, req, 2U);
    n = STS_ReadStatus(id, 2U, &error, data, 30U);

    if (!n || (error != 0U))
    {
        return 0;
    }

    *position = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return 1;
}

static void STS_EnableTorque(uint8_t id, uint8_t enable)
{
    STS_WriteByteNoCheck(id, STS_ADDR_TORQUE_ENABLE, enable ? 1U : 0U);
}

static void STS_WritePosition(uint8_t id,
                              uint16_t position,
                              uint16_t speed,
                              uint8_t acc)
{
    uint8_t p[8];

    if (position > 4095U)
    {
        position = 4095U;
    }

    /* Start at 0x29: ACC + POS(2) + reserved/time(2) + SPEED(2) */
    p[0] = STS_ADDR_ACC;
    p[1] = acc;
    p[2] = (uint8_t)(position & 0xFFU);
    p[3] = (uint8_t)((position >> 8) & 0xFFU);
    p[4] = 0U;
    p[5] = 0U;
    p[6] = (uint8_t)(speed & 0xFFU);
    p[7] = (uint8_t)((speed >> 8) & 0xFFU);

    STS_SendPacket(id, STS_INST_WRITE, p, 8U);
    STS_DrainReply(3U);
}

static void STS_SyncWriteTwoPositions(uint16_t pan_position,
                                      uint16_t tilt_position,
                                      uint16_t speed,
                                      uint8_t acc)
{
    uint8_t p[18];
    uint8_t i = 0U;

    if (pan_position > 4095U) pan_position = 4095U;
    if (tilt_position > 4095U) tilt_position = 4095U;

    p[i++] = STS_ADDR_ACC;
    p[i++] = 7U;

    p[i++] = SERVO_PAN_ID;
    p[i++] = acc;
    p[i++] = (uint8_t)(pan_position & 0xFFU);
    p[i++] = (uint8_t)((pan_position >> 8) & 0xFFU);
    p[i++] = 0U;
    p[i++] = 0U;
    p[i++] = (uint8_t)(speed & 0xFFU);
    p[i++] = (uint8_t)((speed >> 8) & 0xFFU);

    p[i++] = SERVO_TILT_ID;
    p[i++] = acc;
    p[i++] = (uint8_t)(tilt_position & 0xFFU);
    p[i++] = (uint8_t)((tilt_position >> 8) & 0xFFU);
    p[i++] = 0U;
    p[i++] = 0U;
    p[i++] = (uint8_t)(speed & 0xFFU);
    p[i++] = (uint8_t)((speed >> 8) & 0xFFU);

    STS_SendPacket(STS_BROADCAST_ID, STS_INST_SYNC_WRITE, p, i);
}

/* ========================= Conversion / parser ========================= */

static uint16_t DegreeToPosition(uint32_t degree)
{
    if (degree > 360U)
    {
        degree = 360U;
    }

    if (degree == 360U)
    {
        return 4095U;
    }

    return (uint16_t)((degree * 4095U + 180U) / 360U);
}

static uint32_t PositionToDegree(uint16_t pos)
{
    if (pos > 4095U)
    {
        pos = 4095U;
    }

    return ((uint32_t)pos * 360U + 2047U) / 4095U;
}

static int IsSpace(char c)
{
    return (c == ' ') || (c == '\t');
}

static int ParseUInt(const char **s, uint32_t *value)
{
    uint32_t v = 0U;
    int got = 0;

    while (IsSpace(**s))
    {
        (*s)++;
    }

    while ((**s >= '0') && (**s <= '9'))
    {
        got = 1;
        v = v * 10U + (uint32_t)(**s - '0');
        (*s)++;
    }

    if (!got)
    {
        return 0;
    }

    *value = v;
    return 1;
}

static int StartsWith(const char *s, const char *prefix)
{
    while (*prefix != '\0')
    {
        char a = *s;
        char b = *prefix;

        if ((a >= 'a') && (a <= 'z')) a = (char)(a - 'a' + 'A');
        if ((b >= 'a') && (b <= 'z')) b = (char)(b - 'a' + 'A');

        if (a != b)
        {
            return 0;
        }

        s++;
        prefix++;
    }

    return 1;
}

static int SupportedBaud(uint32_t baud)
{
    return (baud == 1000000U || baud == 500000U ||
            baud == 250000U || baud == 128000U ||
            baud == 115200U  || baud == 76800U ||
            baud == 57600U  || baud == 38400U);
}

static void PrintServoPing(uint8_t id)
{
    uint8_t error = 0U;
    UART1_WriteString("ID");
    UART1_WriteUInt(id);
    UART1_WriteString(": ");
    if (!STS_Ping(id, &error))
    {
        UART1_WriteString("NO VALID REPLY\r\n");
    }
    else if (error != 0U)
    {
        UART1_WriteString("REPLY WITH ERROR=");
        UART1_WriteUInt(error);
        UART1_WriteString("\r\n");
    }
    else
    {
        UART1_WriteString("OK\r\n");
    }
}

static void PrintServoPosition(uint8_t id)
{
    uint16_t pos;
    UART1_WriteString("ID");
    UART1_WriteUInt(id);
    UART1_WriteString(": ");
    if (!STS_ReadPosition(id, &pos))
    {
        UART1_WriteString("NO VALID POSITION REPLY\r\n");
        return;
    }
    UART1_WriteString("POS=");
    UART1_WriteUInt(pos);
    UART1_WriteString(" DEG~");
    UART1_WriteUInt(PositionToDegree(pos));
    UART1_WriteString("\r\n");
}

static void PrintHelp(void)
{
    UART1_WriteString(
        "\r\nDual ST3215 (ID1=PAN, ID2=TILT)\r\n"
        "  PING                query both IDs\r\n"
        "  READ                read current positions\r\n"
        "  PAN 180             set ID1 position\r\n"
        "  TILT 180            set ID2 position\r\n"
        "  PT 180 180          set both positions\r\n"
        "  BAUD 1000000        select servo UART rate (until reboot)\r\n"
        "  HELP                show commands\r\n"
        "  ID2                 disabled (IDs already configured)\r\n"
        "Actuator: U extend / D retract / S brake / F coast\r\n"
        "NOTE: SENT means transmitted, not verified movement.\r\n\r\n"
    );
}

static void ProcessLine(char *line)
{
    const char *p;
    uint32_t a;
    uint32_t b;

    if (StartsWith(line, "HELP"))
    {
        PrintHelp();
        return;
    }
    if (StartsWith(line, "BAUD"))
    {
        p = line + 4;
        if (!ParseUInt(&p, &a) || !SupportedBaud(a))
        {
            UART1_WriteString("ERR BAUD: choose 1000000,500000,250000,128000,115200,76800,57600,38400\r\n");
            return;
        }
        UART2_SetBaud(a);
        UART1_WriteString("SERVO UART BAUD -> ");
        UART1_WriteUInt(g_servo_baud);
        UART1_WriteString("\r\n");
        return;
    }
    if (StartsWith(line, "PING"))
    {
        PrintServoPing(SERVO_PAN_ID);
        PrintServoPing(SERVO_TILT_ID);
        return;
    }
    if (StartsWith(line, "READ"))
    {
        PrintServoPosition(SERVO_PAN_ID);
        PrintServoPosition(SERVO_TILT_ID);
        return;
    }
    if (StartsWith(line, "ID2"))
    {
        UART1_WriteString("ID CHANGE DISABLED: ID1 and ID2 are already configured.\r\n");
        return;
    }
    if (StartsWith(line, "PAN"))
    {
        p = line + 3;
        if (!ParseUInt(&p, &a) || a > 360U)
        {
            UART1_WriteString("ERR PAN <0..360>\r\n");
            return;
        }
        STS_EnableTorque(SERVO_PAN_ID, 1U);
        STS_WritePosition(SERVO_PAN_ID, DegreeToPosition(a),
                          SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACC);
        UART1_WriteString("ID1 PAN ");
        UART1_WriteUInt(a);
        UART1_WriteString(" SENT (not confirmed)\r\n");
        return;
    }
    if (StartsWith(line, "TILT"))
    {
        p = line + 4;
        if (!ParseUInt(&p, &a) || a > 360U)
        {
            UART1_WriteString("ERR TILT <0..360>\r\n");
            return;
        }
        STS_EnableTorque(SERVO_TILT_ID, 1U);
        STS_WritePosition(SERVO_TILT_ID, DegreeToPosition(a),
                          SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACC);
        UART1_WriteString("ID2 TILT ");
        UART1_WriteUInt(a);
        UART1_WriteString(" SENT (not confirmed)\r\n");
        return;
    }
    if (StartsWith(line, "PT"))
    {
        p = line + 2;
        if (!ParseUInt(&p, &a) || !ParseUInt(&p, &b) ||
            a > 360U || b > 360U)
        {
            UART1_WriteString("ERR PT <pan 0..360> <tilt 0..360>\r\n");
            return;
        }
        STS_EnableTorque(SERVO_PAN_ID, 1U);
        STS_EnableTorque(SERVO_TILT_ID, 1U);
        STS_SyncWriteTwoPositions(DegreeToPosition(a), DegreeToPosition(b),
                                  SERVO_DEFAULT_SPEED, SERVO_DEFAULT_ACC);
        UART1_WriteString("PT ");
        UART1_WriteUInt(a);
        UART1_WriteString(" ");
        UART1_WriteUInt(b);
        UART1_WriteString(" SENT (not confirmed)\r\n");
        return;
    }

    UART1_WriteString("ERR CMD (type HELP)\r\n");
}

/* ========================= Main ========================= */

int main(void)
{
    char line[64];
    uint8_t line_len = 0U;

    Clock_Init_64MHz();
    GPIO_Init();
    SysTick_Init();
    UART1_Init();
    UART2_Init();

    Actuator_Stop();

    UART1_WriteString("\r\nSTM32 DUAL ST3215 + ACTUATOR v2 READY\r\n");
    UART1_WriteString("PC UART: 115200\r\nSERVO UART: ");
    UART1_WriteUInt(g_servo_baud);
    UART1_WriteString("\r\nIDs: 1=PAN, 2=TILT\r\n");
    PrintHelp();

    while (1)
    {
        int c = UART1_ReadChar();

        if (c >= 0)
        {
            char ch = (char)c;

            if (line_len == 0U)
            {
                if ((ch == 'U') || (ch == 'u'))
                {
                    if (g_actuator_state != ACTUATOR_EXTENDING)
                    {
                        Actuator_Extend();
                    }
                    g_last_actuator_command_ms = g_ms;
                    continue;
                }

                if ((ch == 'D') || (ch == 'd'))
                {
                    if (g_actuator_state != ACTUATOR_RETRACTING)
                    {
                        Actuator_Retract();
                    }
                    g_last_actuator_command_ms = g_ms;
                    continue;
                }

                if ((ch == 'S') || (ch == 's'))
                {
                    Actuator_Stop();
                    UART1_WriteString("ACT STOP\r\n");
                    continue;
                }

                if ((ch == 'F') || (ch == 'f'))
                {
                    Actuator_Coast();
                    UART1_WriteString("ACT COAST\r\n");
                    continue;
                }
            }

            if ((ch == '\r') || (ch == '\n'))
            {
                if (line_len > 0U)
                {
                    line[line_len] = '\0';
                    ProcessLine(line);
                    line_len = 0U;
                }
            }
            else
            {
                if (line_len < (sizeof(line) - 1U))
                {
                    line[line_len++] = ch;
                }
                else
                {
                    line_len = 0U;
                    UART1_WriteString("ERR LINE TOO LONG\r\n");
                }
            }
        }

        if ((g_actuator_state == ACTUATOR_EXTENDING) ||
            (g_actuator_state == ACTUATOR_RETRACTING))
        {
            if ((uint32_t)(g_ms - g_last_actuator_command_ms) >
                ACTUATOR_TIMEOUT_MS)
            {
                Actuator_Stop();
                UART1_WriteString("WATCHDOG STOP\r\n");
            }
        }
    }
}
