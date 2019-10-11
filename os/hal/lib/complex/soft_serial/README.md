# Software Serial Driver

## Usage

This is an example for STM32F103 MCU.

The driver needs a `Tick` function to be called at exactly X times the desired baud rate, in this example, the X is the define `SSD1_BITRATE_MULTIPLIER`, which is 4 times. The timer is setup using the register directly, but could be used a GPT and the Tick function in the callback.

### Setup

```c
#define SSD1_BAUD                     9600
#define SSD1_BITRATE_MULTIPLIER       4
#define SSD1_TX_LINE                  PAL_LINE(GPIOA, 1)
#define SSD1_RX_LINE                  PAL_LINE(GPIOA, 2)

#define SSD1_TIMER                    STM32_TIM3
#define SSD1_TIMER_CLOCK              STM32_TIMCLK1
#define SSD1_TIMER_HANDLER            STM32_TIM3_HANDLER
#define SSD1_TIMER_IRQ_NUMBER         STM32_TIM3_NUMBER
#define SSD1_TIMER_ENABLE()           rccEnableTIM3(true)
#define SSD1_TIMER_DISABLE()          rccDisableTIM3()
#define SSD1_TIMER_RESET()            rccResetTIM3()
#define SSD1_TIMER_FREQ               SSD1_BAUD * SSD1_BITRATE_MULTIPLIER
#define SSD1_TIMER_ARR                (uint16_t)((SSD1_TIMER_CLOCK / (SSD1_TIMER_FREQ)) - 1)
#define SSD1_TIMER_IRQ_PRIORITY       5

static const SoftSerialConfig ssd1_config = {
    SSD1_BITRATE_MULTIPLIER,
    false,
    SSD1_1_RX_LINE,
    SSD1_1_TX_LINE};

SoftSerialDriver SSD1;

/*
 * TIMER SETUP
 */

OSAL_IRQ_HANDLER(SSD1_TIMER_HANDLER)
{
    OSAL_IRQ_PROLOGUE();
    SSD_TIMER->SR = 0; // Clear pending IRQs

    osalSysLockFromISR();
    ssdTickI(&SSD1);
    osalSysUnlockFromISR();

    OSAL_IRQ_EPILOGUE();
}

static void timerStop(void)
{
    SSD1_TIMER->CR1 = 0;  // Timer disabled
    SSD1_TIMER->DIER = 0; // All IRQs disabled
    SSD1_TIMER->SR = 0;   // Clear pending IRQs

    nvicDisableVector(SSD1_TIMER_IRQ_NUMBER);
    SSD1_TIMER_DISABLE();
}

static void timerStart(void)
{
    SSD1_TIMER_ENABLE();
    SSD1_TIMER_RESET();

    nvicEnableVector(SSD1_TIMER_IRQ_NUMBER, SSD1_TIMER_IRQ_PRIORITY);

    SSD1_TIMER->CR1 = 0;  // Initially stopped
    SSD1_TIMER->CR2 = 0;  //
    SSD1_TIMER->PSC = 0;  // Prescaler value
    SSD1_TIMER->SR = 0;   // Clear pending IRQs
    SSD1_TIMER->DIER = 0; // DMA-related DIER bits
    SSD1_TIMER->PSC = 0;  // Prescaler value

    SSD1_TIMER->ARR = SSD_TIMER_ARR;       // Time constant
    SSD1_TIMER->EGR = 0;                   // Update event
    SSD1_TIMER->CNT = 0;                   // Reset counter
    SSD1_TIMER->SR = 0;                    // Clear pending IRQs
    SSD1_TIMER->CR1 = STM32_TIM_CR1_CEN;   // Enable Timer
    SSD1_TIMER->DIER = STM32_TIM_DIER_UIE; // Update Event IRQ enabled
}

/*
 * INITIALIZATION
 */

void softSerialInit(void)
{
    palSetLineMode(SSD1_RX_LINE, PAL_MODE_INPUT_PULLUP);
    palSetLineMode(SSD1_TX_LINE, PAL_MODE_OUTPUT_PUSHPULL);
    ssdObjectInit(&SSD1);
    ssdStart(&SSD1, &ssd1_config);

    timerStart();
}

void softSerialStop(void)
{
    timerStop();

    ssdStop(&SSD1);
}
```

### Usage

```c
chprintf((BaseSequentialStream *)&SSD1, "Software Serial Output!\n");
```
