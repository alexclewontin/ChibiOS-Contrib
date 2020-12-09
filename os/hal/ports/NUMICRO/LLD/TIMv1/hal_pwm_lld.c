/*
    Copyright (C) 2020 Alex Lewontin

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
 * @file    TIMv1/hal_pwm_lld.c
 * @brief   PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#include "hal.h"

#if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define NUC123_PWM_CLKSRC_HSE  0x0UL
#define NUC123_PWM_CLKSRC_HCLK 0x2UL
#define NUC123_PWM_CLKSRC_HSI  0x3UL
#define NUC123_PWM_CLKSRC_LSI  0x7UL

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PWMD1 driver identifier.
 */
#if (NUC123_PWM_USE_PWM1 == TRUE) || defined(__DOXYGEN__)
PWMDriver PWMD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

typedef struct {
    __IO uint32_t CNR;          /* Offset: 0x0C  PWM Counter Register 0                                             */
    __IO uint32_t CMR;          /* Offset: 0x10  PWM Comparator Register 0                                          */
    __I  uint32_t PDR;          /* Offset: 0x14  PWM Data Register 0                                                */
} PWM_CHANNEL_T;

#define PWMA_CHANNELS_BASE        (PWMA_BASE + 0xC)
#define PWMA_CHANNELS             ((PWM_CHANNEL_T *) PWMA_CHANNELS_BASE)


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if 0
static void pwm_set_clk_freq(PWMDriver *pwmp) {

  /* Try to find exact */
  uint32_t target_freq = pwmp->config->frequency;
  uint32_t max_input_freq = target_freq * 256 * 16;
  uint32_t min_input_freq = target_freq * 2;

  //target_freq = (input/256)/16
  //target_freq = (input/2)

  //[input/4096, input/2]
  // input/4096 <= target -> input <= target*4096
  // input/2 >= target -> input >= target * 2
  // target * 2 =< input <= target * 4096

  // Can be LSI, HSI, HCLK, or HSE
#if NUC123_LSI_ENABLED
  if ((min_input_freq <= NUC123_LSICLK) && (NUC123_LSICLK <= max_input_freq)) {
    if (NUC123_LSICLK % target_freq == 0) {
      uint32_t denom = NUC123_LSICLK / target_freq;
    }
  }
#endif

#if NUC123_HSE_ENABLED
  if ((min_input_freq <= NUC123_HSECLK) && (NUC123_HSECLK <= max_input_freq)) {
    if (NUC123_HSECLK % target_freq == 0) {
      uint32_t denom = NUC123_HSECLK / target_freq;
      if (denom % 2) {

      } else {
        
      }
    }
  }
#endif

#if NUC123_HSI_ENABLED
  if ((min_input_freq <= NUC123_HSICLK) && (NUC123_HSICLK <= max_input_freq)) {
    if (NUC123_HSICLK % target_freq == 0) {
      uint32_t denom = NUC123_HSICLK / target_freq;
    }
  }
#endif

  osalDbgAssert((min_input_freq <= NUC123_HCLK) &&
                    (NUC123_HCLK <= max_input_freq),
                "Cannot generate requisite frequency");

}

#endif

/**
 * @brief   Shared IRQ handler.
 *
 * @param[in] pwmd     pointer to a @p PWMDriver object
 *
 * @notapi
 */
static void pwm_lld_serve_interrupt(PWMDriver *pwmp) {

  uint32_t piir;
  uint32_t pier;
  uint32_t poe;

  poe  = PWMA->POE;
  pier = PWMA->PIER;
  piir = PWMA->PIIR;
  PWMA->PIIR = piir;

  if ((piir & PWM_PIIR_PWMDIF0_Msk) && pwmp->config->channels[0].callback) {
    pwmp->config->channels[0].callback(pwmp);
  }

  if ((piir & PWM_PIIR_PWMDIF1_Msk) && pwmp->config->channels[1].callback) {
    pwmp->config->channels[1].callback(pwmp);
  }

  if ((piir & PWM_PIIR_PWMDIF2_Msk) && pwmp->config->channels[2].callback) {
    pwmp->config->channels[2].callback(pwmp);
  }

  if ((piir & PWM_PIIR_PWMDIF3_Msk) && pwmp->config->channels[3].callback) {
    pwmp->config->channels[3].callback(pwmp);
  }

    /* TODO: Periodic callback */

  if ((piir & PWM_PIIR_PWMIF0_Msk) & !pwmIsChannelEnabledI(pwmp, 0)) {
    poe &= ~PWM_POE_PWM1_Msk;
    pier &= PWM_PIER_PWMIE0_Msk;
    pier &= PWM_PIER_PWMDIE0_Msk;
  }

  if ((piir & PWM_PIIR_PWMIF1_Msk) & !pwmIsChannelEnabledI(pwmp, 1)) {
    poe &= ~PWM_POE_PWM1_Msk;
    pier &= PWM_PIER_PWMIE1_Msk;
    pier &= PWM_PIER_PWMDIE1_Msk;
  }

  if ((piir & PWM_PIIR_PWMIF2_Msk) & !pwmIsChannelEnabledI(pwmp, 2)) {
    poe &= ~PWM_POE_PWM2_Msk;
    pier &= PWM_PIER_PWMIE2_Msk;
    pier &= PWM_PIER_PWMDIE2_Msk;
  }

  if ((piir & PWM_PIIR_PWMIF3_Msk) & !pwmIsChannelEnabledI(pwmp, 3)) {
    poe &= ~PWM_POE_PWM3_Msk;
    pier &= PWM_PIER_PWMIE3_Msk;
    pier &= PWM_PIER_PWMDIE3_Msk;
  }

  PWMA->POE = poe;
  PWMA->PIER = pier;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   PWM interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(NUC123_PWMA_HANDLER) 
{
  OSAL_IRQ_PROLOGUE();
  pwm_lld_serve_interrupt(&PWMD1);
  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level PWM driver initialization.
 *
 * @notapi
 */
void pwm_lld_init(void)
{

#if (NUC123_PWM_USE_PWM1 == TRUE)
  /* Driver initialization.*/
  pwmObjectInit(&PWMD1);
#endif
}

/**
 * @brief   Configures and activates the PWM peripheral.
 * @note    Starting a driver that is already in the @p PWM_READY state
 *          disables all the active channels.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_start(PWMDriver *pwmp)
{
  if (pwmp->state == PWM_STOP) {
    /* Clock activation and timer reset.*/
#if (NUC123_PWM_USE_PWM1 == TRUE)
    if (&PWMD1 == pwmp) {

      CLK->CLKSEL1 = 
          (CLK->CLKSEL1 & ~(CLK_CLKSEL1_PWM01_S_Msk)) |
          ((NUC123_PWM_CLKSRC_HCLK << CLK_CLKSEL1_PWM01_S_Pos) & CLK_CLKSEL1_PWM01_S_Msk);
      CLK->CLKSEL1 = 
          (CLK->CLKSEL1 & ~(CLK_CLKSEL1_PWM23_S_Msk)) |
          ((NUC123_PWM_CLKSRC_HCLK << CLK_CLKSEL1_PWM23_S_Pos) & CLK_CLKSEL1_PWM23_S_Msk);
      CLK->CLKSEL2 =
          (CLK->CLKSEL2 & ~(CLK_CLKSEL2_PWM01_S_E_Msk)) |
          ((NUC123_PWM_CLKSRC_HCLK >> 2) << CLK_CLKSEL2_PWM01_S_E_Pos);
      CLK->CLKSEL2 =
          (CLK->CLKSEL2 & ~(CLK_CLKSEL2_PWM23_S_E_Msk)) |
          ((NUC123_PWM_CLKSRC_HCLK >> 2) << CLK_CLKSEL2_PWM23_S_E_Pos);


      CLK->APBCLK |= (CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_PWM23_EN_Msk);
      nvicEnableVector(NUC123_PWMA_NUMBER, NUC123_PWM_IRQ_PRIORITY);
      SYS->IPRSTC2 |= SYS_IPRSTC2_PWM03_RST_Msk;
      SYS->IPRSTC2 &= ~(SYS_IPRSTC2_PWM03_RST_Msk);

      /* Set clock scaling to 1 */
      PWMA->CSR = (8 << PWM_CSR_CSR0_Pos) | (8 << PWM_CSR_CSR1_Pos) |
                  (8 << PWM_CSR_CSR2_Pos) | (8 << PWM_CSR_CSR3_Pos);

      /* Set prescale to set frequency */
      uint32_t prescale = NUC123_HCLK / pwmp->config->frequency;
      PWMA->PPR = (PWMA->PPR & ~(PWM_PPR_CP01_Msk | PWM_PPR_CP23_Msk)) |
                  ((prescale << PWM_PPR_CP01_Pos) & PWM_PPR_CP01_Msk) |
                  ((prescale << PWM_PPR_CP23_Pos) & PWM_PPR_CP23_Msk);


      /* TODO: Polarity inversion */
      /* Notes: PINV == 0 -> active high, PINV == 1 -> active low. */
      /* What does INV do?  (Check Figure 6-34 for more) */

      /* TODO: Deadzone generation */
      /* TODO: auto-reload/one-shot mode and stop  PWM-timer from PWM control register (PCR) */

      /* TODO: Set GPIO Pins to PWM function */

      PWMA->PIER &= ~(PWM_PIER_INT01TYPE_Msk | PWM_PIER_INT23TYPE_Msk);

      /*
4. Set comparator register (CMR) for setting PWM duty.
*/
      PWMA_CHANNELS[0].CMR = 0;
      PWMA_CHANNELS[1].CMR = 0;
      PWMA_CHANNELS[2].CMR = 0;
      PWMA_CHANNELS[3].CMR = 0;
      /*
5. Set PWM down-counter register (CNR) for setting PWM period.
*/
      PWMA_CHANNELS[0].CNR = pwmp->config->period - 1;
      PWMA_CHANNELS[1].CNR = pwmp->config->period - 1;
      PWMA_CHANNELS[2].CNR = pwmp->config->period - 1;
      PWMA_CHANNELS[3].CNR = pwmp->config->period - 1;

      PWMA->PCR |= (PWM_PCR_CH0EN_Msk | PWM_PCR_CH1EN_Msk | PWM_PCR_CH2EN_Msk | PWM_PCR_CH3EN_Msk);

      return;
    }
#endif
  }
}

/**
 * @brief   Deactivates the PWM peripheral.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_stop(PWMDriver *pwmp)
{
  /* If in ready state then disables the PWM clock.*/
  if (pwmp->state == PWM_READY) {
#if NUC123_PWM_USE_PWM1 == TRUE
    if (&PWMD1 == pwmp) {
      CLK->APBCLK &= ~(CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_PWM23_EN_Msk);
      nvicDisableVector(NUC123_PWMA_NUMBER);
      return;
    }
#endif
  }
}

/**
 * @brief   Enables a PWM channel.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is active using the specified configuration.
 * @note    The function has effect at the next cycle start.
 * @note    Channel notification is not enabled.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 * @param[in] width     PWM pulse width as clock pulses number
 *
 * @notapi
 */
void pwm_lld_enable_channel(PWMDriver *pwmp, pwmchannel_t channel,
                            pwmcnt_t width)
{
  /*
4. Set comparator register (CMR) for setting PWM duty.
*/
  PWMA_CHANNELS[channel].CMR = width - 1;
  /*
5. Set PWM down-counter register (CNR) for setting PWM period.
*/
  PWMA_CHANNELS[channel].CNR = pwmp->config->period - 1;

  PWMA->PCR |= (PWM_PCR_CH0EN_Msk << (channel * 8));

  PWMA->POE |= PWM_POE_PWM0_Msk << channel;
}

/**
 * @brief   Disables a PWM channel and its notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The channel is disabled and its output line returned to the
 *          idle state.
 * @note    The function has effect at the next cycle start.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel)
{
  /* We do not disable immediately, but rather set the reload value to 0,
    and the periodic interrupt. We will actually disable in the next ISR. */
  PWMA_CHANNELS[channel].CNR = 0;
  PWMA->PIER |= (PWM_PIER_PWMIE0_Msk << channel);
}

/**
 * @brief   Enables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_enable_periodic_notification(PWMDriver *pwmp) {
  /* TODO: periodic notifications */
  /* PWMA->PIER |= (PWM_PIER_PWMIE0_Msk << pwmp->periodic_channel); */
}

/**
 * @brief   Disables the periodic activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 *
 * @notapi
 */
void pwm_lld_disable_periodic_notification(PWMDriver *pwmp) {
  /* TODO: periodic notifications */
  /*PWMA->PIER &= ~(PWM_PIER_PWMIE0_Msk << pwmp->periodic_channel); */
}

/**
 * @brief   Enables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already enabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                         pwmchannel_t channel)
{
  (void)pwmp;
  PWMA->PIER |= (PWM_PIER_PWMDIE0_Msk << channel);
}

/**
 * @brief   Disables a channel de-activation edge notification.
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @pre     The channel must have been activated using @p pwmEnableChannel().
 * @note    If the notification is already disabled then the call has no effect.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] channel   PWM channel identifier (0...channels-1)
 *
 * @notapi
 */
void pwm_lld_disable_channel_notification(PWMDriver *  pwmp,
                                          pwmchannel_t channel)
{
  (void)pwmp;
  PWMA->PIER &= ~(PWM_PIER_PWMDIE0_Msk << channel);
}

void _pwm_lld_change_period(PWMDriver *pwmp, pwmcnt_t period) {
  PWMA_CHANNELS[0].CNR = pwmp->config->period - 1;
  PWMA_CHANNELS[1].CNR = pwmp->config->period - 1;
  PWMA_CHANNELS[2].CNR = pwmp->config->period - 1;
  PWMA_CHANNELS[3].CNR = pwmp->config->period - 1;
}

#endif /* HAL_USE_PWM */

/** @} */
