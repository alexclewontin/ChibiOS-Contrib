/*
    Copyright (C) 2019 /u/KeepItUnder
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
 * @file    TIMv1/hal_pwm_lld.h
 * @brief   PWM subsystem low level driver header.
 *
 * @addtogroup PWM
 * @{
 */

#ifndef HAL_PWM_LLD_H
#define HAL_PWM_LLD_H

#if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Number of PWM channels per PWM driver.
 */
#define PWM_CHANNELS                            (1)

#define PWM_CH0                                 0x0ul
#define PWM_CH1                                 0x1ul
#define PWM_CH2                                 0x2ul
#define PWM_CH3                                 0x3ul

#define PWM_EDGE_ALIGNED                        (0x0ul)                     /*!< Edge aligned */
#define PWM_CENTER_ALIGNED                      (0x01ul)                    /*!< Center aligned */

#define PWM_CLK_DIV_1                           (0x04ul)                    /*!< Divide by 1 */
#define PWM_CLK_DIV_2                           (0x0ul)                     /*!< Divide by 2 */
#define PWM_CLK_DIV_4                           (0x01ul)                    /*!< Divide by 4 */
#define PWM_CLK_DIV_8                           (0x02ul)                    /*!< Divide by 8 */
#define PWM_CLK_DIV_16                          (0x03ul)                    /*!< Divide by 16 */

#define PWM_PERIOD_INT_UNDERFLOW                (0)                         /*!< Period interrupt - counter underflow */
#define PWM_PERIOD_INT_MATCH_CNR                (PWM_PIER_INT01TYPE_Msk)    /*!< Period interrupt - counter matches CNR */
#define PWM_CAPTURE_INT_RISING_LATCH            (PWM_CCR0_CRL_IE0_Msk)      /*!< Capture interrupt - rising latch */
#define PWM_CAPTURE_INT_FALLING_LATCH           (PWM_CCR0_CFL_IE0_Msk)      /*!< Capture interrupt - falling latch */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    NUC123 configuration options
 * @{
 */
/**
 * @brief   PWMD1 driver enable switch.
 * @details If set to @p TRUE the support for PWM1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(NUC123_PWM_USE_PWM1) || defined(__DOXYGEN__)
#define NUC123_PWM_USE_PWM1 TRUE
#endif

/**
 * @brief   PWMD1 driver IRQ priority.
 */
#if !defined(NUC123_PWM_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define NUC123_PWM_IRQ_PRIORITY 3
#endif
/** @} */

/*===========================================================================*/
/* Configuration checks.                                                     */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of a PWM mode.
 */
typedef uint32_t pwmmode_t;

/**
 * @brief   Type of a PWM channel.
 */
typedef uint8_t pwmchannel_t;

/**
 * @brief   Type of a channels mask.
 */
typedef uint32_t pwmchnmsk_t;

/**
 * @brief   Type of a PWM counter.
 */
typedef uint32_t pwmcnt_t;

/**
 * @brief   Type of a PWM driver channel configuration structure.
 */
typedef struct {
  /**
   * @brief Channel active logic level.
   */
  pwmmode_t mode;
  /**
   * @brief Channel callback pointer.
   * @note  This callback is invoked on the channel compare event. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t callback;
  /* End of the mandatory fields.*/
} PWMChannelConfig;

/**
 * @brief   Type of a PWM driver configuration structure.
 */
typedef struct {
  /**
   * @brief   Timer clock in Hz.
   * @note    The low level can use assertions in order to catch invalid
   *          frequency specifications.
   */
  uint32_t frequency;
  /**
   * @brief   PWM period in ticks.
   * @note    The low level can use assertions in order to catch invalid
   *          period specifications.
   */
  pwmcnt_t period;
  /**
   * @brief Periodic callback pointer.
   * @note  This callback is invoked on PWM counter reset. If set to
   *        @p NULL then the callback is disabled.
   */
  pwmcallback_t callback;
  /**
   * @brief Channels configurations.
   */
  PWMChannelConfig channels[PWM_CHANNELS];
  /* End of the mandatory fields.*/
} PWMConfig;

/**
 * @brief   Structure representing a PWM driver.
 */
struct PWMDriver {
  /**
   * @brief Driver state.
   */
  pwmstate_t state;
  /**
   * @brief Current driver configuration data.
   */
  const PWMConfig *config;
  /**
   * @brief   Current PWM period in ticks.
   */
  pwmcnt_t period;
  /**
   * @brief   Mask of the enabled channels.
   */
  pwmchnmsk_t enabled;
  /**
   * @brief   Number of channels in this instance.
   */
  pwmchannel_t channels;
#if defined(PWM_DRIVER_EXT_FIELDS)
  PWM_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/

  pwmchannel_t periodic_channel;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Changes the period the PWM peripheral.
 * @details This function changes the period of a PWM unit that has already
 *          been activated using @p pwmStart().
 * @pre     The PWM unit must have been activated using @p pwmStart().
 * @post    The PWM unit period is changed to the new value.
 * @note    The function has effect at the next cycle start.
 * @note    If a period is specified that is shorter than the pulse width
 *          programmed in one of the channels then the behavior is not
 *          guaranteed.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] period    new cycle time in ticks
 *
 * @notapi
 */
#define pwm_lld_change_period(pwmp, period) _pwm_lld_change_period(pwmp, period)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (NUC123_PWM_USE_PWM1 == TRUE) && !defined(__DOXYGEN__)
extern PWMDriver PWMD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void pwm_lld_init(void);
  void pwm_lld_start(PWMDriver *pwmp);
  void pwm_lld_stop(PWMDriver *pwmp);
  void pwm_lld_enable_channel(PWMDriver *pwmp,
                              pwmchannel_t channel,
                              pwmcnt_t width);
  void pwm_lld_disable_channel(PWMDriver *pwmp, pwmchannel_t channel);
  void pwm_lld_enable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_disable_periodic_notification(PWMDriver *pwmp);
  void pwm_lld_enable_channel_notification(PWMDriver *pwmp,
                                           pwmchannel_t channel);
  void pwm_lld_disable_channel_notification(PWMDriver *pwmp,
                                            pwmchannel_t channel);
  void _pwm_lld_change_period(PWMDriver *pwmp, pwmcnt_t period);

#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_PWM */

#endif /* HAL_PWM_LLD_H */

/** @} */
