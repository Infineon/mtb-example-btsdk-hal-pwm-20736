/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */


/** @file
*
* PWM Tones Sample
*
* This application provides the sample code that uses the
* on-chip PWM. No buzzer on the board to render
* generated PWM tones, instead the LED2 is used for output
* when the button connected to P4 is pressed.
*
* App generate random width pulses using HW based random number generator.
*
* Features demonstrated
*  - Use of the PWM driver interface.
*  - Using PWM2 on P28.
*  - Handling SW1 generated interrupts.
*  - Use of the on-chip HW random number generator.
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Once the application is running, push SW1 and every
* 	time the button is pressed, the PWM generated
*   is used to illuminate LED2 with different levels of brightness.
* 4. Attach an oscilloscope to P28 to observe signal widths in detail.
*
*/

#include "spar_utils.h"
#include "bleprofile.h"
#include "bleapp.h"
#include "blecen.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "bleappconfig.h"
#include "sparcommon.h"
#include "cfa.h"
#include "aclk.h" // For auxiliary clock and the PWM driver.
#include "pwm.h"

/******************************************************
 *                      Constants
 ******************************************************/
#define TONES_LED2_GPIO       (28)

/******************************************************
 *               Types
 ******************************************************/
enum
{
	TONES_STATE_0,
	TONES_STATE_1,
	TONES_STATE_2,
	TONES_STATE_3,
	TONES_STATE_4,
	TONES_STATE_MAX
};

/******************************************************
 *               Function Prototypes
 ******************************************************/
static void tones_create(void);
static void tones_fine_timeout(UINT32 arg);
static void tones_button_interrupt_handler(UINT8 value);
static void tones_go_to_next_state(void);
static void tones_blink_led2(void);

/******************************************************
 *               Variables Definitions
 ******************************************************/
UINT8 tones_current_state = TONES_STATE_0;
UINT8 button_value = 0;

// assume init_value is 0 so max toggle count is 0x3ff
UINT32 toggle_values[TONES_STATE_MAX] = { 0, 50, 100, 400, 1000};

// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG tones_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/  PUARTENABLE | GPIO_PIN_UART_TX,
    /*.rxpin      =*/  PUARTENABLE | GPIO_PIN_UART_RX,
};

// Following structure defines GPIO configuration used by the application
const BLE_PROFILE_GPIO_CFG tones_gpio_cfg =
{
    /*.gpio_pin =*/
    {
        GPIO_PIN_WP,      // This need to be used to enable/disable NVRAM write protect
        GPIO_PIN_BUTTON,  // Button GPIO is configured to trigger either direction of interrupt
        GPIO_PIN_LED,     // LED GPIO, optional to provide visual effects
        GPIO_PIN_BATTERY, // Battery monitoring GPIO. When it is lower than particular level, it will give notification to the application
        TONES_LED2_GPIO,  // PWM GPIO, optional to provide audio effects
        -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 // other GPIOs are not used
    },
    /*.gpio_flag =*/
    {
        GPIO_SETTINGS_WP,
        GPIO_SETTINGS_BUTTON, // otherwise jumper to 3v3 from GPIO_PIN_BUTTON is needed.
        GPIO_SETTINGS_LED,
        GPIO_SETTINGS_BATTERY,
        GPIO_LED | GPIO_OUTPUT | GPIO_INIT_LOW,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    }
};

/******************************************************
 *               Function Definitions
 ******************************************************/

// Application initialization
APPLICATION_INIT()
{
    bleapp_set_cfg(NULL,
                   0,
                   NULL,
                   (void *)&tones_puart_cfg,
                   (void *)&tones_gpio_cfg,
                   tones_create);

    // BLE_APP_DISABLE_TRACING();     // Uncomment to disable all tracing
    BLE_APP_ENABLE_TRACING_ON_PUART();
}

// Create temperature sensor
void tones_create(void)
{

    ble_trace0("tones_create()\n");
    blecen_Create();
    bleprofile_GPIOInit(bleprofile_gpio_p_cfg);

    // register interrupt handler for button inputs.
    bleprofile_regIntCb((BLEPROFILE_SINGLE_PARAM_CB) tones_button_interrupt_handler);

    // Configure auxiliary clock 1 (needed for PWM when using PMU_CLK; not needed
    // when reference is LHL_CLK) and use the 24MHz system
    // clock as the reference. Since we will generating up to 8KHz in this
    // sample app, feeding a 512K signal to the PWM as the reference should give
    // us sufficient accuracy. For higher accuracy, use a higher reference
    // frequency. Typically, for LEDs, we use lower PWM frequency and lower
    // reference frequencies.
    aclk_configure(512000, ACLK1, ACLK_FREQ_24_MHZ);

#if defined(CYW920736M2EVB_01)
    // tristate the pads sharing same pins as P26 and P28
    gpio_configurePin(0, 12, GPIO_OUTPUT_DISABLE, 0);
    gpio_configurePin(0, 13, GPIO_OUTPUT_DISABLE, 0);
#endif
    tones_blink_led2();
}

// Fine timer callback.
void tones_fine_timeout(UINT32 arg)
{
  //  ble_trace0("tones_fine_timeout()\n");
    bleapptimer_stopFineTimer();
    tones_go_to_next_state();
    tones_blink_led2();
}

// Three Interrupt inputs (Buttons) can be handled here.
// If the following value == 1, Button is pressed. Different than initial value.
// If the following value == 0, Button is depressed. Same as initial value.
// Button1 : value&0x01
// Button2 : (value&0x02)>>1
// Button3 : (value&0x04)>>2
void tones_button_interrupt_handler(UINT8 value)
{
    //ble_trace1("tones_interrupt_handler() value = %x\n", value);

    // On press.
    if (value & 0x01)
    {
        bleapptimer_stopFineTimer();
        button_value = gpio_getPinInput((TONES_LED2_GPIO) / 16, (TONES_LED2_GPIO) % 16);
        // start debounce timer
        // needs a bit of delay or pwm does not update properly
        bleapptimer_startFineTimer(tones_fine_timeout, 8); // time tick is 12.5 ms or 80 calls/sec
    }
}

// Sets the state machine to the next state.
void tones_go_to_next_state(void)
{
    ble_trace1("previous state %d\n", tones_current_state );
    tones_current_state++;

    if (tones_current_state > TONES_STATE_4)
        tones_current_state = TONES_STATE_0;
}

// Blinks LEDs using some random values
void tones_blink_led2(void)
{
    pwm_start(PWM2, LHL_CLK, toggle_values[tones_current_state], 0);

    // Get init and toggle counts for LED and trace it.
    // Everytime the button is pressed the state will be changed.
    // The init state is TONES_STATE_0 and the toggle count is 0 (toggle_values[0]).
    // First time button pressed : state changed to TONES_STATE_1, the toggle count is 50 (toggle_values[1]).
    // Second time button pressed: state changed to TONES_STATE_2, the toggle count is 100 (toggle_values[2]);
    // Third time button pressed : state changed to TONES_STATE_3, the toggle count is 400 (toggle_values[3]);
    // Fourth time button pressed: state changed to TONES_STATE_4, the toggle count is 1000 (toggle_values[4]);
    // Fifth time button pressed : state changed to TONES_STATE_0, the toggle count is 0, (toggle_values[0]);

    ble_trace3("PWM init count: 0x%03X, Toggle Count: 0x%03X, current state: %d\n", pwm_getInitValue(PWM2), pwm_getToggleCount(PWM2), tones_current_state);
}
