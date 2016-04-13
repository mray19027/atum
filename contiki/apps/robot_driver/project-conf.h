#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

// No need for UART
// #define STARTUP_CONF_VERBOSE 0
// #define UART_CONF_ENABLE 0

#ifndef UART0_CONF_WITH_INPUT
#define UART_CONF_WITH_INPUT 1
#endif

/** BUTTON_LEFT -> PC4 */
#define BUTTON_PORT        GPIO_C_NUM
#define BUTTON_PIN         0
#define BUTTON_VECTOR      NVIC_INT_GPIO_PORT_C

// /** BUTTON_DOWN -> PC7 */
// #define BUTTON_BACK_PORT        GPIO_C_NUM
// #define BUTTON_BACK_PIN         1
// #define BUTTON_BACK_VECTOR      NVIC_INT_GPIO_PORT_C

#endif /* PROJECT_CONF_H_ */

/** @} */
