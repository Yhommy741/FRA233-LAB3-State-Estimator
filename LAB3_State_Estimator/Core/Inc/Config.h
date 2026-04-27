/*
 * Config.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define ULTRASONIC_TRIG_PORT GPIOC
#define ULTRASONIC_TRIG_PIN GPIO_PIN_1
#define ULTRASONIC_TRIGGER_HTIM &htim6
#define ULTRASONIC_ECHO_HTIM &htim1
#define ULTRASONIC_TIM_CHANNEL_1 TIM_CHANNEL_1
#define ULTRASONIC_TIM_CHANNEL_2 TIM_CHANNEL_2

#define OBSERVER_HTIM &htim3

#endif /* INC_CONFIG_H_ */
