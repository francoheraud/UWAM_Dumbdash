/*
 * can.h
 *
 *  Created on: Feb 20, 2025
 *      Author: fhn
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_can.h"
#include "stm32f3xx_hal_gpio.h"
//#include "stm32f3xx_hal_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CAN_ID_LAUNCH_CONTROL				((uint32_t)	0x6C0	)
#define CAN_ID_ROTARY_ENCODER				((uint32_t)	0x6C0	)
#define CAN_ID_BUTTON_LED_STATUS			((uint32_t)	0x6C2	)

#define CAN_DATA_FIELD_LAUNCH_CONTROL_ON	((uint8_t)	0x000	) // add datafield byte
#define CAN_DATA_FIELD_LC_LED_STATUS_ON		((uint8_t)	0x000	) // add datafield byte

#define INIT_FILTER_BANK					((uint32_t)	0x000	)
#define SLAVE_FILTER_BANK					((uint32_t)	0x000	)

#define	DEBOUNCE_TIME_MS					((uint32_t)	0		)


typedef enum {
	IDLE_lcb,
	DEBOUNCE_TIMER_lcb,
	TRANSMIT_CAN_MSG_lcb
} FSM_DebounceStates;


typedef struct {
	uint32_t current_time;
	uint32_t elapsed_time;
} TickManager_TypeDef;


typedef struct {

	//CAN_InitTypeDef *can_init;
	//UART_HandleTypeDef *uart_handle;
	CAN_HandleTypeDef *can_handle;
	CAN_TxHeaderTypeDef *tx_header;
	CAN_RxHeaderTypeDef *rx_header;
	CAN_FilterTypeDef *filter_header;

	TickManager_TypeDef tick_t;
	FSM_DebounceStates current_state;

	uint8_t tx_data[8];
	uint8_t rx_data[8];
	uint32_t tx_mailbox;
	uint32_t rx_mailbox;

	uint8_t prev_enc_pos_1;
	uint8_t prev_enc_pos_2;


} Dumbdash_TypeDef;



HAL_StatusTypeDef Dumbdash_Transmit_CAN_Msg(Dumbdash_TypeDef *dev, uint32_t id, uint32_t length);
uint8_t Dumbdash_Read_Rotary_Encoder_Outputs(const uint16_t *output_arr);
void Dumbdash_CAN_HAL_Init(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef *tx_header, CAN_RxHeaderTypeDef *rx_header, CAN_FilterTypeDef *filter_header);
void Dumbdash_Init(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef *tx_header, CAN_RxHeaderTypeDef *rx_header, CAN_FilterTypeDef *filter_header);
HAL_StatusTypeDef Dumbdash_Launch_Control_Interrupt_FSM(Dumbdash_TypeDef *dev, volatile uint8_t data_ready_flag);
//void Dumbdash_UART_TestFunc_Init(Dumbdash_TypeDef *dev, UART_HandleTypeDef *uart_handle);
//void Dumbdash_Encoder_UART_TestFunc(Dumbdash_TypeDef *dev, HAL_StatusTypeDef status, UART_HandleTypeDef *uart_handle, uint8_t encoder_pos);
HAL_StatusTypeDef Dumbdash_Transmit_Encoder_Data_v2(Dumbdash_TypeDef *dev);
HAL_StatusTypeDef Dumbdash_Receive_CAN_Msg(Dumbdash_TypeDef *dev, uint32_t id, uint32_t length);


#endif /* INC_CAN_H_ */



