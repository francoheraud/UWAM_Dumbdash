/*
 * can.c
 *
 *  Created on: Feb 20, 2025
 *      Author: poo brain
 *
 *
 *
 * 3am moment
 */

#include "can.h"

/*
 * @brief 	Abstracted CAN transmission function post init function calls. Supposed to aid with handling CAN errors if present.
 * @param 	Dumbdash_TypeDef
 * @return 	HAL status of CAN transaction
 * */
HAL_StatusTypeDef Dumbdash_Transmit_CAN_Msg(Dumbdash_TypeDef *dev, uint32_t id, uint32_t length) {
	dev->tx_header->StdId = id;
	dev->tx_header->DLC = length;
	dev->tx_header->IDE = CAN_ID_STD;
	dev->tx_header->RTR = CAN_RTR_DATA;
	dev->tx_header->TransmitGlobalTime = DISABLE;

	return HAL_CAN_AddTxMessage(dev->can_handle, dev->tx_header, dev->tx_data, &dev->tx_mailbox);
}



// must be able to recieve in order for dumbdash testing to work!!!!
// must be used in conjunction with the HAL_CAN_RxFifo0MsgPendingCallback function in main.c
HAL_StatusTypeDef Dumbdash_Receive_CAN_Msg(Dumbdash_TypeDef *dev, uint32_t id, uint32_t length) {
	dev->rx_header->StdId = id;
	dev->rx_header->DLC = length;
	dev->rx_header->IDE = CAN_ID_STD;
	dev->rx_header->RTR = CAN_RTR_DATA;

	//HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo,
    //CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])

	return HAL_CAN_GetRxMessage(dev->can_handle, CAN_RX_FIFO0, dev->rx_header, dev->rx_data);
}


/* DO NOT CHANGE */
//-----------------------------------
const uint16_t rot_enc_arr1[4] = {
		GPIO_PIN_12,
		GPIO_PIN_13,
		GPIO_PIN_14,
		GPIO_PIN_15
};

const uint16_t rot_enc_arr2[4] = {
		GPIO_PIN_4,
		GPIO_PIN_5,
		GPIO_PIN_6,
		GPIO_PIN_7
};
//-----------------------------------


/*
 * @brief Main algorithm for converting rotary encoder output values to an integer value.
 * @param output_arr A 2-byte array containing the configured GPIO pins (size 4).
 * @return The 4-bit rotary encoder position as an integer in binary.
 * */
uint8_t Dumbdash_Read_Rotary_Encoder_Outputs(const uint16_t *output_arr) {
	uint16_t result = 0x00;
	for (uint8_t i = 0; i < 4; i++) {
		result |= (HAL_GPIO_ReadPin(GPIOB, output_arr[i]) << i);
	}
	return ((uint8_t)result * (0xFF / 0x0F)); // positions are mapped from 0-15 to 0-255
}



/*
 * @brief Initialises CAN communication protocol
 * @param Dumbdash_TypeDef
 * @param CAN_HandleTypeDef
 * */
//CAN_FilterTypeDef can_filter_config;
void Dumbdash_CAN_HAL_Init(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *can_handle,
		CAN_TxHeaderTypeDef *tx_header, CAN_RxHeaderTypeDef *rx_header, CAN_FilterTypeDef *filter_header) {
	dev->can_handle = can_handle;
	dev->tx_header = tx_header;
	dev->rx_header = rx_header;
	dev->filter_header = filter_header;

	HAL_CAN_Start(dev->can_handle);
	HAL_CAN_ActivateNotification(dev->can_handle, CAN_IT_RX_FIFO0_MSG_PENDING);

	dev->filter_header->FilterActivation = 			ENABLE;
	dev->filter_header->FilterBank = 				INIT_FILTER_BANK; // 0
	dev->filter_header->FilterFIFOAssignment = 		CAN_RX_FIFO1;
	dev->filter_header->FilterIdHigh = 				(0x6C0 << 5);
	dev->filter_header->FilterIdLow = 				0x0000;
	dev->filter_header->FilterMaskIdHigh =			(0x7FF << 5);
	dev->filter_header->FilterMaskIdLow =			0x0000;
	dev->filter_header->FilterMode =				CAN_FILTERMODE_IDMASK;
	dev->filter_header->FilterScale = 				CAN_FILTERSCALE_32BIT;
	dev->filter_header->FilterFIFOAssignment = 		CAN_FILTER_FIFO0;
	dev->filter_header->SlaveStartFilterBank = 		SLAVE_FILTER_BANK;

	HAL_CAN_ConfigFilter(dev->can_handle, dev->filter_header);





}


// vv DEFINE IN MAIN.C
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//
//	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
//		Error_Handler();
//	}
	// if rx_header.StdId is something then so something else
//}




/*
 * @brief Init function which initialises time measurements for FSM
 * @param Dumbdash_TypeDef
 * @param GPIO_TypeDef
 * @param CAN_Handle_TypeDef
 *  */
void Dumbdash_Init(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef *tx_header, CAN_RxHeaderTypeDef *rx_header, CAN_FilterTypeDef *filter_header) {
	Dumbdash_CAN_HAL_Init(dev, can_handle, tx_header, rx_header, filter_header);
	//dev->uart_handle = uart_handle;
	dev->tick_t.current_time = 0;
	dev->tick_t.elapsed_time = 0;
	dev->prev_enc_pos_1 = 0;
	dev->prev_enc_pos_2 = 0;
}



/*
 * @brief 	Intended to work with the weak definition of HAL_GPIO_EXTI_Callback to transmit encoder output values through the CAN bus
 * @param 	Dumbdash_TypeDef
 * @param 	condition_1 pin and state condition for pedal map encoder
 * @param 	condition_2 etc...
 * @return 	HAL status enum for future error handling
 * */
HAL_StatusTypeDef Dumbdash_Transmit_Encoder_Data(Dumbdash_TypeDef *dev, volatile uint8_t encoder_1_ready, volatile uint8_t encoder_2_ready) {
	volatile uint8_t enc_pos_1 = Dumbdash_Read_Rotary_Encoder_Outputs(rot_enc_arr1); // pedal map select
	volatile uint8_t enc_pos_2 = Dumbdash_Read_Rotary_Encoder_Outputs(rot_enc_arr2); // traction control gain
	HAL_StatusTypeDef status = HAL_ERROR; // common state

	if (encoder_1_ready && (enc_pos_1 != dev->prev_enc_pos_1)) {
		dev->prev_enc_pos_1 = enc_pos_1;
		dev->tx_data[0] = enc_pos_1;
		status = Dumbdash_Transmit_CAN_Msg(dev, CAN_ID_ROTARY_ENCODER, 2);
	}
	if (encoder_2_ready && (enc_pos_2 != dev->prev_enc_pos_2)) {
		dev->prev_enc_pos_2 = enc_pos_2;
		dev->tx_data[1] = enc_pos_2;
		status = Dumbdash_Transmit_CAN_Msg(dev, CAN_ID_ROTARY_ENCODER, 2);
	}
	return status;
}


// more refined version of above function, only using polling
HAL_StatusTypeDef Dumbdash_Transmit_Encoder_Data_v2(Dumbdash_TypeDef *dev) {
	dev->prev_enc_pos_1 = Dumbdash_Read_Rotary_Encoder_Outputs(rot_enc_arr1);
	dev->prev_enc_pos_2 = Dumbdash_Read_Rotary_Encoder_Outputs(rot_enc_arr2);
	dev->tx_data[0] = dev->prev_enc_pos_1;
	dev->tx_data[1] = dev->prev_enc_pos_2;
	return Dumbdash_Transmit_CAN_Msg(dev, CAN_ID_ROTARY_ENCODER, 2);
}


/*
 * @brief 	Uses FSM control statements to implement a debounce timer on the 'lcb' (Launch Control Button) due the absence of a RC low pass filter
 * The debounce timer is an optional feature and can be configured using 'DEBOUNCE_TIME_MS' define
 * @param 	Dumbdash_TypeDef	...
 * @param 	data_ready_flag		Toggles to '1' upon external interrupt, used to trigger the function
 * @return	HAL status enum to prevent undefined behaviour
 * */
HAL_StatusTypeDef Dumbdash_Launch_Control_Interrupt_FSM(Dumbdash_TypeDef *dev, volatile uint8_t data_ready_flag) {

	dev->tick_t.current_time = HAL_GetTick();
	HAL_StatusTypeDef can_status = HAL_ERROR; // known state for debugging

	switch(dev->current_state) {

	case IDLE_lcb:

		if (data_ready_flag) {
			dev->tick_t.elapsed_time = dev->tick_t.current_time;
			dev->current_state = DEBOUNCE_TIMER_lcb;
		}
		break;

	case DEBOUNCE_TIMER_lcb:

		if ((dev->tick_t.current_time - dev->tick_t.elapsed_time) == DEBOUNCE_TIME_MS) {
			dev->current_state = TRANSMIT_CAN_MSG_lcb;
		}
		break;

	case TRANSMIT_CAN_MSG_lcb:

		memset(dev->tx_data, 0x00, sizeof(dev->tx_data));

		dev->tx_data[0] = CAN_DATA_FIELD_LAUNCH_CONTROL_ON;
		can_status = Dumbdash_Transmit_CAN_Msg(dev, CAN_ID_LAUNCH_CONTROL, 1);

		dev->tx_data[0] = CAN_DATA_FIELD_LC_LED_STATUS_ON;
		can_status = Dumbdash_Transmit_CAN_Msg(dev, CAN_ID_BUTTON_LED_STATUS, 1);

		dev->current_state = IDLE_lcb;
		break;
	}
	return can_status;
}





/*
 * @brief Initialises UART for easier future debugging and assigns the uart handle ptr to the DD typedef struct member
 * @param Dumbdash_TypeDef
 * @param UART_HandleTypeDef
 * */
/*
void Dumbdash_UART_TestFunc_Init(Dumbdash_TypeDef *dev, UART_HandleTypeDef *uart_handle) {
	dev->uart_handle = uart_handle;
}
*/

/*
 * @brief Sends encoder positions via the UART communication protocol
 * @param Dumbdash_TypeDef
 * */
/*
void Dumbdash_Encoder_UART_TestFunc(Dumbdash_TypeDef *dev, HAL_StatusTypeDef status, UART_HandleTypeDef *uart_handle, uint8_t encoder_pos) {
	char buf[50];
	const char *str;

	// transmit uart pos for debugging/testing
	if (status == HAL_OK) {
		str = "Position: %d \r\n";
		//str = "set this to whatever you want"
		snprintf((char*)buf, 50, str, encoder_pos);
		HAL_UART_Transmit(dev->uart_handle, (const uint8_t*)buf, 50, HAL_MAX_DELAY);
	} else {
		snprintf((char*)buf, 50, "ERROR: Encoder position couldnt be read!\r\n");
		HAL_UART_Transmit(dev->uart_handle, (const uint8_t*)buf, 50, HAL_MAX_DELAY);
	}

}
*/


