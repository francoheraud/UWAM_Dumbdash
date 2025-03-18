/*
 * can.c
 *
 *  Created on: Feb 20, 2025
 *      Author: Adam Berta (Electrical Lead)
 *
 *
 *
 *
 */

#include "can.h"


/*
 * @brief Initialises CAN communication protocol and configures the CAN filter
 * @param Dumbdash_TypeDef
 * @param CAN_HandleTypeDef
 * */

HAL_StatusTypeDef Dumbdash_CAN_HAL_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *filter_header) {

	__HAL_RCC_CAN1_CLK_ENABLE();
	hcan->Instance = CAN;
	hcan->Init.Prescaler = 6; // Adjust for desired baud rate
	hcan->Init.Mode = CAN_MODE_LOOPBACK; // Loopback mode
	hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan->Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan->Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan->Init.TimeTriggeredMode = DISABLE;
	hcan->Init.AutoBusOff = DISABLE;
	hcan->Init.AutoWakeUp = DISABLE;
	hcan->Init.AutoRetransmission = DISABLE;
	hcan->Init.ReceiveFifoLocked = DISABLE;
	hcan->Init.TransmitFifoPriority = DISABLE;


	if (HAL_CAN_Init(hcan) != HAL_OK) {
		return HAL_ERROR;
	}

	filter_header->FilterBank = 0;
	filter_header->FilterMode = CAN_FILTERMODE_IDMASK;
	filter_header->FilterScale = CAN_FILTERSCALE_32BIT;
	filter_header->FilterIdHigh = 0x0000;
	filter_header->FilterIdLow = 0x0000;
	filter_header->FilterMaskIdHigh = 0x0000;
	filter_header->FilterMaskIdLow = 0x0000;
	filter_header->FilterFIFOAssignment = CAN_RX_FIFO0;
	filter_header->FilterActivation = ENABLE;

	if (HAL_CAN_ConfigFilter(hcan, filter_header) != HAL_OK) {
		return HAL_ERROR;
	}

	if (HAL_CAN_Start(hcan) != HAL_OK) {
		return HAL_ERROR;
	}
	//config rx interrupt
	//HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	return HAL_OK;
}


/*
 * @brief Init function which initialises time measurements for FSM
 * @param Dumbdash_TypeDef
 * @param CAN_Handle_TypeDef
 *  */
HAL_StatusTypeDef Dumbdash_Init(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *filter_header) {
	dev->tick_t.current_time 	= 0;
	dev->tick_t.elapsed_time 	= 0;
	dev->prev_enc_pos_1 		= 0;
	dev->prev_enc_pos_2 		= 0;
	for (uint8_t i = 0; i < 8; i++) {
		dev->tx_data[i] = 0;
		dev->rx_data[i] = 0;
	}

	return Dumbdash_CAN_HAL_Init(hcan, filter_header); // will have to change this soon I think
}

/*
 * @brief 	Abstracted CAN transmission function post init function calls. Supposed to aid with handling CAN errors if present.
 * @param 	Dumbdash_TypeDef
 * @return 	HAL status of CAN transaction
 * */

HAL_StatusTypeDef Dumbdash_Transmit_CAN_Msg(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx) {
	uint32_t tx_mailbox;
	tx->IDE = CAN_ID_STD;
	tx->RTR = CAN_RTR_DATA;
	tx->DLC = dev->len;
	tx->StdId = dev->id;
	return HAL_CAN_AddTxMessage(hcan, tx, dev->tx_data, &tx_mailbox);
}



/*
 * @brief 	Abstracted CAN receive function post init function calls. Supposed to aid with handling CAN errors if present.
 * @param 	Dumbdash_TypeDef
 * @param 	CAN_Driver_TypeDef
 * @return 	HAL status of CAN transaction
 * */

HAL_StatusTypeDef Dumbdash_Receive_CAN_Msg(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx) {
	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0);
	return HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, rx, dev->rx_data);
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
 * @brief 	Fills the CAN data field 'low' register with the two outputs from the rotary encoders
 * @param	Dumbdash_TypeDef
 * @param 	CAN_Driver_TypeDef
 * @return	Status of CAN transmission which also requests and begins CAN transmission
 */
HAL_StatusTypeDef Dumbdash_Transmit_Encoder_Data(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx) {

	// saved outputs to struct member for future debugging
	dev->prev_enc_pos_1 = Dumbdash_Read_Rotary_Encoder_Outputs(rot_enc_arr1);
	dev->prev_enc_pos_2 = Dumbdash_Read_Rotary_Encoder_Outputs(rot_enc_arr2);
	dev->tx_data[0] 	= dev->prev_enc_pos_1;
	dev->tx_data[1] 	= dev->prev_enc_pos_2;
	dev->id 			= CAN_ID_ROTARY_ENCODER;
	dev->len			= 2;

	return Dumbdash_Transmit_CAN_Msg(dev, hcan, tx);
}

HAL_StatusTypeDef Test_Read(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx) {
	dev->tx_data[0] = Dumbdash_Read_Rotary_Encoder_Outputs(rot_enc_arr2);
	dev->tx_data[1] = 0;
	dev->id 		= CAN_ID_ROTARY_ENCODER;
	dev->len		= 2;
	return Dumbdash_Transmit_CAN_Msg(dev, hcan, tx);
}

#define TEST_CAN_ID 	((uint32_t)	0x49B)
// ensure loopback is enabled
HAL_StatusTypeDef Test_Loopback(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx, CAN_RxHeaderTypeDef *rx) {
	dev->id = TEST_CAN_ID; //using stdid
	dev->len= 1;
	dev->tx_data[0] = 0x0F;

	if (Dumbdash_Transmit_CAN_Msg(dev, hcan, tx) != HAL_OK) {
		//Error_Handler();
		return HAL_ERROR;
	}

	if (Dumbdash_Receive_CAN_Msg(dev, hcan, rx) != HAL_OK) {
		//Error_Handler();
		return HAL_ERROR;
	}

	if (rx->StdId 	!= TEST_CAN_ID		||
		rx->DLC 	!= 1				||
		rx->IDE		!= CAN_ID_STD		||
		dev->rx_data[0] 		!= 0x0F) {

		return HAL_ERROR;
	}


	return HAL_OK; // all tests passed if true
}



/*
 * @brief 	Uses FSM control statements to implement a debounce timer on the 'lcb' (Launch Control Button) due the absence of a RC low pass filter
 * The debounce timer is an optional feature and can be configured using 'DEBOUNCE_TIME_MS' define
 * @param 	Dumbdash_TypeDef	...
 * @param 	data_ready_flag		Toggles to '1' upon external interrupt, used to trigger the function
 * @return	HAL status enum to prevent undefined behaviour
 * */
HAL_StatusTypeDef Dumbdash_Launch_Control_Interrupt_FSM(Dumbdash_TypeDef *dev, CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx, volatile uint8_t data_ready_flag) {

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

		//memset(dev->tx_data, 0x00, sizeof(dev->tx_data));

		dev->tx_data[0] = CAN_DATA_FIELD_LAUNCH_CONTROL_ON;
		dev->id			= CAN_ID_LAUNCH_CONTROL;
		can_status = Dumbdash_Transmit_CAN_Msg(dev, hcan, tx);

		dev->tx_data[0] = CAN_DATA_FIELD_LC_LED_STATUS_ON;
		dev->id			= CAN_ID_BUTTON_LED_STATUS;
		can_status = Dumbdash_Transmit_CAN_Msg(dev, hcan, tx);

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
