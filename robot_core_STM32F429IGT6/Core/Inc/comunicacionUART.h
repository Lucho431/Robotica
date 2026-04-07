/*
 * comunicacionUART.h
 *
 *  Created on: 25 oct. 2022
 *      Author: luciano
 */

#ifndef INC_COMUNICACIONUART_H_
#define INC_COMUNICACIONUART_H_

#include "comandosUart.h"
#include "main.h"

void init_controlRxTx (UART_HandleTypeDef*);
void send_info (uint8_t []);

void procesaCmd (uint8_t [], uint8_t);
#endif /* INC_COMUNICACIONUART_H_ */
