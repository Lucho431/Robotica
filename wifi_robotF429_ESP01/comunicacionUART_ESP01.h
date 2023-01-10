/*
 * comunicacionUART_ESP01.h
 *
 *  Created on: 08 ene. 2023
 *      Author: Luciano Salvatore
 */

#ifndef INC_COMUNICACIONUART_H_
#define INC_COMUNICACIONUART_H_

#include "comandosUart.h"
#include <Arduino.h>

typedef enum{
	NO_ACC,
	SEND_MQTT,
	SEND_TXUART,
	SEND_BOTH,
}T_CTRL_COM;

void init_controlRxTx (char[], char[], uint8_t[]);
T_CTRL_COM controlRxTxUART (char []);
void iniciaInstruccion (T_CMD);

#endif /* INC_COMUNICACIONUART_H_ */
