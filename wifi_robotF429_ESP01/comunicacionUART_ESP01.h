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

void init_controlRxTx (char [], char []);
uint8_t controlRxTxUART (uint8_t []);
void iniciaInstruccion (T_CMD);

#endif /* INC_COMUNICACIONUART_H_ */
