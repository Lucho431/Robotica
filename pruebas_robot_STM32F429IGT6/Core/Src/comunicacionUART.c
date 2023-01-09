/*
 * comunicacionUART.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Luciano Salvatore
 */

#include "comunicacionUART.h"


//////variables externas////////

extern T_MODO modoFuncionamiento;
extern uint8_t esp01Presente;
extern uint8_t flag_encoders;


///////variables locales////////

UART_HandleTypeDef* uart_handler;

uint8_t tx [4];
uint8_t* p_rx;
T_CMD cmdEsperado = NO_CMD;
T_CMD cmdActual = NO_CMD;
uint8_t cmdSecuencia = 0; //contador decremental de tramas restantes de una instruccion


////prototipos de funciones/////
void iniciaInstruccion(void);
void continuaInstruccion(void);


void controlRxTxUART (uint8_t rx[]){

	if (rx[3] != 0){
		tx[0] = CMD_ERROR;
		tx[3] = '\0';
		HAL_UART_Transmit_IT(uart_handler, tx, 4);
		HAL_UART_Receive_IT(uart_handler, rx, 4);
		return;
	}

	p_rx = &rx[0];

	if (cmdEsperado != NO_CMD){
		continuaInstruccion();
	}else{
		iniciaInstruccion();
	}

	HAL_UART_Receive_IT(uart_handler, rx, 4);

} //end controlRxTxUART ()


void iniciaInstruccion (void){

	switch (p_rx[0]){
		case HOLA:
			esp01Presente = 1;
			cmdEsperado = NO_CMD;
			tx[0] = HOLA;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;

		case MODO:
			switch (p_rx[1]) {
				case AUTOMATICO:
					modoFuncionamiento = AUTOMATICO;
					flag_encoders = 0;
					tx[0] = OK_;
					tx[3] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
					break;
				case MANUAL:
//					status_movimiento = QUIETO;
					modoFuncionamiento = MANUAL;
					tx[0] = OK_;
					tx[3] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
					break;
				default:
					tx[0] = CMD_ERROR;
					tx[3] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
			} //end switch p_rx[1]
		break;
		case HOME:
			cmdActual = HOME;
			tx[0] = COORD_X;
			tx[1] = 0x0;
			tx[2] = 0xFA;
			tx[3] = '\0';
			cmdEsperado = OK_;
			cmdSecuencia = 2;
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case SET_HOME:
			cmdActual = SET_HOME;
			tx[0] = OK_;
			tx[3] = '\0';
			cmdEsperado = COORD_X;
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case AVANCE:
//			avance_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case RETROCEDE:
//			retroceso_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case GIRO_IZQ:
//			giroIzq_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case GIRO_DER:
//			giroDer_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case VEL_AVANCE:
			cmdEsperado = NO_CMD;

//			mpu9265_Read_Accel(&mpu9265);

			tx[0] = VEL_AVANCE;
//			tx[1] = (uint8_t)(mpu9265.Accel_X_RAW >> 8);
//			tx[2] = (uint8_t)(mpu9265.Accel_X_RAW & 0xFF);
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case COORD_ANG:
			cmdEsperado = NO_CMD;
/*
			mpu9265_Read_Magnet(&mpu9265);
			magX = mpu9265.Magnet_X_RAW;
			magY = mpu9265.Magnet_Y_RAW;

			direccion_f32 = atan2f(magY, magX);
			direccion_f32 *= (180.0/M_PI);
//			direccion_i16 = direccion_f32/180;
			direccion_i16 = direccion_f32;
			direccion_i16 -= 138;

			direccion_f32 *= (180.0/M_PI);
			direccion_i16 = direccion_f32/180;

			txUart[0] = COORD_ANG;
			txUart[1] = (uint8_t)(direccion_i16 >> 8);
			txUart[2] = (uint8_t)(direccion_i16 & 0xFF);
			txUart[3] = '\0';
			*/
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		default:
			tx[0] = CMD_ERROR;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
	} //end switch (cmdEsperado)

} //end iniciaInstruccion ()



void continuaInstruccion(void){

	if (cmdEsperado != p_rx[0]){
		cmdActual = NO_CMD;
		cmdEsperado = NO_CMD;
		cmdSecuencia = 0;
		tx[0] = CMD_ERROR;
		tx[3] = '\0';
		HAL_UART_Transmit_IT(uart_handler, tx, 4);
		return;
	}

	switch (cmdActual) {
		case HOME:
			switch (cmdSecuencia){
				case 2:
					tx[0] = COORD_Y;
					tx[1] = 0x0;
					tx[2] = 0xFB;
					tx[3] = '\0';
					cmdSecuencia--;
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case 1:
					tx[0] = COORD_ANG;
					tx[1] = 0x0;
					tx[2] = 0xFC;
					tx[3] = '\0';
					cmdSecuencia--;
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case 0:
					cmdActual = NO_CMD;
					cmdEsperado = NO_CMD;
				break;
			} //end switch cmdSecuencia

		break;

		case SET_HOME:
			switch (cmdEsperado){
				case COORD_X:
					//recibi la coordenada X
					cmdEsperado = COORD_Y;
					tx[0] = OK_;
					tx[3] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case COORD_Y:
					//recibi la coordenada Y
					cmdEsperado = COORD_ANG;
					tx[0] = OK_;
					tx[3] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case COORD_ANG:
					//recibi el angulo
					cmdEsperado = NO_CMD;
					tx[0] = OK_;
					tx[3] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				default:
				break;
			} //end switch cmdEsperado
		break;
		default:
		break;
	} //end switch cmdActual


} //end continuaInstruccion()





