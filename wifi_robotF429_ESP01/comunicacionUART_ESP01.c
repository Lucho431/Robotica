/*
 * comunicacionUART.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Luciano Salvatore
 */

#include <ESP8266WiFi.h>
#include "comunicacionUART_ESP01.h"


//////variables externas////////

/*
extern T_MODO modoFuncionamiento;
extern uint8_t esp01Presente;
extern uint8_t flag_encoders;
*/

///////variables locales////////

//UART_HandleTypeDef* uart_handler;
//variables UART
uint8_t tx [4];
uint8_t* p_rx;
//variables comandos
T_CMD cmdEsperado = NO_CMD;
T_CMD cmdActual = NO_CMD;
uint8_t cmdSecuencia = 0; //contador decremental de tramas restantes de una instruccion
//variables MQTT
char* p_topic;
char* p_txt;
uint8_t flag_MQTT = 0;
//variables coordenadas
uint8_t stm_x;
uint8_t stm_y;
uint8_t stm_ang;

////prototipos de funciones/////

//void iniciaInstruccion(T_CMD);
void continuaInstruccion(void);


void init_controlRxTx (char topic[], char texto[]){
	p_topic = topic[0];
	p_txt = texto[0];
} //end init_controlRxTx()

uint8_t controlRxTxUART (uint8_t rx[]){

	flag_MQTT = 0;
	
	if (rx[3] != 0){
		tx[0] = CMD_ERROR;
		tx[3] = '\0';
		Serial.write (tx, 4);
		return;
	}

	p_rx = &rx[0];

	if (cmdEsperado != NO_CMD){
		continuaInstruccion();
	}else{
		iniciaInstruccion();
	}

	//HAL_UART_Receive_IT(uart_handler, rx, 4);
	
	return flag_MQTT;

} //end controlRxTxUART ()


void iniciaInstruccion (T_CMD cmdIni){
	
	switch (cmdIni){
		case HOLA:
			esp01Presente = 1;
			cmdEsperado = NO_CMD;
			tx[0] = HOLA;
			tx[3] = '\0';
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;

		case MODO:
			switch (p_rx[1]) {
				case AUTOMATICO:
					//modoFuncionamiento = AUTOMATICO;
					//flag_encoders = 0;
					tx[0] = OK_;
					tx[3] = '\0';
					//HAL_UART_Transmit_IT(uart_handler, tx, 4);
					break;
				case MANUAL:
//					status_movimiento = QUIETO;
					//modoFuncionamiento = MANUAL;
					tx[0] = OK_;
					tx[3] = '\0';
					//HAL_UART_Transmit_IT(uart_handler, tx, 4);
					break;
				default:
					tx[0] = CMD_ERROR;
					tx[3] = '\0';
					//HAL_UART_Transmit_IT(uart_handler, tx, 4);
			} //end switch p_rx[1]
		break;
		case HOME:
			cmdActual = HOME;
			cmdEsperado = COORD_X;
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case SET_HOME:
			cmdActual = SET_HOME;
			cmdEsperado = OK_;
			cmdSecuencia = 3;
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case AVANCE:
//			avance_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case RETROCEDE:
//			retroceso_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case GIRO_IZQ:
//			giroIzq_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case GIRO_DER:
//			giroDer_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
		//	HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case VEL_AVANCE:
			cmdEsperado = NO_CMD;

//			mpu9265_Read_Accel(&mpu9265);

			tx[0] = VEL_AVANCE;
//			tx[1] = (uint8_t)(mpu9265.Accel_X_RAW >> 8);
//			tx[2] = (uint8_t)(mpu9265.Accel_X_RAW & 0xFF);
			tx[3] = '\0';
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
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
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		default:
			tx[0] = CMD_ERROR;
			tx[3] = '\0';
			//HAL_UART_Transmit_IT(uart_handler, tx, 4);
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
		//HAL_UART_Transmit_IT(uart_handler, tx, 4);
		return;
	}

	switch (cmdActual) {
		case HOME:
			switch (cmdEsperado){
				case COORD_X:
					stm_x = p_rx[2]; //almacena X
					tx[0] = OK_;
					tx[3] = '\0';
					cmdEsperado = COORD_Y;
					//HAL_UART_Transmit_IT(uart_handler, tx, 4);
					Serial.write (tx, 4);
				break;
				case COORD_Y:
					stm_y = p_rx[2]; //almacena Y
					tx[0] = OK_;
					tx[3] = '\0';
					cmdEsperado = COORD_ANG;
					//HAL_UART_Transmit_IT(uart_handler, tx, 4);
					Serial.write (tx, 4);
				break;
				case COORD_ANG:
					stm_ang = p_rx[2]; //almacena el angulo
					sprintf(p_topic, "Info/Nodo_ESP01/VEL_AVANCE");
					sprintf(p_txt, "X=%d, Y=%d, Ang=%d", stm_x, stm_y, stm_ang);
					flag_MQTT = 1;
					
					tx[0] = OK_;
					tx[3] = '\0';
					cmdActual = NO_CMD;
					cmdEsperado = NO_CMD;
				break;
			} //end switch cmdEsperado

		break;

		case SET_HOME:
			switch (cmdSecuencia){
				case 3:
					//define la coord X
					tx[0] = COORD_X;
					tx[1] = 0xA1;
					tx[2] = 0x0;
					tx[3] = '\0';
					cmdSecuencia--;
					//HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case 2:
					//define la coord Y
					tx[0] = COORD_Y;
					tx[1] = 0xB1;
					tx[2] = 0x0;
					tx[3] = '\0';
					cmdSecuencia--;
					//HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case 1:
					//define la coord angulo
					tx[0] = COORD_ANG;
					tx[1] = 0xC1;
					tx[2] = 0x0;
					tx[3] = '\0';
					cmdSecuencia;
					//HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case 0:
					//Envia el MQTT
					cmdActual = NO_CMD;
					cmdEsperado = NO_CMD;
				default:
				break;
			} //end switch cmdEsperado
		break;
		default:
		break;
	} //end switch cmdActual


} //end continuaInstruccion()
