/*
 * comunicacionUART.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Luciano Salvatore
 */


#ifdef __cplusplus
extern "C" {
#endif


#ifndef INC_COMUNICACIONUART_C_
#define INC_COMUNICACIONUART_C_

//#include <ESP8266WiFi.h>
//#include "stdint.h"
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
char* p_rx;
uint8_t* p_cmd;
//variables comandos
T_CMD cmdEsperado = NO_CMD;
T_CMD cmdActual = NO_CMD;
uint8_t cmdSecuencia = 0; //contador decremental de tramas restantes de una instruccion
//variables MQTT
char* p_topic;
char* p_txt;
//variables coordenadas
int16_t stm_x;
int16_t stm_y;
int16_t stm_ang;
//variables de controlRxTx
T_CTRL_COM ctrl_com = NO_ACC;

////prototipos de funciones/////

//void iniciaInstruccion(T_CMD);
void continuaInstruccion(void);


void init_controlRxTx (char topic[], char texto[], uint8_t cmd[]){
	p_topic = &topic[0];
	p_txt = &texto[0];
	p_cmd = &cmd[0];
} //end init_controlRxTx()

T_CTRL_COM controlRxTxUART (char rx[]){

	ctrl_com = NO_ACC;	
	
	if (rx[7] != 0){
		p_cmd[0] = CMD_ERROR;
		p_cmd[7] = '\0';
		//Serial.write (tx, 4);
		return SEND_TXUART;
	}

	p_rx = &rx[0];

	if (cmdEsperado != NO_CMD){
		continuaInstruccion();
	}

	return ctrl_com;

} //end controlRxTxUART ()


void iniciaInstruccion (T_CMD cmdIni){
	
	switch (cmdIni){
		case HOLA:
			//esp01Presente = 1;
			cmdEsperado = NO_CMD;
			p_cmd[0] = HOLA;
			p_cmd[7] = '\0';
		break;

		case MODO:
			switch (p_rx[1]) {
				case AUTOMATICO:
				case MANUAL:
				break;
				default:
					p_cmd[0] = CMD_ERROR;
					p_cmd[7] = '\0';
			} //end switch p_rx[1]
		break;
		case POSICION:
			cmdActual = POSICION;
			cmdEsperado = POSICION;
		break;
		case DESTINO:
			cmdEsperado = NO_CMD;
		break;
		case SET_HOME:
			cmdActual = SET_HOME;
			cmdEsperado = OK_;
			cmdSecuencia = 3;
			ctrl_com = SEND_TXUART;
		break;
		case AVANCE:
//			avance_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
		break;
		case RETROCEDE:
//			retroceso_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
		break;
		case GIRO_IZQ:
//			giroIzq_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
		break;
		case GIRO_DER:
//			giroDer_cant += (uint16_t) (rx[2] + (rx[1] << 8));
			cmdEsperado = NO_CMD;
		break;
		case VEL_AVANCE:
			cmdEsperado = NO_CMD;

//			mpu9265_Read_Accel(&mpu9265);

			p_cmd[0] = VEL_AVANCE;
//			tx[1] = (uint8_t)(mpu9265.Accel_X_RAW >> 8);
//			tx[2] = (uint8_t)(mpu9265.Accel_X_RAW & 0xFF);
			p_cmd[7] = '\0';
		break;
		case DIST_GIRO:
			cmdActual = DIST_GIRO;
			cmdEsperado = COORD_ANG;
		break;
		default:
			p_cmd[0] = CMD_ERROR;
			p_cmd[7] = '\0';
		break;
	} //end switch (cmdEsperado)

} //end iniciaInstruccion ()



void continuaInstruccion(void){

	if (cmdEsperado != p_rx[0]){
		cmdActual = NO_CMD;
		cmdEsperado = NO_CMD;
		cmdSecuencia = 0;
		p_cmd[0] = CMD_ERROR;
		p_cmd[3] = '\0';
		ctrl_com = SEND_TXUART;
		return;
	}

	switch (cmdActual) {
		case POSICION:
			stm_x = (int16_t)  ((p_rx[1] << 8) |  p_rx[2]) ; //almacena X
			stm_y = (int16_t)  ((p_rx[3] << 8) |  p_rx[4]); //almacena Y
			stm_ang = (int16_t)  ((p_rx[5] << 8) |  p_rx[6]); //almacena el angulo
			sprintf(p_topic, "Info/Nodo_ESP01/POSICION");
			sprintf(p_txt, "X=%d, Y=%d, Ang=%d", stm_x, stm_y, stm_ang);
			//flag_MQTT = 1;
			
			p_cmd[0] = OK_;
			p_cmd[7] = '\0';
			cmdActual = NO_CMD;
			cmdEsperado = NO_CMD;
			ctrl_com = SEND_BOTH;
		break;

		case SET_HOME:
			switch (cmdSecuencia){
				case 3:
					//define la coord X
					p_cmd[0] = COORD_X;
					p_cmd[1] = 0xA1;
					p_cmd[2] = 0x0;
					p_cmd[7] = '\0';
					cmdSecuencia--;
					ctrl_com = SEND_TXUART;
				break;
				case 2:
					//define la coord Y
					p_cmd[0] = COORD_Y;
					p_cmd[1] = 0xB1;
					p_cmd[2] = 0x0;
					p_cmd[7] = '\0';
					cmdSecuencia--;
					ctrl_com = SEND_TXUART;
				break;
				case 1:
					//define la coord angulo
					p_cmd[0] = COORD_ANG;
					p_cmd[1] = 0xC1;
					p_cmd[2] = 0x0;
					p_cmd[7] = '\0';
					cmdSecuencia--;
					ctrl_com = SEND_TXUART;
				break;
				case 0:
					//Envia el MQTT
					sprintf(p_topic, "Info/Nodo_ESP01/SET_HOME");
					sprintf(p_txt, "X=%d, Y=%d, Ang=%d", stm_x, stm_y, stm_ang);
					
					cmdActual = NO_CMD;
					cmdEsperado = NO_CMD;
					ctrl_com = SEND_MQTT;
				default:
				break;
			} //end switch cmdEsperado
		break;
		case DIST_GIRO:
			sprintf(p_topic, "Info/Nodo_ESP01/COORD_ANG");
			sprintf(p_txt, "%d", (int16_t) ( (p_rx[1] << 8) |  p_rx[2]) );
			cmdActual = NO_CMD;
			cmdEsperado = NO_CMD;
			ctrl_com = SEND_MQTT;
		default:
		break;
	} //end switch cmdActual


} //end continuaInstruccion()


#endif /* INC_COMUNICACIONUART_C_ */


#ifdef __cplusplus
}
#endif
