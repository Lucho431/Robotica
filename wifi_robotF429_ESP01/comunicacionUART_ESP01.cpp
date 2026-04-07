/*
 * comunicacionUART.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Luciano Salvatore
 */

#ifndef INC_COMUNICACIONUART_C_
#define INC_COMUNICACIONUART_C_

//#include "stdint.h"
#include "comunicacionUART_ESP01.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "comandosUart.h"

//////variables externas////////
extern PubSubClient client;

///////variables locales////////

//UART_HandleTypeDef* uart_handler;
//variables UART
uint8_t txt_mqtt [50];
uint8_t size_texto;
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
void cmd_defaultCmd (uint8_t*);
void cmd_hola (uint8_t*);
void cmd_modo (uint8_t*);
void cmd_posicion (uint8_t*);
void cmd_home (uint8_t*);
void cmd_avance (uint8_t*);
void cmd_giroIzq (uint8_t*);
void cmd_giroDer (uint8_t*);
void cmd_retrocede (uint8_t*);

void ( *tablaCmd[SIZE_T_CMD] ) (uint8_t*) = {
												cmd_defaultCmd, //NO_CMD
												cmd_hola, //HOLA = 0x01, 		//mensaje inicial
												cmd_defaultCmd, //OK_ = 0x02,			//confirmación de recepcion
												cmd_defaultCmd, //CANCEL_ = 0x03,		//instrucción de cancelación
												cmd_defaultCmd, //CMD_ERROR = 0x04,	//recepcion fallida
												cmd_modo, //MODO = 0x05,		//modo de funcionamiento (manual o automático)
												cmd_defaultCmd, //INFOMSG = 0x06,		//manda 6 caracteres con info
												cmd_posicion, //POSICION = 0x07,	//lectura de la coordenada actual
												cmd_defaultCmd, //DESTINO = 0x08,		//comndo de siguiente coordenada de destino (instruccion)
												cmd_defaultCmd, //HOME = 0x09,		//lectura de la coordenada de HOME (consulta)
												cmd_defaultCmd, //GO_HOME = 0x0A,		//dirigirse a la coordenada de HOME (instruccion)
												cmd_avance, //AVANCE = 0x0B,		//envia instrucción de avance (modo manual) (instruccion)
												cmd_giroIzq, //GIRO_IZQ = 0x0C,	//envia instrucción de girar a la izquierda (modo manual) (instruccion)
												cmd_giroDer, //GIRO_DER = 0x0D,	//envia instrucción de girar a la derecha (modo manual) (instruccion)
												cmd_retrocede, //RETROCEDE = 0x0E,	//envia instrucción de retroceder (modo manual) (instruccion)
												cmd_defaultCmd, //STOP = 0x0F,		//envia instrucción de detenerse (modo manual) (instruccion)
												cmd_defaultCmd, //DELTA_ENC_L = 0x10,	//delta del encoder izquierdo (consulta)
												cmd_defaultCmd, //DELTA_ENC_R = 0x11,	//delta del encoder derecho (consulta)
												cmd_defaultCmd, //DATA_GYRO = 0x12,	//lectura de datos del giroscopio
												cmd_defaultCmd, //DATA_MAG = 0x13,	//lectura de datos del magnetometro (brujula)
												cmd_defaultCmd, //DATA_ACEL = 0x14,	//lectura de datos del acelerometro
												cmd_defaultCmd, //DATA_ENC = 0x15,	//lectura de datos de los ecoders [izq, der]
												cmd_defaultCmd, //DATA_SR04 = 0x16,	//lectura de datos del HC_SR04
												cmd_defaultCmd, //DATA_IR = 0x17,		//lectura de datos de los sensores ifrarrojos [izq, der]
};


uint8_t validaTrama (uint8_t *p_trama, uint8_t sizeTrama){
	
	if (p_trama[0] != 0x2A) return -1; //-1: error de cabecera
	
	uint8_t lastByte = sizeTrama - 1;
	uint8_t suma = 0;
	
	for (uint8_t i = 0; i < lastByte; i++){
		suma += p_trama[i];
	} //end for i
	if ( suma != p_trama[lastByte] ) return -2; //-2: error de checksum
	
	return 0; //todo bien
} //end validaRxUart ()


void ejecutaCmd (uint8_t *p_trama){
	tablaCmd [ p_trama[1] ] (p_trama);
} //end ejecutaCmd ()


uint8_t generaRta (void){
	return 0;
} //end generaRta ()


void cmd_defaultCmd (uint8_t *p_trama){
	switch (p_trama[2]){
		case ERROR_DESCONOCIDO:
			client.publish("Info/Nodo_ESP01/ERROR", "Tipo de error desconocido.");
			client.flush();
		break;
		case ERROR_CABECERA:
			client.publish("Info/Nodo_ESP01/ERROR", "Cabecera de trama erronea.");
			client.flush();
		break;
		case ERROR_CHECKSUM:
			client.publish("Info/Nodo_ESP01/ERROR", "Checksum no coincide.");
			client.flush();
		break;
		case ERROR_CMD:
			client.publish("Info/Nodo_ESP01/ERROR", "Comando no valido.");
			client.flush();
		break;
		case ERROR_PARAM:
			client.publish("Info/Nodo_ESP01/ERROR", "Parametros no validos.");
			client.flush();
		break;
		case ERROR_DENEGADO_CMD:
			client.publish("Info/Nodo_ESP01/ERROR", "Comando rechazado.");
			client.flush();
		break;
		case ERROR_DENEGADO_PARAM:
			client.publish("Info/Nodo_ESP01/ERROR", "Parametros rechazados.");
			client.flush();
		break;
		default:
			client.publish("Info/Nodo_ESP01/ERROR", "Tipo de error desconocido.");
			client.flush();
		break;
	} //end switch p_trama[2]
}


void cmd_hola (uint8_t *p_trama){
	if (p_trama[2] == PREG){
		p_trama[2] = RESP;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		Serial.write(p_trama, 4);
	} //end if PREG
	client.publish("Info/Nodo_ESP01/HOLA", "Saluda el robot.");
	client.flush();
} //end cmd_hola ()

void cmd_modo (uint8_t *p_trama){
	switch (p_trama[3]){
		case AUTOMATICO:
			client.publish("Info/Nodo_ESP01/MODO", "AUTOMATICO");
			client.flush();
		break;
		case MANUAL:
			client.publish("Info/Nodo_ESP01/MODO", "MANUAL");
			client.flush();
		break;
		case CALIBRA_MAG:
			client.publish("Info/Nodo_ESP01/MODO", "CALIBRA MAG");
			client.flush();
		break;
		case PUNTO_A_PUNTO:
			client.publish("Info/Nodo_ESP01/MODO", "PUNTO A PUNTO");
			client.flush();
		break;
		default:
			client.publish("Info/Nodo_ESP01/MODO", "DESCONOCIDO");
			client.flush();
		break;
	} //end switch p_trama
} //end cmd_modo ()


void cmd_posicion (uint8_t *p_trama){
	sprintf ((char*)txt_mqtt, "%d , %d , %d", (int16_t) (p_rx[3] + (p_rx[2] << 8)), (int16_t) (p_rx[5] + (p_rx[4] << 8)), (int16_t) (p_rx[7] + (p_rx[6] << 8)) );
	
	client.publish("Info/Nodo_ESP01/POS", (char*)txt_mqtt);
	client.flush();	
} //end cmd_posicion ()


void cmd_home (uint8_t *p_trama){
	if (p_trama[2] == READ){
		sprintf ((char*)txt_mqtt, "R: %d , %d , %d", (int16_t) (p_rx[4] + (p_rx[3] << 8)), (int16_t) (p_rx[6] + (p_rx[5] << 8)), (int16_t) (p_rx[8] + (p_rx[7] << 8)) );
	} else	if (p_trama[2] == READ){
		sprintf ((char*)txt_mqtt, "W: %d , %d , %d", (int16_t) (p_rx[4] + (p_rx[3] << 8)), (int16_t) (p_rx[6] + (p_rx[5] << 8)), (int16_t) (p_rx[8] + (p_rx[7] << 8)) );
	}

	client.publish("Info/Nodo_ESP01/HOME", (char*)txt_mqtt);
	client.flush();	
} //end cmd_home ()


void cmd_avance (uint8_t *p_trama){
	client.publish("Info/Nodo_ESP01/AVANCE", "Avance agregado.");
	client.flush();
} //end cmd_avance ()


void cmd_giroIzq (uint8_t *p_trama){
	client.publish("Info/Nodo_ESP01/GIRO_IZQ", "Giro a izquierda agregado.");
	client.flush();
} //end cmd_giroIzq ()


void cmd_giroDer (uint8_t *p_trama){
	client.publish("Info/Nodo_ESP01/GIRO_DER", "Giro a derecha agregado.");
	client.flush();
} //end cmd_giroDer ()


void cmd_retrocede (uint8_t*){
	client.publish("Info/Nodo_ESP01/GIRO_IZQ", "Retroceso agregado.");
	client.flush();
} //end cmd_retrocede ()

#endif /* INC_COMUNICACIONUART_C_ */
