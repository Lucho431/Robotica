/*
 * comandosUart.h
 *
 *  Created on: 18 mar. 2022
 *      Author: Luciano Salvatore
 */

#ifndef INC_COMANDOSUART_H_
#define INC_COMANDOSUART_H_

//#define HOLA
//#define OK
//#define CANCEL
//#define MODO
//#define COORD_X
//#define COORD_Y
//#define COORD_ANG
//#define DESTINO
//#define AVANZA
//#define GIRO_IZQ
//#define GIRO_DER
//#define RETROCEDE

typedef enum{
	//cmd base:
	NO_CMD = 0x00, 		//comando nulo
	HOLA = 0x01, 		//mensaje inicial
	OK_ = 0x02,			//confirmación de recepcion
	CANCEL_ = 0x03,		//instrucción de cancelación
	CMD_ERROR = 0x04,	//recepcion fallida
	MODO = 0x05,		//modo de funcionamiento
	INFOMSG = 0x06,		//mensaje de informacion
	//posicionamiento:
	POSICION = 0x07,	//lectura de la coordenada actual
	DESTINO = 0x08,		//comndo de siguiente coordenada de destino
	HOME = 0x09,		//coordenada de HOME (lectura o escritura)
	GO_HOME = 0x0A,		//dirigirse a la coordenada de HOME (modo punto a punto)
	//comando de movimiento:
	AVANCE = 0x0B,		//envia instrucción de avance (modo manual)
	GIRO_IZQ = 0x0C,	//envia instrucción de girar a la izquierda (modo manual)
	GIRO_DER = 0x0D,	//envia instrucción de girar a la derecha (modo manual)
	RETROCEDE = 0x0E,	//envia instrucción de retroceder (modo manual)
	STOP = 0x0F,		//envia instrucción de detenerse (modo manual)
	//debug:
	DELTA_ENC_L = 0x10,	//delta del encoder izquierdo (consulta)
	DELTA_ENC_R = 0x11,	//delta del encoder derecho (consulta)
	DATA_GYRO = 0x12,	//lectura de datos del giroscopio
	DATA_MAG = 0x13,	//lectura de datos del magnetometro (brujula)
	DATA_ACEL = 0x14,	//lectura de datos del acelerometro
	DATA_ENC = 0x15,	//lectura de datos de los ecoders [izq, der]
	DATA_SR04 = 0x16,	//lectura de datos del HC_SR04
	DATA_IR = 0x17,		//lectura de datos de los sensores ifrarrojos [izq, der]
	SIZE_T_CMD,			//SIEMPRE ULTIMO VALOR; tamaño del enum
}T_CMD;

typedef enum{
	AUTOMATICO,
	MANUAL,
	CALIBRA_MAG,
	PUNTO_A_PUNTO,
}T_MODO;

typedef enum{
	READ,
	WRITE,
}T_RW;

typedef enum{
	PREG,
	RESP,
}T_PREG_RESP;

typedef enum{
	ERROR_DESCONOCIDO,
	ERROR_CABECERA,
	ERROR_CHECKSUM,
	ERROR_CMD,
	ERROR_PARAM,
	ERROR_DENEGADO_CMD,
	ERROR_DENEGADO_PARAM,
}T_ERROR;

#endif /* INC_COMANDOSUART_H_ */
