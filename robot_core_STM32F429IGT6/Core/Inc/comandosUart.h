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
	//cmd base:3
	NO_CMD = 0x00, 	//comando nulo
	HOLA = 0x01, 	//mensaje inicial
	OK_ = 0x02,			//confirmación de recepcion
	CANCEL_ = 0x03,		//instrucción de cancelación
	CMD_ERROR = 0x04,		//recepcion fallida
	MODO = 0x05,			//modo de funcionamiento (manual o automático)
	INFOMSG = 0x06,			//manda 6 caracteres con info
	//posicionamiento:
	COORD_X = 0x07,		//coordenada X (en pulsos del encoder) (consulta o parte de una trama)
	COORD_Y = 0x08,		//coordenada Y (un pulsos del encoder) (consulta o parte de una trama)
	COORD_ANG = 0x09,		//angulo del robot (Norte = 0 por omisión) (consulta o parte de una trama)
	POSICION = 0x0A,		//lectura de la coordenada actual
	DESTINO = 0x0B,		//comndo de siguiente coordenada de destino (instruccion)
	HOME = 0x0C,			//lectura de la coordenada de HOME (consulta)
	GO_HOME = 0x0D,		//dirigirse a la coordenada de HOME (instruccion)
	SET_HOME = 0x0E,		//definir como coordenada de HOME a la posición actual (instruccion)
	//comando de movimiento:
	AVANCE = 0x0F,			//envia instrucción de avance (modo manual) (instruccion)
	GIRO_IZQ = 0x10,		//envia instrucción de girar a la izquierda (modo manual) (instruccion)
	GIRO_DER = 0x11,		//envia instrucción de girar a la derecha (modo manual) (instruccion)
	RETROCEDE = 0x12,		//envia instrucción de retroceder (modo manual) (instruccion)
	STOP = 0x13,			//envia instrucción de detenerse (modo manual) (instruccion)
	//lecturas de movimiento:
	ACEL_AVANCE = 0x14,	//lectura de aceleración en sentido avance (consulta)
	DIST_AVANCE = 0x15,	//lectura de distancia avanzada (consulta)
	VEL_AVANCE = 0x16,		//lectura de velocidad de avance (consulta)
	ACEL_GIRO = 0x17,		//lectura de aceleración de giro (izq = sentido positivo) (consulta)
	DIST_GIRO = 0x18,		//lectura de angulo de giro (izq = sentido positivo) (consulta)
	VEL_GIRO = 0x19,		//lectura de velocidad de giro (izq = sentido positivo) (consulta)
	DELTA_ENC_L = 0x1A,	//delta del encoder izquierdo (consulta)
	DELTA_ENC_R = 0x1B,	//delta del encoder derecho (consulta)
}T_CMD;

typedef enum{
	AUTOMATICO,
	MANUAL,
	CALIBRA_MAG,
	PUNTO_A_PUNTO,
}T_MODO;



#endif /* INC_COMANDOSUART_H_ */
