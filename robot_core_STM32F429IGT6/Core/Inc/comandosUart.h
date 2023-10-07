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
	NO_CMD = 0x00, 	//comando nulo
	HOLA = 0x01, 	//mensaje inicial
	OK_,			//confirmación de recepcion
	CANCEL_,		//instrucción de cancelación
	CMD_ERROR,		//recepcion fallida
	MODO,			//modo de funcionamiento (manual o automático)
	INFOMSG,			//manda 6 caracteres con info
	//posicionamiento:
	COORD_X,		//coordenada X (en pulsos del encoder) (consulta o parte de una trama)
	COORD_Y,		//coordenada Y (un pulsos del encoder) (consulta o parte de una trama)
	COORD_ANG,		//angulo del robot (Norte = 0 por omisión) (consulta o parte de una trama)
	POSICION,		//lectura de la coordenada actual
	DESTINO,		//comndo de siguiente coordenada de destino (instruccion)
	HOME,			//lectura de la coordenada de HOME (consulta)
	GO_HOME,		//dirigirse a la coordenada de HOME (instruccion)
	SET_HOME,		//definir como coordenada de HOME a la posición actual (instruccion)
	//comando de movimiento:
	AVANCE,			//envia instrucción de avance (modo manual) (instruccion)
	GIRO_IZQ,		//envia instrucción de girar a la izquierda (modo manual) (instruccion)
	GIRO_DER,		//envia instrucción de girar a la derecha (modo manual) (instruccion)
	RETROCEDE,		//envia instrucción de retroceder (modo manual) (instruccion)
	STOP,			//envia instrucción de detenerse (modo manual) (instruccion)
	//lecturas de movimiento:
	ACEL_AVANCE,	//lectura de aceleración en sentido avance (consulta)
	DIST_AVANCE,	//lectura de distancia avanzada (consulta)
	VEL_AVANCE,		//lectura de velocidad de avance (consulta)
	ACEL_GIRO,		//lectura de aceleración de giro (izq = sentido positivo) (consulta)
	DIST_GIRO,		//lectura de angulo de giro (izq = sentido positivo) (consulta)
	VEL_GIRO,		//lectura de velocidad de giro (izq = sentido positivo) (consulta)
	DELTA_ENC_L,	//delta del encoder izquierdo (consulta)
	DELTA_ENC_R,	//delta del encoder derecho (consulta)
}T_CMD;

typedef enum{
	AUTOMATICO,
	MANUAL,
	CALIBRA_MAG,
	PUNTO_A_PUNTO,
}T_MODO;



#endif /* INC_COMANDOSUART_H_ */
