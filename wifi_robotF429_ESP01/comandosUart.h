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
	HOLA = 0x01, 	//mensaje inicial
	OK_,			//confirmación de recepcion
	CANCEL_,		//instrucción de cancelación
	CMD_ERROR,		//recepcion fallida
	MODO,			//modo de funcionamiento (manual o automático)
	//posicionamiento:
	COORD_X,		//coordenada X (en pulsos del encoder)
	COORD_Y,		//coordenada Y (un pulsos del encoder)
	COORD_ANG,		//angulo del robot (Norte = 0 por omisión)
	DESTINO,		//comndo de siguiente coordenada de destino
	HOME,			//lectura de la coordenada de HOME
	GO_HOME,		//dirigirse a la coordenada de HOME
	SET_HOME,		//definir como coordenada de HOME a la posición actual
	//comando de movimiento:
	AVANCE,			//envia instrucción de avance (modo manual)
	GIRO_IZQ,		//envia instrucción de girar a la izquierda (modo manual)
	GIRO_DER,		//envia instrucción de girar a la derecha (modo manual)
	RETROCEDE,		//envia instrucción de retroceder (modo manual)
	STOP,			//envia instrucción de detenerse (modo manual)
	//lecturas de movimiento:
	ACEL_AVANCE,	//lectura de aceleración en sentido avance	
	DIST_AVANCE,	//lectura de distancia avanzada
	VEL_AVANCE,		//lectura de velocidad de avance
	ACEL_GIRO,		//lectura de aceleración de giro (izq = sentido positivo)
	DIST_GIRO,		//lectura de angulo de giro (izq = sentido positivo)
	VEL_GIRO,		//lectura de velocidad de giro (izq = sentido positivo)
	DELTA_ENC_L,	//delta del encoder izquierdo
	DELTA_ENC_R,	//delta del encoder derecho
}T_CMD;

typedef enum{
	AUTOMATICO,
	MANUAL,
}T_MODO;



#endif /* INC_COMANDOSUART_H_ */
