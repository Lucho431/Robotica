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
	HOLA = 0x01,
	OK_,
	CANCEL_,
	CMD_ERROR,
	MODO,
	//posicionamiento:
	COORD_X,
	COORD_Y,
	COORD_ANG,
	DESTINO,
	//comando de movimiento:
	AVANCE,
	GIRO_IZQ,
	GIRO_DER,
	RETROCEDE,
	STOP,
	//lecturas de movimiento:
	ACEL_AVANCE,
	DIST_AVANCE,
	VEL_AVANCE,
	ACEL_GIRO,
	DIST_GIRO,
	VEL_GIRO,
	DELTA_ENC_L,
	DELTA_ENC_R,
}T_CMD;

typedef enum{
	AUTOMATICO,
	MANUAL,
}T_MODO;



#endif /* INC_COMANDOSUART_H_ */
