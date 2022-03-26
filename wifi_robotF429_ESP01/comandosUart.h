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
	HOLA = 0x01,
	OK_,
	CANCEL_,
	CMD_ERROR,
	MODO,
	COORD_X,
	COORD_Y,
	COORD_ANG,
	DESTINO,
	AVANCE,
	GIRO_IZQ,
	GIRO_DER,
	RETROCEDE,
}T_CMD;

typedef enum{
	AUTOMATICO,
	MANUAL,
}T_MODO;



#endif /* INC_COMANDOSUART_H_ */
