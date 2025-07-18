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

extern uint8_t pos_x;
extern uint8_t pos_y;
extern uint8_t pos_ang;
extern int16_t posX_i16;
extern int16_t posY_i16;
extern int16_t posX_home;
extern int16_t posY_home;
extern int16_t direccion_i16;
extern int16_t direccion_home;

extern uint16_t avance_cant;
extern uint16_t retroceso_cant;
extern uint16_t giroIzq_cant;
extern uint16_t giroDer_cant;

extern int16_t posX_dest;
extern int16_t posY_dest;
extern int16_t direccion_dest;
extern uint8_t flag_dest;

///////variables locales////////

UART_HandleTypeDef* uart_handler;

uint8_t tx [8];
uint8_t* p_rx;
T_CMD cmdEsperado = NO_CMD;
T_CMD cmdActual = NO_CMD;
uint8_t cmdSecuencia = 0; //contador decremental de tramas restantes de una instruccion

int16_t aux_direccion;


////prototipos de funciones/////
void iniciaInstruccion(void);
void continuaInstruccion(void);

void init_controlRxTx (UART_HandleTypeDef* huart){
	uart_handler = huart;
} //end init_controlRxTx ()


void controlRxTxUART (uint8_t rx[]){

	if (rx[7] != 0){
		tx[0] = CMD_ERROR;
		tx[1] = 1;
		tx[7] = '\0';
		HAL_UART_Transmit_IT(uart_handler, tx, 8);
		HAL_UART_Receive_IT(uart_handler, rx, 8);
		return;
	}

	p_rx = &rx[0];

	if (cmdEsperado != NO_CMD){
		continuaInstruccion();
	}else{
		iniciaInstruccion();
	}

	HAL_UART_Receive_IT(uart_handler, p_rx, 8);

} //end controlRxTxUART ()


void iniciaInstruccion (void){

	switch (p_rx[0]){
		case HOLA:
			esp01Presente = 1;
			cmdEsperado = NO_CMD;
			tx[0] = HOLA;
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;

		case MODO:
			switch (p_rx[1]) {
				case AUTOMATICO:
					modoFuncionamiento = AUTOMATICO;
					flag_dest = 0;
					flag_encoders = 0;
					tx[0] = OK_;
					tx[7] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
					break;
				case MANUAL:
//					status_movimiento = QUIETO;
					modoFuncionamiento = MANUAL;
					flag_dest = 0;
					tx[0] = OK_;
					tx[7] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
					break;
				case CALIBRA_MAG:
					modoFuncionamiento = CALIBRA_MAG;
					flag_dest = 0;
					tx[0] = OK_;
					tx[7] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
					break;
				case PUNTO_A_PUNTO:
					modoFuncionamiento = PUNTO_A_PUNTO;
					flag_dest = 0;
					tx[0] = OK_;
					tx[7] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
					break;
				default:
					tx[0] = CMD_ERROR;
					tx[1] = 2;
					tx[7] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
			} //end switch p_rx[1]
		break;
		case POSICION:
			tx[0] = POSICION;
			tx[1] = posX_i16 >> 8;
			tx[2] = posX_i16 & 0xFF;
			tx[3] = posY_i16 >> 8;
			tx[4] = posY_i16 & 0xFF;
			if (direccion_i16 < 0){
				aux_direccion = direccion_i16 + 360;
			}else{
				aux_direccion = direccion_i16;
			}
			tx[5] = aux_direccion >>8;
			tx[6] = aux_direccion & 0xFF;
			tx[7] = '\0';
//			cmdEsperado = OK_;
			cmdEsperado = NO_CMD;
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
		case DESTINO:
			if (flag_dest != 0){
				tx[0] = CANCEL_;
				tx[7] = '\0';
				HAL_UART_Transmit_IT(uart_handler, tx, 8);
				break;
			}

			posX_dest = (p_rx[2] + (p_rx[1] << 8));
			posY_dest = (p_rx[4] + (p_rx[3] << 8));
			direccion_dest = (p_rx[6] + (p_rx[5] << 8));
			if (modoFuncionamiento == PUNTO_A_PUNTO){
				flag_dest = 1;
			}

			tx[0] = OK_;
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
			break;
		break;
		case HOME:
			tx[0] = HOME;
			tx[1] = posX_home >> 8;
			tx[2] = posX_home & 0xFF;
			tx[3] = posY_home >> 8;
			tx[4] = posY_home & 0xFF;
			tx[5] = direccion_home >>8;
			tx[6] = direccion_home & 0xFF;
			tx[7] = '\0';
			cmdEsperado = NO_CMD;
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
			break;
		break;
		case GO_HOME:
			if (flag_dest != 0){
				tx[0] = CANCEL_;
				tx[7] = '\0';
				HAL_UART_Transmit_IT(uart_handler, tx, 8);
				break;
			}

			posX_dest = posX_home;
			posY_dest = posY_home;
			direccion_dest = direccion_home;
			if (modoFuncionamiento == PUNTO_A_PUNTO){
				flag_dest = 1;
			}

			tx[0] = OK_;
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
			break;
		break;
		case SET_HOME:
			posX_home = (p_rx[2] + (p_rx[1] << 8));
			posY_home = (p_rx[4] + (p_rx[3] << 8));
			direccion_home = (p_rx[6] + (p_rx[5] << 8));

			tx[0] = OK_;
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
			break;
		break;
		case AVANCE:
			avance_cant += (uint16_t) (p_rx[2] + (p_rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
		case RETROCEDE:
			retroceso_cant += (uint16_t) (p_rx[2] + (p_rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
		case GIRO_IZQ:
			giroIzq_cant += (uint16_t) (p_rx[2] + (p_rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
		case GIRO_DER:
			giroDer_cant += (uint16_t) (p_rx[2] + (p_rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
		case VEL_AVANCE:
			cmdEsperado = NO_CMD;

//			mpu9265_Read_Accel(&mpu9265);

			tx[0] = VEL_AVANCE;
//			tx[1] = (uint8_t)(mpu9265.Accel_X_RAW >> 8);
//			tx[2] = (uint8_t)(mpu9265.Accel_X_RAW & 0xFF);
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
		case DIST_GIRO:
			cmdEsperado = NO_CMD;
			tx[0] = DIST_GIRO;
			if (direccion_i16 < 0){
				aux_direccion = direccion_i16 + 360;
			}else{
				aux_direccion = direccion_i16;
			}

			tx[1] = (uint8_t)(aux_direccion >> 8);
			tx[2] = (uint8_t)(aux_direccion & 0xFF);
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
		case OK_:
			cmdEsperado = NO_CMD;
		break;
		default:
			tx[0] = CMD_ERROR;
			tx[1]= 3;
			tx[2]= p_rx[0];
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
	} //end switch (cmdEsperado)

} //end iniciaInstruccion ()


void continuaInstruccion(void){

	if (cmdEsperado != p_rx[0]){
		cmdActual = NO_CMD;
		cmdEsperado = NO_CMD;
		cmdSecuencia = 0;
		tx[0] = CMD_ERROR;
		tx[1] = 4;
		tx[7] = '\0';
		HAL_UART_Transmit_IT(uart_handler, tx, 8);
		return;
	}

	switch (cmdActual) {
		case POSICION:
			switch (cmdSecuencia){
				case 2:
					tx[0] = COORD_Y;
					tx[1] = posY_i16 >> 8;
					tx[2] = posY_i16 & 0xFF;
					tx[7] = '\0';
					cmdSecuencia--;
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
				break;
				case 1:
					tx[0] = COORD_ANG;
					tx[1] = direccion_i16 >>8;
					tx[2] = direccion_i16 & 0xFF;
					tx[7] = '\0';
					cmdSecuencia--;
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
				break;
				case 0:
					cmdActual = NO_CMD;
					cmdEsperado = NO_CMD;
				break;
			} //end switch cmdSecuencia

		break;
		case HOME:
			switch (cmdSecuencia){
				case 2:
					tx[0] = COORD_Y;
					tx[1] = 0x0;
					tx[2] = posY_i16 & 0xFF;
					tx[7] = '\0';
					cmdSecuencia--;
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
				break;
				case 1:
					tx[0] = COORD_ANG;
					tx[1] = 0x0;
					tx[2] = pos_ang;
					tx[7] = '\0';
					cmdSecuencia--;
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
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
					pos_x = p_rx[1];//recibi la coordenada X
					cmdEsperado = COORD_Y;
					tx[0] = OK_;
					tx[7] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
				break;
				case COORD_Y:
					pos_y = p_rx[1];//recibi la coordenada Y
					cmdEsperado = COORD_ANG;
					tx[0] = OK_;
					tx[7] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
				break;
				case COORD_ANG:
					pos_ang = p_rx[1];//recibi el angulo
					cmdEsperado = NO_CMD;
					tx[0] = OK_;
					tx[7] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 8);
				break;
				default:
				break;
			} //end switch cmdEsperado
		break;
		default:
		break;
	} //end switch cmdActual


} //end continuaInstruccion()


void send_info (uint8_t msg[]){
	if (cmdActual == NO_CMD){
		tx[0] = INFOMSG;
		tx[1] = msg[0];
		tx[2] = msg[1];
		tx[3] = msg[2];
		tx[4] = msg[3];
		tx[5] = msg[4];
		tx[6] = msg[5];
		tx[7] = '\0';
		HAL_UART_Transmit_IT(uart_handler, tx, 8);
	}
}

///////////////////////////////
/////       OLD         ///////
///////////////////////////////

/*
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
		case POSICION:
			cmdActual = POSICION;
			tx[0] = COORD_X;
			tx[1] = posX_i16 >> 8;
			tx[2] = posX_i16 & 0xFF;
			tx[3] = '\0';
			cmdEsperado = OK_;
			cmdSecuencia = 2;
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case HOME:
			cmdActual = HOME;
			tx[0] = COORD_X;
			tx[1] = 0x0;
			tx[2] = posX_i16 & 0xFF;
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
			avance_cant += (uint16_t) (p_rx[2] + (p_rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case RETROCEDE:
			retroceso_cant += (uint16_t) (p_rx[2] + (p_rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case GIRO_IZQ:
			giroIzq_cant += (uint16_t) (p_rx[2] + (p_rx[1] << 8));
			cmdEsperado = NO_CMD;
			tx[0] = OK_;
			tx[3] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 4);
		break;
		case GIRO_DER:
			giroDer_cant += (uint16_t) (p_rx[2] + (p_rx[1] << 8));
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
		case DIST_GIRO:
			cmdEsperado = NO_CMD;

			tx[0] = COORD_ANG;
			tx[1] = (uint8_t)(direccion_i16 >> 8);
			tx[2] = (uint8_t)(direccion_i16 & 0xFF);
			tx[3] = '\0';
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
		case POSICION:
			switch (cmdSecuencia){
				case 2:
					tx[0] = COORD_Y;
					tx[1] = posY_i16 >> 8;
					tx[2] = posY_i16 & 0xFF;
					tx[3] = '\0';
					cmdSecuencia--;
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case 1:
					tx[0] = COORD_ANG;
					tx[1] = direccionMag_grad_i16 >>8;
					tx[2] = direccionMag_grad_i16 & 0xFF;
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
		case HOME:
			switch (cmdSecuencia){
				case 2:
					tx[0] = COORD_Y;
					tx[1] = 0x0;
					tx[2] = posY_i16 & 0xFF;
					tx[3] = '\0';
					cmdSecuencia--;
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case 1:
					tx[0] = COORD_ANG;
					tx[1] = 0x0;
					tx[2] = pos_ang;
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
					pos_x = p_rx[1];//recibi la coordenada X
					cmdEsperado = COORD_Y;
					tx[0] = OK_;
					tx[3] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case COORD_Y:
					pos_y = p_rx[1];//recibi la coordenada Y
					cmdEsperado = COORD_ANG;
					tx[0] = OK_;
					tx[3] = '\0';
					HAL_UART_Transmit_IT(uart_handler, tx, 4);
				break;
				case COORD_ANG:
					pos_ang = p_rx[1];//recibi el angulo
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

*/



