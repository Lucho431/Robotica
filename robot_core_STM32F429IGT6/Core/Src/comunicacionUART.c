/*
 * comunicacionUART.c
 *
 *  Created on: 25 oct. 2022
 *      Author: Luciano Salvatore
 */

#include "comunicacionUART.h"


//////variables externas////////

extern T_MODO modoFuncionamiento;
extern uint8_t estatus_calibraMag;
extern uint8_t flag_esp01Presente;

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

uint8_t tx [MAX_TX];
uint8_t* p_rx;
T_CMD cmdEsperado = NO_CMD;
T_CMD cmdActual = NO_CMD;
uint8_t cmdSecuencia = 0; //contador decremental de tramas restantes de una instruccion

int16_t aux_direccion;

uint8_t tramaValida = 0;

////prototipos de funciones/////
void iniciaInstruccion(void);

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
												cmd_home, //HOME = 0x09,		//lectura de la coordenada de HOME (consulta)
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
	}else{
		iniciaInstruccion();
	}

	HAL_UART_Receive_IT(uart_handler, p_rx, 8);

} //end controlRxTxUART ()


void iniciaInstruccion (void){

	switch (p_rx[0]){
		case POSICION:
			tx[0] = POSICION;
			if (posX_i16 < 0){
				tx[1] = 1;
				tx[2] = -posX_i16;
			}else{
				tx[1] = 0;
				tx[2] = posX_i16;
			}
			if (posY_i16 < 0){
				tx[3] = 1;
				tx[4] = -posY_i16;
			}else{
				tx[3] = 0;
				tx[4] = posY_i16;
			}
//			tx[1] = posX_i16 >> 8;
//			tx[2] = posX_i16 & 0xFF;
//			tx[3] = posY_i16 >> 8;
//			tx[4] = posY_i16 & 0xFF;
			if (direccion_i16 < 0){
//				aux_direccion = direccion_i16 + 360;
				tx[5] = 1;
				tx[6] = -direccion_i16;
			}else{
//				aux_direccion = direccion_i16;
				tx[5] = 0;
				tx[6] = direccion_i16;
			}
//			tx[5] = aux_direccion >>8;
//			tx[6] = aux_direccion & 0xFF;
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

			if (modoFuncionamiento == PUNTO_A_PUNTO){
				posX_dest = (p_rx[2] + (p_rx[1] << 8));
				posY_dest = (p_rx[4] + (p_rx[3] << 8));
				direccion_dest = (p_rx[6] + (p_rx[5] << 8));
				flag_dest = 1;
			}

			tx[0] = OK_;
			tx[7] = '\0';
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

			if (modoFuncionamiento == PUNTO_A_PUNTO){
				posX_dest = posX_home;
				posY_dest = posY_home;
				direccion_dest = direccion_home;
				flag_dest = 1;
			}

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
		case 55: //VEL_AVANCE:
			cmdEsperado = NO_CMD;

//			mpu9265_Read_Accel(&mpu9265);

//			tx[1] = (uint8_t)(mpu9265.Accel_X_RAW >> 8);
//			tx[2] = (uint8_t)(mpu9265.Accel_X_RAW & 0xFF);
			tx[7] = '\0';
			HAL_UART_Transmit_IT(uart_handler, tx, 8);
		break;
		case 56: //DIST_GIRO:
			cmdEsperado = NO_CMD;
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
/////       NEW         ///////
///////////////////////////////


void procesaCmd (uint8_t p_trama[], uint8_t sizeTrama){

	if (p_trama[0] != 0x2A){
		p_trama[0] = 0x2A;
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_CABECERA;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
		HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
		return;
	} //end if 0x2A

	uint8_t lastByte = sizeTrama - 1;
	uint8_t suma = 0;

	for (uint8_t i = 0; i < lastByte; i++){
		suma += p_trama[i];
	} //end for i

	if ( suma != p_trama[lastByte] ){
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_CHECKSUM;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
		HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
		return;
	} //end if suma != checksum

	tablaCmd [ p_trama[1] ] (p_trama);
 } //end procesaCmd ()


void cmd_defaultCmd (uint8_t *p_trama){
	p_trama[1] = CMD_ERROR;
	p_trama[2] = ERROR_CMD;
	p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
	HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_defaultCmd ()


void cmd_hola (uint8_t *p_trama){
	if (p_trama[2] == PREG){
		p_trama[2] = RESP;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
	}else if (p_trama[2] != RESP){
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_PARAM;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
	}
	HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
	flag_esp01Presente = 1;
	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_hola ()


void cmd_modo (uint8_t *p_trama){
	if (p_trama[2] == WRITE){ // R/W byte
		switch (p_trama[3]){ //parametro
			case AUTOMATICO:
				modoFuncionamiento = AUTOMATICO;
				flag_dest = 0;
				HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
				break;
			case MANUAL:
				modoFuncionamiento = MANUAL;
				flag_dest = 0;
				HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
				break;
			case CALIBRA_MAG:
				modoFuncionamiento = CALIBRA_MAG;
				estatus_calibraMag = 0;
				flag_dest = 0;
				HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
				break;
			case PUNTO_A_PUNTO:
				modoFuncionamiento = PUNTO_A_PUNTO;
				flag_dest = 0;
				HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
				break;
			default:
				tx[0] = CMD_ERROR;
				tx[1] = 2;
				tx[7] = '\0';
				HAL_UART_Transmit_IT(uart_handler, tx, 8);
		} //end switch p_trama 3
		modoFuncionamiento = p_trama[3];
	}else if (p_trama[2] == READ){ // R/W byte
		p_trama[3] = modoFuncionamiento;
		p_trama[4] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2] + p_trama[3]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
	}else{ // R/W byte
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_PARAM;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
	} //end if p_Trama 2

	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_modo


void cmd_posicion (uint8_t *p_trama){
	p_trama[2] = posX_i16 >> 8;
	p_trama[3] = posX_i16 & 0xFF;
	p_trama[4] = posY_i16 >> 8;
	p_trama[5] = posY_i16 & 0xFF;
	p_trama[6] = direccion_i16 >> 8;
	p_trama[7] = direccion_i16 & 0xFF;
	p_trama[8] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2] + p_trama[3] + p_trama[4] + p_trama[5] + p_trama[6] + p_trama[7]);
	HAL_UART_Transmit_IT(uart_handler, p_trama, 9);

	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_posicion ()


void cmd_home (uint8_t *p_trama){
	if (p_trama[2] == READ){
		p_trama[3] = posX_home >> 8;
		p_trama[4] = posX_home & 0xFF;
		p_trama[5] = posY_home >> 8;
		p_trama[6] = posY_home & 0xFF;
		p_trama[7] = direccion_home >> 8;
		p_trama[8] = direccion_home & 0xFF;
		p_trama[9] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2] + p_trama[3] + p_trama[4] + p_trama[5] + p_trama[6] + p_trama[7] + p_trama[8] + p_trama[9]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 10);
	}else if (p_trama[2] == WRITE){
		posX_home = (int16_t) (p_rx[4] + (p_rx[3] << 8));
		posY_home = (int16_t) (p_rx[6] + (p_rx[5] << 8));
		direccion_home = (int16_t) (p_rx[8] + (p_rx[7] << 8));
		HAL_UART_Transmit_IT(uart_handler, p_trama, 10);
	}else{
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_PARAM;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
	} //ens if p_trama 2

	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_home

void cmd_avance (uint8_t *p_trama){
	if (modoFuncionamiento == MANUAL){
		avance_cant += (uint16_t) (p_trama[3] + (p_trama[2] << 8));
		HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
	}else{
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_DENEGADO_CMD;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
	}
	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_avance ()


void cmd_giroIzq (uint8_t *p_trama){
	if (modoFuncionamiento == MANUAL){
		giroIzq_cant += (uint16_t) (p_trama[3] + (p_trama[2] << 8));
		HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
	}else{
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_DENEGADO_CMD;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
	}
	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_giroIzq ()


void cmd_giroDer (uint8_t *p_trama){
	if (modoFuncionamiento == MANUAL){
		giroDer_cant += (uint16_t) (p_trama[3] + (p_trama[2] << 8));
		HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
	}else{
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_DENEGADO_CMD;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
	}
	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_giroDer ()


void cmd_retrocede (uint8_t *p_trama){
	if (modoFuncionamiento == MANUAL){
		retroceso_cant += (uint16_t) (p_trama[3] + (p_trama[2] << 8));
		HAL_UART_Transmit_IT(uart_handler, p_trama, 5);
	}else{
		p_trama[1] = CMD_ERROR;
		p_trama[2] = ERROR_DENEGADO_CMD;
		p_trama[3] = (uint8_t) (p_trama[0] + p_trama[1] + p_trama[2]);
		HAL_UART_Transmit_IT(uart_handler, p_trama, 4);
	}
	HAL_UART_Receive_IT(uart_handler, p_trama, MAX_RX);
} //end cmd_retrocede ()

