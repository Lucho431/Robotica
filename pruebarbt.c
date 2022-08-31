/* 
 * PASOS A CUMPLIR:
 * 
 * 1) setear los vectores de velocidad
 * 
 * 2) obtener modulo y direccion de las velocidades finales
 * 
 * 3) corroborar modulo y direccion actual 
 * 
 * 4) setear el cambio progresivo de velocidad (aceleracion, desaceleracion y sentido)
 * 
 * 5) corroborar contra el encoder 
 * 
 * POR CHECKEAR:
 * 
 * 1) ¿cuánto debería acelerar/desacelerar?
 * 
 * */


typedef enum{
	QUIETO,
	EN_AVANCE,
	EN_RETROCESO,
}T_STAT_MOV;


//variables encoders
uint8_t encoderL;
uint8_t encoderR;

//variables de vectores de velocidad
int8_t velL; //de -5 a +5 ranuras por 210ms.
int8_t velR; //de -5 a +5 ranuras por 210ms.

//variables de velocidad instantanea
uint8_t instaVelL;
uint8_t instaVelR;
uint8_t instaDirL;
uint8_t instaDirR;

//variables de velocidad final
uint8_t finalVelL;
uint8_t finalVelR;
uint8_t finalDirL;
uint8_t finalDirR;

//variables de estado de velocidades
T_STAT_MOV status_movimiento = QUIETO;

//vaiables de aceleración
uint8_t acel = 2;


uint8_t velocidades (void);
uint8_t aceleraciones (void);
uint8_t encoder (void);

uint8_t velocidades (void){
	
	finalVelL = (uint8_t) abs(velL); //saca el signo del int8_t
	finalVelR = (uint8_t) abs(velR); //saca el signo del int8_t
	finalDirL = (uint8_t) (velL >> 7); //1: retrocede; 0: avanza;
	finalDirR = (uint8_t) (velR >> 7); //1: retrocede; 0: avanza;
}

uint8_t aceleraciones (void){
	
	switch (status_movimiento){
		case QUIETO:
			if (finalDirL != 0){
				//seteo ruedas en retroceso
				status_movimiento = EN_RETROCESO;
				break;
			}else{
				//seteo ruedas en avance
				if (finalVelL > 0){
					status_movimiento = EN_RETROCESO;
				}
			}
			
			
		break;
		case EN_AVANCE:
			
			if (finalDirL != 0){

			}

		break;
		case EN_RETROCESO:
			if (!finalDirL){
				insta
			}
		default:
		break;
	}
	
	
}
