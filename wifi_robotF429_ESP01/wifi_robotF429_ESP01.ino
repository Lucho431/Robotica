#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "comandosUart.h"
#include "comunicacionUART_ESP01.h"
//#include "comunicacionUART_ESP01.c"

#define RXUART_BUFFER_SIZE 4

#define TICK_PERIOD     10 //en ms.




typedef enum{
    TRYING_WIFI,
    WIFI_TIMING_OUT,
    TRYING_MQTT,
    MQTT_TIMING_OUT,
    ALL_CONNECTED,
}T_CONN;


//variables red
/*
const char* ssid = "ESP8266_CU";
const char* password = "tclpqtp123456";
const char* mqtt_server = "192.168.4.1";
*/
const char* ssid = "TeleCentro-c078";
const char* password = "tclpqtp123456";
const char* mqtt_server = "192.168.0.86";


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

String clientId;
char rxUart [RXUART_BUFFER_SIZE];
uint8_t sizeCmd = 0;
uint8_t cmdFrame [4];


//variables timer
uint8_t flag_tick = 0;
unsigned long last_tick = 0;

//variables conexion
T_CONN conn_status = TRYING_WIFI;
uint16_t reconnect_time = 0; // 1 * 10ms

//variable de texto
char texto[50];
char txtTopic[50];

//variables de comando
uint8_t cmdEnProceso = 0;

void timer_update(void){
    
    unsigned long now = millis();
    
    if(now > last_tick + TICK_PERIOD){
        flag_tick = 1;
        last_tick = now;        
    }
} //end timer_update ()


void callback_MQTT(char* topic, byte* payload, unsigned int length) {
	
	char pl[20];
	/*
	Serial.print("Message arrived [");
	Serial.print(topic);
	Serial.print("] ");
	*/
	for (int i = 0; i < length; i++) {
		pl[i] = (char)payload[i];
	}
	pl[length] = '\0';
	/*
	Serial.print(pl);
	Serial.println();
	*/
	
	if ( !strcmp(pl, "up") ){//si recibe por MQTT el comando "up"
		//Serial.print ("arriba");
		cmdFrame[0] = AVANCE;
		cmdFrame[1] = 0x00;
		cmdFrame[2] = 0x14;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "down") ){//si recibe por MQTT el comando "down"
		//Serial.print ("abajo");
		cmdFrame[0] = RETROCEDE;
		cmdFrame[1] = 0x00;
		cmdFrame[2] = 0x14;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "left") ){//si recibe por MQTT el comando "left"
		//Serial.print ("izquierda");
		cmdFrame[0] = GIRO_IZQ;
		cmdFrame[1] = 0x00;
		cmdFrame[2] = 0x07;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "right") ){//si recibe por MQTT el comando "right"
		//Serial.print ("derecha");
		cmdFrame[0] = GIRO_DER;
		cmdFrame[1] = 0x00;
		cmdFrame[2] = 0x07;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "manual") ){//si recibe por MQTT el comando "manual"
		//Serial.print ("manual");
		cmdFrame[0] = MODO;
		cmdFrame[1] = MANUAL;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "auto") ){//si recibe por MQTT el comando "auto"
		//Serial.print ("auto");
		cmdFrame[0] = MODO;
		cmdFrame[1] = AUTOMATICO;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "v_avan") ){//si recibe por MQTT el comando "auto"
		//Serial.print ("auto");
		cmdFrame[0] = VEL_AVANCE;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "ang+90") ){//si recibe por MQTT el comando "d_giro"
		//Serial.print ("auto");
		cmdFrame[0] = COORD_ANG;
		cmdFrame[1] = 90;
		cmdFrame[2] = 0;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "ang-90") ){//si recibe por MQTT el comando "d_giro"
		//Serial.print ("auto");
		cmdFrame[0] = COORD_ANG;
		cmdFrame[1] = 0;
		cmdFrame[2] = 90;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		return;
	}
	
	if ( !strcmp(pl, "d_giro") ){//si recibe por MQTT el comando "d_giro"
		//Serial.print ("auto");
		cmdFrame[0] = DIST_GIRO;
		cmdFrame[3] = '\0';
		Serial.write (cmdFrame, 4);
		iniciaInstruccion(DIST_GIRO);
		return;
	}
	
	if ( !strcmp(pl, "home") ){//si recibe por MQTT el comando "home"
		cmdFrame[0] = HOME;
		cmdFrame[3] = '\0';
		cmdEnProceso = 1;
		Serial.write (cmdFrame, 4);
		iniciaInstruccion (HOME);
		return;
	}
	
	if ( !strcmp(pl, "set_home") ){//si recibe por MQTT el comando "set_home"
		cmdFrame[0] = SET_HOME;
		cmdFrame[3] = '\0';
		cmdEnProceso = 1;
		Serial.write (cmdFrame, 4);
		iniciaInstruccion (SET_HOME);
		return;
	}

} //fin callback()



void connections_handler() {
    
    switch(conn_status){
        
        case TRYING_WIFI:
            // We start by connecting to a WiFi network
            Serial.println();
            Serial.print("Connecting to ");
            Serial.println(ssid);

            WiFi.mode(WIFI_STA);
            if (WiFi.begin(ssid, password) != WL_CONNECTED){
                Serial.print(".");
                conn_status = WIFI_TIMING_OUT;
                reconnect_time = 500;
            }else{
                Serial.println("");
                Serial.println("WiFi connected");
                Serial.println("IP address: ");
                Serial.println(WiFi.localIP());
                reconnect_time = 0;
                conn_status = TRYING_MQTT;
            }
        break;
        
        case WIFI_TIMING_OUT:
            
            if (WiFi.status() != WL_CONNECTED){            
                if (!reconnect_time){
                    conn_status = TRYING_WIFI;
                }
            }else{
                Serial.println("");
                Serial.println("WiFi connected");
                Serial.println("IP address: ");
                Serial.println(WiFi.localIP());
                reconnect_time = 0;
                conn_status = TRYING_MQTT;
            }
                
        break;
        
        case TRYING_MQTT:
            
            if (WiFi.status() != WL_CONNECTED){
                Serial.println("Wifi connection lost. restarting...");
                conn_status = TRYING_WIFI;
                break;
            }
            
            
            Serial.print("Attempting MQTT connection...");
            // Create a random client ID
            //clientId = "Nodo_ventilador";
            clientId += String(random(0xffff), HEX); //must be random because of the uMQTTbroker bug
            
            // Attempt to connect
            if (client.connect(clientId.c_str())) {
                Serial.println("connected");
                // Once connected, publish an announcement...
                client.publish("Hola", "Nodo_ESP01");
                // ... and resubscribe
                client.subscribe("Cmd/Nodo_ESP01/#");
                
                conn_status = ALL_CONNECTED;
                
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
                // Wait 5 seconds before retrying
                
                conn_status = MQTT_TIMING_OUT;
                reconnect_time = 500;
            }
        break;
        
        case MQTT_TIMING_OUT:
            
            if (WiFi.status() != WL_CONNECTED){
                Serial.println("Wifi connection lost. restarting...");
                conn_status = TRYING_WIFI;
                break;
            }
            
            if (!reconnect_time){
                conn_status =TRYING_MQTT;
            }
        break;
        
        case ALL_CONNECTED:
            if (WiFi.status() != WL_CONNECTED){
                Serial.println("Wifi connection lost. restarting...");
                conn_status = TRYING_WIFI;
                break;
            }
            
            if (client.connected()){
                client.loop();
            }else{
                conn_status = TRYING_MQTT;
            }
        break;        
        
    }//end switch
}//end connections_handler ()


void serialCom_handler(void){
	sizeCmd = Serial.readBytes(rxUart, RXUART_BUFFER_SIZE);
	//Serial.println(rxUart);
	
	//controlRxTxUART (rxUart);
	
	//if (rxUart == "hola"){
	//if (!strcmp(rxUart, "hola")){
	if(rxUart[0] == HOLA){
		client.publish("Info/Nodo_ESP01", "STM32 dice hola");
		client.flush();
		return;
	}
	
	if (!strcmp(rxUart, "auto")){
		client.publish("Info/Nodo_ESP01", "el robot está en AUTO");
		client.flush();
		return;
	}
	
	if (!strcmp(rxUart, "manu")){
		client.publish("Info/Nodo_ESP01", "el robot está en MANUAL");
		client.flush();
		return;
	}
	
	//VEL_AVANCE
	if (rxUart[0] == VEL_AVANCE){
		sprintf(texto, "%d", (uint16_t) ( (rxUart[1] << 8) |  rxUart[2]) );
		client.publish("Info/Nodo_ESP01/VEL_AVANCE", texto);
		client.flush();
		return;
	}
	
	//COORD_ANG
	if (rxUart[0] == DIST_GIRO){
		sprintf(texto, "%d", (int16_t) ( (rxUart[1] << 8) |  rxUart[2]) );
		client.publish("Info/Nodo_ESP01/COORD_ANG", texto);
		client.flush();
		return;
	}
	
		
	for (uint8_t i = 0; i < RXUART_BUFFER_SIZE; i++){
		rxUart[i] = '\0';
	}
	
} //end serialCom_handler()


void setup() {
	
	Serial.begin(115200);
    randomSeed(micros());
    
    //Serial.println("algo");
    
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback_MQTT);
    
	connections_handler();
	
	init_controlRxTx (txtTopic, texto, cmdFrame);
	
	cmdFrame[0] = HOLA;
	cmdFrame[3] = '\0';
	Serial.write (cmdFrame, 4);
    
}

void loop() {
	
	timer_update();
	
	if (Serial.available() > 0){
		//serialCom_handler();
		sizeCmd = Serial.readBytes(rxUart, RXUART_BUFFER_SIZE);
		switch (controlRxTxUART (rxUart) ){
			case SEND_MQTT:
				client.publish(txtTopic, texto);
				client.flush();
			break;
			case SEND_TXUART:
				Serial.write (cmdFrame, 4);
			break;
			case SEND_BOTH:
				Serial.write (cmdFrame, 4);
				client.publish(txtTopic, texto);
				client.flush();
			default:
			break;
		} //end switch controlRxTxUART
	} //end if Serial.available
	
	if (flag_tick){
		if (reconnect_time) reconnect_time--;
		
		flag_tick = 0;		
	} //end if flag_tick
	
	connections_handler();
	
}
