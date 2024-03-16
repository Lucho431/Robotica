#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "comandosUart.h"
#include "comunicacionUART_ESP01.h"

#define RXUART_BUFFER_SIZE 8

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
String clientId;
unsigned long lastMsg = 0;

uint8_t sizeCmd = 0;

char rxUart [RXUART_BUFFER_SIZE];
uint8_t cmdUart [3][8];
uint8_t index_uartIn = 0;
uint8_t index_uartOut = 0;
uint8_t cola_uart = 0;

uint8_t cmdMqtt [3][8];
uint8_t index_mqttIn = 0;
uint8_t index_mqttOut = 0;
uint8_t cola_mqtt = 0;

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
	
	if (cola_mqtt < 3){
		cmdMqtt [index_mqttIn] [0] = payload [0];
		cmdMqtt [index_mqttIn] [1] = payload [1];
		cmdMqtt [index_mqttIn] [2] = payload [2];
		cmdMqtt [index_mqttIn] [3] = payload [3];
		cmdMqtt [index_mqttIn] [4] = payload [4];
		cmdMqtt [index_mqttIn] [5] = payload [5];
		cmdMqtt [index_mqttIn] [6] = payload [6];
		cmdMqtt [index_mqttIn] [7] = payload [7];
		if (index_mqttIn < 2){
			index_mqttIn++;
		}else{
			index_mqttIn = 0;
		} //end if index_mqttIn
		cola_mqtt++;
	} //end if cola_mqtt
	
	//Serial.write (cmdUart, 8);
	return;

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

void setup() {
	
	Serial.begin(115200);
	Serial.setTimeout(100);
    randomSeed(micros());
    
    //Serial.println("algo");
    
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback_MQTT);
    
	connections_handler();
	
	//init_controlRxTx (txtTopic, texto, cmdUart);
	
	cmdUart[0][0] = HOLA;
	cmdUart[0][7] = '\0';
	Serial.write (&cmdUart[0][0], 8);
    
}

void loop() {
	
	timer_update();
	
	if (Serial.available() > 0){
		
		sizeCmd = Serial.readBytes(rxUart, RXUART_BUFFER_SIZE);
		if (sizeCmd == 8){
			if (cola_uart < 3){
				cmdUart [index_uartIn] [0] = rxUart [0];
				cmdUart [index_uartIn] [1] = rxUart [1];
				cmdUart [index_uartIn] [2] = rxUart [2];
				cmdUart [index_uartIn] [3] = rxUart [3];
				cmdUart [index_uartIn] [4] = rxUart [4];
				cmdUart [index_uartIn] [5] = rxUart [5];
				cmdUart [index_uartIn] [6] = rxUart [6];
				cmdUart [index_uartIn] [7] = rxUart [7];
				if (index_uartIn < 2){
					index_uartIn++;
				}else{
					index_uartIn = 0;
				} //end if index_uartIn
				cola_uart++;
			} //end if cola_uart
		} //end if (sizeCmd == 8)
	} //end if Serial.available
	
	if (cola_uart != 0){
		sprintf(txtTopic, "Info/Nodo_ESP01/INFO_MSG");
		sprintf(texto, "%c%c%c%c%c%c", cmdUart [index_uartOut][1], cmdUart [index_uartOut][2], cmdUart [index_uartOut][3], cmdUart [index_uartOut][4], cmdUart [index_uartOut][5], cmdUart [index_uartOut][6]);
		client.publish(txtTopic, texto);
		client.flush();
		if (index_uartOut < 2){
			index_uartOut++;
		}else{
			index_uartOut = 0;
		} //end if index_uartOut
		cola_uart--;
	} //end if cola_uart
	
	if (cola_mqtt != 0){
		Serial.write( &cmdMqtt[index_mqttOut][0], 8);
		if (index_mqttOut < 2){
			index_mqttOut++;
		}else{
			index_mqttOut = 0;
		} //end if index_mqttOut
		cola_mqtt--;
	} //end if cola_mqtt
	
	
	if (flag_tick){
		if (reconnect_time != 0) reconnect_time--;
		flag_tick = 0;		
	} //end if flag_tick
	
	connections_handler();
	
}
