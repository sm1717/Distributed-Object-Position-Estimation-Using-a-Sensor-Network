/**************************2771019597  SNIFFERLAURA**************************************/


#include "painlessMesh.h"
#include "math.h"

#define MESH_PREFIX   "MeshNetwork"
#define MESH_PASSWORD "password123"  // minimo 8 caratteri
#define MESH_PORT     5555

Scheduler userScheduler;
painlessMesh mesh;


#define nodi 4

float sxu_x;
float sxu_y;

float sxl_x;
float sxl_y;

float dxu_x;
float dxu_y;

float dxl_x;
float dxl_y;


String nome="SNIFFER-Laura";
uint32_t pippi=5624955;
uint32_t vuoto=3958302360;
uint32_t aldo= 5744379 ;
uint32_t alessia= 5702239;


void print(uint32_t from,float a,float b){
   
    if(from==pippi){
        sxu_x=a;
        sxu_y=b;
    }
    if(from==vuoto){
        sxl_x=a;
        sxl_y=b;
    }
    if(from==aldo){
        dxl_x=a;
        dxl_y=b;
    }
    if(from==alessia){
        dxu_x=a;
        dxu_y=b;
    }

    // Invia tutto su una riga CSV
    Serial.print(sxu_x, 3); Serial.print(",");
    Serial.print(sxu_y, 3); Serial.print(",");
    Serial.print(sxl_x, 3); Serial.print(",");
    Serial.print(sxl_y, 3); Serial.print(",");
    Serial.print(dxu_x, 3); Serial.print(",");
    Serial.print(dxu_y, 3); Serial.print(",");
    Serial.print(dxl_x, 3); Serial.print(",");
    Serial.println(dxl_y, 3);  // println aggiunge \n
}



void receivedCallback(uint32_t from, String &msg) {
    //Serial.printf("[RX] Da %u: %s| ", from, msg.c_str());
    int sep1 = msg.indexOf('-');
    int sep2 = msg.indexOf('-', sep1 + 1);

    float type = msg.substring(0, sep1).toFloat();
    float a = msg.substring(sep1 + 1, sep2).toFloat();
    float b = msg.substring(sep2 + 1).toFloat();

    
    if(type==2){
        print(from,a,b);
    }
    
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("[+] Nuovo nodo connesso: %u\n", nodeId);
    Serial.printf("    Nodi in rete: %d\n", mesh.getNodeList().size() + 1);
}

void changedConnectionCallback() {
    Serial.printf("[~] Topologia cambiata: %s\n", mesh.subConnectionJson().c_str());
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("[T] Clock sincronizzato, offset: %d us\n", offset);
}

void setup() {
    Serial.begin(115200);
    delay(200);

    //mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, 6);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);



    Serial.printf("\n=== NODO AVVIATO ===\n");
    Serial.printf("ID:      %u\n", mesh.getNodeId());
    Serial.printf("NOME:  %s\n",nome);
    Serial.println("===================\n");
}

void loop() {
    mesh.update();
}