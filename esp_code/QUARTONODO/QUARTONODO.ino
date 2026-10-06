/************************** 3958302360 VUOTO**************************************/


#include "painlessMesh.h"
#include "math.h"

#define MESH_PREFIX   "MeshNetwork"
#define MESH_PASSWORD "password123"  // minimo 8 caratteri
#define MESH_PORT     5555

Scheduler userScheduler;
painlessMesh mesh;

#define trigPin D6
#define echoPin D7

#define nodi 4

String nome="Vuoto";
float x_st=8.5;
float y_st=8.5;



float x=0;
float y=0;

//Nodo che si chiama vuoto parla solo con 2 vicini: pippi e aldo
//di ID: 5624955 (pippi) 5744379 (aldo)
uint32_t vicino1=5624955;
uint32_t vicino2=5744379;

//**********
float durata, cm;

int energia=200;
bool led=false;

struct stima_vicino {
    uint32_t id;
    float x;
    float y;
};

stima_vicino st_vicini[3] = {
    {0, 0.0, 0.0},   // 0 = slot libero
    {0, 0.0, 0.0},
    {0, 0.0, 0.0}
};

void inserisci(uint32_t id, float x, float y) {
    // 1) Cerca se l'ID esiste già
    for (int i = 0; i < 3; i++) {
        if (st_vicini[i].id == id) {
            st_vicini[i].x = x;   // aggiorna
            st_vicini[i].y = y; 
            return;
        }
    }

    // 2) Altrimenti aggiungi in coda (primo slot libero)
    for (int i = 0; i < 3; i++) {
        if (st_vicini[i].id == 0) {         // slot libero
            st_vicini[i].id = id;
            st_vicini[i].x = x;
            st_vicini[i].y = y; 
            return;
        }
    }

    // 3) Se arrivi qui, il vettore è pieno
    Serial.println("Vettore pieno, impossibile inserire");
}

// Task: ogni secondo misura
Task taskMisura(TASK_SECOND * 1, TASK_FOREVER, []() {

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    durata = pulseIn(echoPin, HIGH);
    cm = durata / 58; 
    Serial.print("Cm = ");
    Serial.print(cm);
    Serial.print("Mia stima: ");
    Serial.print(x_st);
    Serial.print(", ");
    Serial.println(y_st);
    if (energia > 0){
       digitalWrite(LED_BUILTIN, led);
        led=!led; 
    }else{
        digitalWrite(LED_BUILTIN, LOW);
    }
    

    if(cm<=20){
        String msg ="1-"+String(x_st)+"-"+String(y_st);
        mesh.sendBroadcast(msg);
        energia--;
        Serial.print("RILEVATO invio e decremento");
        Serial.println (energia);
    }
});


void stimadistribuita(){  
    float x_medio;
    float y_medio;

    int n=1;


    x_medio=x_st;
    y_medio=y_st;

    for(int i=0; i<3; i++){
        if(st_vicini[i].id==vicino1 || st_vicini[i].id==vicino2){
            x_medio+=st_vicini[i].x;
            y_medio+=st_vicini[i].y;
            n++;
        }
    }

    x_medio=x_medio/n;
    y_medio=y_medio/n;

    if (cm<=20){
        float rx = x_medio-x;
        float ry = y_medio-y;
        float r=sqrt(rx*rx+ry*ry);
        float error=r-cm;

        float ux=rx/r;
        float uy=ry/r;

        x_st = x_medio - 0.2 * error * ux;
        y_st = y_medio - 0.2 * error * uy;
        Serial.println("Vedo");
    }else{
        x_st = x_medio;
        y_st = y_medio;
        Serial.println("Non vedo");
    }

    /*
    }else{
        float temp1x=0, temp2x=0, temp1y=0, temp2y=0;
        for(int i=0; i<3; i++){
            if(st_vicini[i].id==vicino1){
                temp1x=x_st-st_vicini[i].x;
                temp1y=y_st-st_vicini[i].y;
            }
            if(st_vicini[i].id==vicino2){
                temp2x=x_st-st_vicini[i].x;
                temp2y=y_st-st_vicini[i].y;
            }
        }
        x_st = x_st -0.5*1*temp1x -0.5*1*temp2x;
        y_st = y_st - 0.5*1*temp1y-0.5*1*temp2y;


    }
    */

    String msg = "2-"+String(x_st)+"-"+String(y_st);
    mesh.sendBroadcast(msg);

    Serial.print("nuova stima calcolata: ");
    Serial.print(x_st);
    Serial.print(",");
    Serial.println(y_st);
}


void receivedCallback(uint32_t from, String &msg) {
    //Serial.printf("[RX] Da %u: %s| ", from, msg.c_str());
    int sep1 = msg.indexOf('-');
    int sep2 = msg.indexOf('-', sep1 + 1);

    float type = msg.substring(0, sep1).toFloat();
    float a = msg.substring(sep1 + 1, sep2).toFloat();
    float b = msg.substring(sep2 + 1).toFloat();

    

    //Serial.print(a);
    //Serial.print(",");
    //Serial.println(b);
    if(type==1 && energia>0){
        inserisci(from,a,b);
        stimadistribuita();
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

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    delay(2000);
    userScheduler.addTask(taskMisura);
    taskMisura.enable();

    Serial.printf("\n=== NODO AVVIATO ===\n");
    Serial.printf("ID:      %u\n", mesh.getNodeId());
    Serial.printf("NOME:  %s\n",nome);
    Serial.print("Sono in angolo: ");
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
    Serial.println("===================\n");
}

void loop() {
    mesh.update();
    if(energia<=0){
        Serial.print("scaricato");
        Serial.println (energia);
            digitalWrite(LED_BUILTIN, 1);
            taskMisura.disable();
    }
}
