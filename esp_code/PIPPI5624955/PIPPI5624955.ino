/**************************5624955 PIPPI**************************************/

//importiamo le librerie necessarie
#include "painlessMesh.h"
#include "math.h"
//definiamo i parametri della rete
#define MESH_PREFIX   "MeshNetwork"
#define MESH_PASSWORD "password123"  // minimo 8 caratteri
#define MESH_PORT     5555

Scheduler userScheduler;
painlessMesh mesh;

//pin del sensore di prossimità
#define trigPin D6
#define echoPin D7

//variabili che conservano la posizione stimata
float x_st=8.5;
float y_st=8.5;

//POSIZIONE  E NOME DEL NODO
String nome="Pippi";
float x=0;
float y=17;
//Nodo che si chiama pippi parla solo con 2 vicini: vuoto e alessia
//di ID: 3958302360 (vuoto) 5702239 (alessia)
uint32_t vicino1=3958302360;
uint32_t vicino2=5702239;

//**********
float durata, cm;
bool led=false;

//carica della batteria 
int energia=200;

//strutture dati di appoggio per contenere quanto ricevuto dai vicini
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
//funzione ausiliaria per la gestione delle strutture precedenti
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

// Task: ogni secondo effettua una misura
Task taskMisura(TASK_SECOND * 1, TASK_FOREVER, []() {
    //ogni volta che misuriamo lampeggiamo il led
    if (energia > 0){
        digitalWrite(LED_BUILTIN, led);
            led=!led; 
        }else{
            digitalWrite(LED_BUILTIN, LOW);
        }

    //effettuiamo la misura vera e propria
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

    //ottenuta la misura verifico se ci sia l'intruso
    if(cm<=20){
        //incaso rilevo qualcosa, invio la mia stima ai vicini
        String msg ="1-"+String(x_st)+"-"+String(y_st);
        mesh.sendBroadcast(msg);
        //avendo effettuato una trasmissione decremento l'energia
        energia--;
        Serial.print("RILEVATO invio e decremento");
        Serial.println (energia);
    }
});


//ogni qual volta ricevo un messaggio dai vicini eseguo tale funzione
//la quale esegue l'algoritmo mostrato nei paragrafi precedenti
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

    //come gia mostrato se io vedo il nodo allora eseguo l'algoritmo descritto
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
        //ma se ricevo messaggi ma non vedo allora faccio banalmente una media
        x_st = x_medio;
        y_st = y_medio;
        Serial.println("Non vedo");
    }


    //stampa di debug serve solo per lo sniffer per fare il grafico
    String msg = "2-"+String(x_st)+"-"+String(y_st);
    mesh.sendBroadcast(msg);

    Serial.print("nuova stima calcolata: ");
    Serial.print(x_st);
    Serial.print(",");
    Serial.println(y_st);
}


//callBack eseguita quando si riceve un messaggio
void receivedCallback(uint32_t from, String &msg) {
    //si estrae il messaggio
    //Serial.printf("[RX] Da %u: %s| ", from, msg.c_str());
    int sep1 = msg.indexOf('-');
    int sep2 = msg.indexOf('-', sep1 + 1);

    float type = msg.substring(0, sep1).toFloat();
    float a = msg.substring(sep1 + 1, sep2).toFloat();
    float b = msg.substring(sep2 + 1).toFloat();

    //se è un messaggio dell'algoritmo e non di debug allora chiamiamo la funzione precedente
    //ovviamente solo se abbiamo ancora energia
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

