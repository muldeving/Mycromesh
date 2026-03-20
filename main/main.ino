#include "LiteLora.h"
#include <limits.h>
#include <ESP32Time.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include <SD.h>
#include <time.h>
#include "Adafruit_BME680.h"
#include <Preferences.h>
#include <Update.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHT10.h>
#include <NimBLEDevice.h>
#include <OneWire.h>
#include <DallasTemperature.h>

Adafruit_BMP280 bmp;
Adafruit_AHT10 aht;

Preferences prefs;

ESP32Time rtc;

LiteLora lora;

const String FIRMWARE_VERSION = "1.3.0";

#define uS_TO_S_FACTOR 1000000

#define PACKET_SIZE 180
#define GROUP_K 32
#define PARITY_M 6
#define CHUNK_SIZE 90  // 180/90=2 chunks exacts par paquet, laisse place au CRC16

#define UPDATE_FILE "/update/firmware.bin"
#define BUF_SIZE 4096

#define CPU_FREQ_IDLE   20
#define CPU_FREQ_BLE    80   // fréquence minimale requise par le stack BLE sur ESP32-C3
#define CPU_FREQ_TURBO  160

static uint8_t gf_exp[512];
static uint8_t gf_log_tbl[256];

Adafruit_BME680 bme;

const int oneWireBus = 3; 
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
// Variable Cron

String crontabString;

// Variable de parametres

long MAX_PING_AGE = 20000;      // Durée maximale (en millisecondes) avant traitement d'une entrée — stocké en ms, p.cfg en secondes
long starttimeout = 15000;      // Durée timout procudure start — stocké en ms, p.cfg en secondes
int localAddress = 61;          // address of this device
long DELAY = 3600000;           // Délai en millisecondes — stocké en ms, p.cfg en secondes
long MAX_ENTRY_AGE = 20000;     // Durée maximale (en millisecondes) avant traitement d'une entrée — stocké en ms, p.cfg en secondes
long actiontimerdel = 30;
bool maintmode = true;
int stationgateway = 1;
long TOGATE_COMMAND_TIMEOUT = 60000;  // stocké en ms, p.cfg en secondes
long NETIO_TIMEOUT     = 300000;      // ms sans activité avant fermeture automatique du tunnel — stocké en ms, p.cfg en secondes
int filetimeout = 300;
int filetxtimeout = 60;

// Niveaux de sortie série : 0=rien, 1=normal, 2=verbose, 3=debug
#define LOG_NONE    0
#define LOG_NORMAL  1
#define LOG_VERBOSE 2
#define LOG_DEBUG   3
int serialLevel = 1;

// Mode E/S : sélectionne le canal actif pour ioOutput / handleSerialInput
#define IO_USB       0
#define IO_BLUETOOTH 1
#define IO_NETIO     2   // tunnel réseau : entrée/sortie via une autre station du maillage
int ioMode = IO_USB;

// Variables du tunnel réseau E/S (mode IO_NETIO)
bool          netioMaster       = false;   // cette station est le maître du tunnel
bool          netioSlave        = false;   // cette station est l'esclave du tunnel
int           netioRemote       = -1;      // adresse de la station distante
unsigned long netioLastActivity = 0;       // horodatage de la dernière activité (millis)
int           ntioPrevIoMode    = IO_USB;  // mode E/S sauvegardé avant activation du tunnel
bool          ntioPrevMaintMode = true;    // état maintmode sauvegardé avant activation du tunnel

// variables d'état

int startstat = 0;
byte msgCount = 0;            // count of outgoing messages

// variables d'environnement

const int TAILLE_TAMPON = 256;
const int MAX_EDGES = 300;      // nombre max de liaisons
const int MAX_VERTICES = 60;    // nombre max de stations
const int MAX_ENTRIES = 10;     // Nombre maximum d'entrées
const int MAX_PINGS = 10;       // Nombre maximum d'entrées
const int MAX_SIZE = 10;        // Taille maximale du tableau de stockage
const int MAX_TOGATE_COMMANDS = 10;
const int MAX_TOGATE_COMMANDS_FILE = 20;

const int csPin = 21;          // LoRa radio chip select
const int irqPin = 8;          // change for your board; must be a hardware interrupt pin

// variables systeme

int remof;
String ftfpath;
bool isfdeson = false;
bool infilecache = false;
int filetxdelai;
int filedelai;
int filesender = -1;
int filereceivientstation = -1;
unsigned long ltgdel = 0;
bool sensorstart = false;
bool sensor_bme680 = false;
bool sensor_aht20 = false;
bool sensor_bmp280 = false;
bool sensor_ds18b20 = false;
int lastMinuteChecked = -1;
int rtmapdel = 0, fpingdel = 0;
unsigned long starttime;
int starttry = 0;
int nearestforstart;
unsigned long timegeth = 0;
int pingCount = 0; // Nombre actuel d'entrées dans la liste
int dataCount = 0;        // Compteur de valeurs stockées
unsigned long lastAddTime = 0; // Temps de la dernière addition
int sender;
int pingphase = 0;
unsigned long tmps = 0;
int numEdges = 0;
int numVertices = 0;
unsigned long actiontimer = 0;
unsigned long sleeptimer = 0;
int stationstat = 0;
const int MAX_COMMANDS = 10;
int nbtogatefail = 0;
unsigned long ltgdelfile = 0;
int nbtogatefailfile = 0;
int togateCountFile = 0;
unsigned long lastair;
int rxglob = 0;
int rxbrd = 0;
int rxloc = 0;
int dijktx = 0;
int dijkrx = 0;
int dijkhop = 0;
int resend1 = 0;
int resend2 = 0;
int txcnt = 0;

// Variables diffusion réseau (broadcast fichier)
bool broadcastMode = false;         // Station en mode réception diffusion
bool broadcastEmitter = false;      // Cette station est l'émettrice
String broadcastPath = "";          // Chemin du fichier en cours de diffusion
unsigned long lastBroadcastRecv = 0;// Horodatage dernière activité (secondes)
int lastBrdfSeq = -1;              // Dernier numéro de séquence brdf traité
bool broadcastFileSending = false;  // Émettrice : envoi de lignes en cours
unsigned long lastBrdfSendMs = 0;  // Dernier envoi brdf (millis)
int brdfSeqCounter = 0;            // Compteur de séquence brdf

// Tableaux

struct DelayedCommand {
  unsigned long triggerTime;
  String command;
  bool active;
};
DelayedCommand commandQueue[MAX_COMMANDS];

struct TogateCommand {
  int id;
  String command;
  unsigned long timestamp;
};

TogateCommand togateQueue[MAX_TOGATE_COMMANDS];
int togateCount = 0;

struct TogateCommandFile {
  int id;
  String command;
  unsigned long timestamp;
};

TogateCommandFile togateQueueFile[MAX_TOGATE_COMMANDS_FILE];

struct Data {
  String value;        // La valeur de type String stockée
  unsigned long time;  // Le temps d'enregistrement de la valeur
};

Data dataArray[MAX_SIZE]; // Tableau pour stocker les valeurs

struct Edge {
  int vertex1;
  int vertex2;
  int weight;
};


Edge edges[MAX_EDGES];
int vertices[MAX_VERTICES];

// Configuration du tableau
struct Entry {
  String id;
  unsigned long timesend;
  int nbtrysend;
  String msgtosend;
};
Entry entryList[MAX_ENTRIES];
int entryCount = 0; // Nombre actuel d'entrées dans la liste


struct PingEntry {
  int nbtoping;
  unsigned long pingTime;
  int nbping;
};
PingEntry pingList[MAX_PINGS];


// --- BLE UART (Nordic UART Service) ---
#define BLE_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer*         bleServer    = nullptr;
NimBLECharacteristic* bleTxChar    = nullptr;
bool               bleConnected = false;
String             bleRxBuffer  = "";
bool               bleRxReady   = false;

class BLEServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&)    override { bleConnected = true;  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo&, int) override {
    bleConnected = false;
    s->startAdvertising(); // relance la publicité pour permettre une reconnexion
  }
};

class BLERxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    bleRxBuffer = c->getValue().c_str();
    bleRxReady  = true;
  }
};

void startBLE() {
  cpuTurbo();
  if (bleServer) return; // déjà démarré
  NimBLEDevice::init(("Mycromesh-" + String(localAddress)).c_str());
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BLEServerCB());

  NimBLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  bleTxChar = svc->createCharacteristic(BLE_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  // NimBLE gère automatiquement le descripteur CCCD (BLE2902), pas besoin de l'ajouter manuellement

  NimBLECharacteristic* rxChar = svc->createCharacteristic(BLE_RX_UUID, NIMBLE_PROPERTY::WRITE);
  rxChar->setCallbacks(new BLERxCB());

  svc->start();

  // Configure l'advertising : UUID du service NUS dans le paquet principal,
  // nom du périphérique dans la scan response séparée.
  // Le nom ne peut pas être dans le paquet principal : flags(3) + UUID128(18) + nom(14) = 35 > 31 octets.
  // enableScanResponse(true) seul ne suffit pas en NimBLE v2.x — il faut passer les données explicitement.
  NimBLEAdvertisementData scanResponse;
  scanResponse.setName(("Mycromesh-" + String(localAddress)).c_str());

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponseData(scanResponse);
  adv->start();
}

void stopBLE() {
  if (!bleServer) return;
  bleServer->getAdvertising()->stop();
  NimBLEDevice::deinit(true);
  bleServer    = nullptr;
  bleTxChar    = nullptr;
  bleConnected = false;
  bleRxReady   = false;
  bleRxBuffer  = "";
}
// --------------------------------------

// --- Tunnel réseau E/S (netio) ---

// Affiche un message sur le canal physique précédent (USB ou BLE) du maître,
// utilisé pour afficher les réponses qui arrivent de l'esclave.
void netioDisplay(const String& msg) {
  switch (ntioPrevIoMode) {
    case IO_USB:
      Serial.println(msg);
      break;
    case IO_BLUETOOTH:
      if (bleConnected && bleTxChar) {
        String line = msg + "\n";
        bleTxChar->setValue(line.c_str());
        bleTxChar->notify();
      }
      break;
  }
}

// Ferme le tunnel réseau E/S et restaure l'état précédent des deux côtés.
void closeNetioTunnel() {
  netioMaster       = false;
  netioSlave        = false;
  ioMode            = ntioPrevIoMode;
  maintmode         = ntioPrevMaintMode;
  netioRemote       = -1;
  netioLastActivity = 0;
}
// ---------------------------------

// --- Couche d'abstraction E/S ---
// Centralise toutes les sorties texte du firmware.
// Pour ajouter un canal, incrémenter IO_* et ajouter un case ici.
void ioOutput(const String& msg) {
  switch (ioMode) {
    case IO_USB:
      Serial.println(msg);
      break;
    case IO_BLUETOOTH:
      if (bleConnected && bleTxChar) {
        String line = msg + "\n";
        bleTxChar->setValue(line.c_str());
        bleTxChar->notify();
      }
      break;
    case IO_NETIO:
      if (netioSlave && netioRemote >= 0) {
        // Esclave : transmet la sortie au maître via le réseau maillé (send)
        String rsp = "send:";
        rsp += netioRemote;
        rsp += ":ntirsp:";
        rsp += msg;
        interpreter(rsp);
        netioLastActivity = millis();
      }
      // Maître : supprime sa propre sortie — seules les réponses de l'esclave sont affichées
      break;
  }
}

// Lit l'entrée du canal actif et envoie la commande à l'interpréteur.
// Pour ajouter un canal, ajouter un case ici.
void handleSerialInput() {
  switch (ioMode) {
    case IO_USB:
      if (Serial.available() > 0) { String _s = Serial.readString(); _s.trim(); interpreter(_s); delay(100); }
      break;
    case IO_BLUETOOTH:
      if (bleRxReady) {
        bleRxReady = false;
        String cmd = bleRxBuffer;
        bleRxBuffer = "";
        interpreter(cmd);
        delay(100);
      }
      break;
    case IO_NETIO:
      if (netioMaster) {
        // Le maître lit depuis son canal physique précédent et transmet à l'esclave
        String inputCmd = "";
        bool hasInput = false;
        if (ntioPrevIoMode == IO_USB) {
          if (Serial.available() > 0) { inputCmd = Serial.readString(); hasInput = true; }
        } else if (ntioPrevIoMode == IO_BLUETOOTH) {
          if (bleRxReady) {
            bleRxReady = false;
            inputCmd = bleRxBuffer;
            bleRxBuffer = "";
            hasInput = true;
          }
        }
        if (hasInput) {
          inputCmd.trim();
          if (inputCmd.length() > 0) {
            if (inputCmd == "exit") {
              // Fermeture propre du tunnel : signaler l'esclave puis restaurer l'état
              int savedRemote = netioRemote;
              closeNetioTunnel();
              interpreter("trsp:" + String(savedRemote) + ":nticlose");
              ioOutput("[NETIO] Tunnel fermé proprement");
            } else {
              // Transmettre la commande à l'esclave via le réseau maillé (send)
              netioLastActivity = millis();
              interpreter("send:" + String(netioRemote) + ":ntidata:" + inputCmd);
            }
          }
        }
      }
      // L'esclave ne lit pas d'entrée locale : ses commandes arrivent via ntidata réseau
      break;
  }
}
// --------------------------------

void logN(const String& msg) { if(serialLevel >= LOG_NORMAL)  ioOutput(msg); }
void logV(const String& msg) { if(serialLevel >= LOG_VERBOSE) ioOutput(msg); }
void logD(const String& msg) { if(serialLevel >= LOG_DEBUG)   ioOutput(msg); }

void loraToSD() {
    lora.releaseBus();
    digitalWrite(20, 0);
    delay(100);
    if (!SD.begin(7)) { logN("[ERREUR] Carte SD introuvable ou non initialisée"); }
}

void sdToLora() {
    digitalWrite(20, 1);
    lora.acquireBus();
}

bool togateAddCommand(int id, String command) {
  if (togateCount >= MAX_TOGATE_COMMANDS) {
    logN("[ERREUR] File de commandes gateway pleine, commande ignorée");
    return false;
  }

  togateQueue[togateCount].id = id;
  togateQueue[togateCount].command = command;
  togateQueue[togateCount].timestamp = millis();
  logD("togate:add id=" + String(id));
  togateCount++;
  return true;
}

// Supprimer les commandes plus vieilles qu'une minute
void togatePurgeOld() {
  bool purged = false;

  for (int i = 0; i < togateCount; ) {
    if ((millis() - togateQueue[i].timestamp) > TOGATE_COMMAND_TIMEOUT) {
      logD("togate:expire id=" + String(togateQueue[i].id));
      prefs.putBool("incache", true);
      nbtogatefail ++;
      if (nbtogatefail >= 5 && prefs.getBool("isgateonline", 0) == 1){        
        prefs.putBool("isgateonline", false);
      }

      delay(50);
      loraToSD();
      File myFile;
      myFile = SD.open("/togate.cache", FILE_APPEND);   
      if (myFile) {
        myFile.println(togateQueue[i].command);
        myFile.close();
      }
      else{
        logN("[ERREUR SD] Impossible d'écrire dans /togate.cache");
      }

      sdToLora();

      // Supprimer cette entrée en décalant les suivantes
      for (int j = i; j < togateCount - 1; j++) {
        togateQueue[j] = togateQueue[j + 1];
      }
      togateCount--;
      purged = true;
    } else {
      i++;
    }
  }
}

// Supprimer une commande par son ID
bool togateRemoveById(int id) {
  for (int i = 0; i < togateCount; i++) {
    if (togateQueue[i].id == id) {
      logD("togate:rm id=" + String(id));
      prefs.putBool("isgateonline", true);
      ltgdel = (millis()/1000);
      nbtogatefail = 0;
      // Décaler les éléments suivants
      for (int j = i; j < togateCount - 1; j++) {
        togateQueue[j] = togateQueue[j + 1];
      }
      togateCount--;
      return true;
    }
  }

  logD("togate:id " + String(id) + " non trouvé");
  return false;
}

// Ajouter une commande à la file d'export de fichier
void togateAddCommandFile(int id, String command) {
  // Vérifier si l'ID existe déjà
  for (int i = 0; i < togateCountFile; i++) {
    if (togateQueueFile[i].id == id) {
      return;  // déjà présent
    }
  }

  if (togateCountFile < MAX_TOGATE_COMMANDS_FILE) {
    togateQueueFile[togateCountFile].id = id;
    togateQueueFile[togateCountFile].command = command;
    togateQueueFile[togateCountFile].timestamp = millis();
    togateCountFile++;
    logD("togatef:add id=" + String(id));
  } else {
    logN("[ERREUR] File de commandes fichier pleine");
  }
}

// Supprimer une commande par son ID (file exportfile)
bool togateRemoveByIdFile(int id) {
  for (int i = 0; i < togateCountFile; i++) {
    if (togateQueueFile[i].id == id) {
      logD("togatef:rm id=" + String(id));
      
      isfdeson = true;
      ltgdelfile = (millis()/1000);
      nbtogatefailfile = 0;
      
      // Décaler les éléments suivants
      for (int j = i; j < togateCountFile - 1; j++) {
        togateQueueFile[j] = togateQueueFile[j + 1];
      }
      togateCountFile--;
      return true;
    }
  }

  logD("togatef:id " + String(id) + " non trouvé");
  return false;
}

// Remettre les commandes expirées dans tx.txt
void purgeToOldFile() {
  unsigned long currentTime = millis();
  bool hasExpired = false;
  String linesToRestore = "";
  
  for (int i = 0; i < togateCountFile; i++) {
    if (currentTime - togateQueueFile[i].timestamp >= TOGATE_COMMAND_TIMEOUT) {
      logD("togatef:expire id=" + String(togateQueueFile[i].id));
      ltgdelfile = (millis()/1000);
      
      linesToRestore += togateQueueFile[i].command;
      linesToRestore += "\n";
      
      nbtogatefailfile++;
      hasExpired = true;
      
      // Décaler les éléments suivants
      for (int j = i; j < togateCountFile - 1; j++) {
        togateQueueFile[j] = togateQueueFile[j + 1];
      }
      togateCountFile--;
      i--; // Revérifier cet index
    }
  }
  
  if (hasExpired && linesToRestore.length() > 0) {
    // Écrire les lignes expirées dans tx.txt
    delay(50);
    loraToSD();
    File myFile = SD.open("/tx.txt", FILE_APPEND);
    if (myFile) {
      myFile.print(linesToRestore);
      myFile.close();
      logD("togatef:restore tx.txt");
    } else {
      logN("[ERREUR SD] Impossible de restaurer les lignes dans /tx.txt");
    }
    sdToLora();
  }
  
  // Si trop d'échecs, marquer la gate comme hors ligne
  if (nbtogatefailfile >= 3 && isfdeson == 1) {
    isfdeson = false;
    logV("ftx:gate hors ligne");
  }
}


// Fonction pour ajouter une nouvelle entrée
void addPingEntry(int nbtoping, int nbping) {
  for (int i = 0; i < pingCount; i++) {
    if (pingList[i].nbtoping == nbtoping) { return; }
  }
  if (pingCount < MAX_PINGS) {
    pingList[pingCount].nbtoping = nbtoping;
    pingList[pingCount].pingTime = millis();
    pingList[pingCount].nbping = nbping;
    pingCount++;
    logD("ping:add " + String(nbtoping));
  } else {
    logD("ping:liste pleine");
  }
}

// Fonction pour surveiller, mettre à jour ou supprimer les entrées trop anciennes
void checkAndRemoveOldPingEntries() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < pingCount; ) {
    if (currentMillis - pingList[i].pingTime > MAX_PING_AGE) {
      if (pingList[i].nbping < 3) {
        // Augmenter nbping et mettre à jour pingTime
        pingList[i].nbping++;
        pingList[i].pingTime = currentMillis;
        logD("ping:retry " + String(pingList[i].nbtoping) + " n=" + String(pingList[i].nbping));
        
        String tempping = "trsm:";
        tempping += pingList[i].nbtoping;
        tempping += ":ping";
        scheduleCommand(300, tempping);
        
        i++; // Passer à l'entrée suivante
      } else {
        logD("ping:rm " + String(pingList[i].nbtoping));
        String outgoingumap = exportEdgesContainingVertex(localAddress);
        addValue(getValue(outgoingumap, ':', 1));
        logD("umap:" + outgoingumap);
        sendMessage(1, outgoingumap, 0);
        for (int j = i; j < pingCount - 1; j++) {
          pingList[j] = pingList[j + 1];
        }
        pingCount--; // Réduire le nombre d'entrées
      }
      return;
    } else {
      i++; // Passer à l'entrée suivante
    }
  }
}

// Fonction pour supprimer une entrée par nbtoping
void removePingEntryByNbtoping(int nbtoping) {
  for (int i = 0; i < pingCount; i++) {
    if (pingList[i].nbtoping == nbtoping) {
      logD("ping:rm " + String(nbtoping));
      for (int j = i; j < pingCount - 1; j++) {
        pingList[j] = pingList[j + 1];
      }
      pingCount--; // Réduire le nombre d'entrées
      return;
    }
  }
}

// Fonction pour ajouter une nouvelle entrée
void addEntry(const String& id, int nbtrysend, const String& msgtosend) {
  if (entryCount < MAX_ENTRIES) {
    entryList[entryCount].id = id;
    entryList[entryCount].timesend = millis();
    entryList[entryCount].nbtrysend = nbtrysend;
    entryList[entryCount].msgtosend = msgtosend;
    entryCount++;
    logD("entry:add id=" + id);
  } else {
    logD("entry:liste pleine");
  }
}

// Fonction pour surveiller, mettre à jour ou supprimer les entrées trop anciennes
void checkAndRemoveOldEntries() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < entryCount; ) {
    if (currentMillis - entryList[i].timesend > MAX_ENTRY_AGE) {
      if (entryList[i].nbtrysend < 3) {
        // Augmenter nbtrysend et mettre à jour timesend
        entryList[i].nbtrysend++;
        entryList[i].timesend = currentMillis;
        if(entryList[i].nbtrysend == 2){
          resend1 ++;
        }
        if(entryList[i].nbtrysend == 3){
          resend2 ++;
        }
        logD("entry:retry id=" + entryList[i].id + " n=" + String(entryList[i].nbtrysend));
        dijkstra(localAddress, getValue(entryList[i].msgtosend, ':', 1).toInt(), entryList[i].msgtosend);
        i++; // Passer à l'entrée suivante
      } else {
        logD("entry:rm id=" + entryList[i].id);
        for (int j = i; j < entryCount - 1; j++) {
          entryList[j] = entryList[j + 1];
        }
        entryCount--; // Réduire le nombre d'entrées
        if(deststilinl(getValue(entryList[i].msgtosend, ':', 1).toInt())){
        int despingverif = nextstep(localAddress, getValue(entryList[i].msgtosend, ':', 1).toInt());
        addPingEntry(despingverif, 1);
        removeEdge(localAddress, despingverif);
        String tempping = "trsm:";
        tempping += despingverif;
        tempping += ":ping";
        scheduleCommand(300, tempping);
        }
      }
      return;
    }       
    else {
      i++; // Passer à l'entrée suivante
    }
  }
}

bool deststilinl(int dest){
    for (int i = 0; i < entryCount; ) {
    if (getValue(entryList[i].msgtosend, ':', 1).toInt() == dest) {
      return false;
    }
    i++; // Passer à l'entrée suivante
  }
  return true;
}

// Fonction pour supprimer une entrée par ID
void removeEntryByID(const String& id) {
  for (int i = 0; i < entryCount; i++) {
    if (entryList[i].id == id) {
      logD("entry:rm id=" + id);
      for (int j = i; j < entryCount - 1; j++) {
        entryList[j] = entryList[j + 1];
      }
      entryCount--; // Réduire le nombre d'entrées
      return;
    }
  }
}

void initGaloisField() {
  uint8_t x = 1;
  for (int i = 0; i < 255; i++) {
    gf_exp[i] = x;
    gf_log_tbl[x] = i;
    x = (x << 1) ^ ((x & 0x80) ? 0x1D : 0);
  }
  for (int i = 255; i < 512; i++) gf_exp[i] = gf_exp[i - 255];
  gf_log_tbl[0] = 0;
}

uint8_t gfMul(uint8_t a, uint8_t b) {
  if (a == 0 || b == 0) return 0;
  int r = gf_log_tbl[a] + gf_log_tbl[b];
  return gf_exp[r % 255];
}

uint8_t gfPow(uint8_t x, int power) {
  if (power == 0) return 1;
  if (x == 0) return 0;
  int r = (gf_log_tbl[x] * power) % 255;
  return gf_exp[r];
}

uint8_t gfInv(uint8_t a) {
  if (a == 0) return 0;
  return gf_exp[(255 - gf_log_tbl[a]) % 255];
}

// CRC16/CCITT-FALSE — polynôme 0x1021, init 0xFFFF
uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

uint32_t crc32_file(File &f) {
  uint32_t crc = 0xFFFFFFFF;
  f.seek(0);
  uint8_t buf[256];
  while (f.available()) {
    int r = f.read(buf, sizeof(buf));
    for (int i = 0; i < r; i++) {
      crc ^= buf[i];
      for (int j = 0; j < 8; j++) crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}

void encodeGroupParity(uint8_t **dataBuf, uint8_t **parityBuf) {
  for (int p = 0; p < PARITY_M; p++) {
    uint8_t alpha = (uint8_t)(p + 1);
    memset(parityBuf[p], 0, PACKET_SIZE);
    for (int i = 0; i < GROUP_K; i++) {
      uint8_t coef = gfPow(alpha, i);
      for (int b = 0; b < PACKET_SIZE; b++) {
        parityBuf[p][b] ^= gfMul(dataBuf[i][b], coef);
      }
    }
  }
}

bool invertMatrixGF(uint8_t *mat, uint8_t *inv, int n) {
  uint8_t *work = (uint8_t*)malloc(n * n);
  if (!work) return false;
  memcpy(work, mat, n * n);

  for (int r = 0; r < n; r++) for (int c = 0; c < n; c++) inv[r*n + c] = (r==c)?1:0;

  for (int col = 0; col < n; col++) {
    int pivot = -1;
    for (int r = col; r < n; r++) if (work[r*n + col] != 0) { pivot = r; break; }
    if (pivot == -1) { free(work); return false; }

    if (pivot != col) {
      for (int c = 0; c < n; c++) {
        uint8_t t = work[col*n + c]; work[col*n + c] = work[pivot*n + c]; work[pivot*n + c] = t;
        uint8_t s = inv[col*n + c]; inv[col*n + c] = inv[pivot*n + c]; inv[pivot*n + c] = s;
      }
    }

    uint8_t pv = work[col*n + col];
    uint8_t pv_inv = gfInv(pv);

    for (int c = 0; c < n; c++) work[col*n + c] = gfMul(work[col*n + c], pv_inv);
    for (int c = 0; c < n; c++) inv[col*n + c] = gfMul(inv[col*n + c], pv_inv);

    for (int r = 0; r < n; r++) {
      if (r == col) continue;
      uint8_t factor = work[r*n + col];
      if (factor == 0) continue;
      for (int c = 0; c < n; c++) {
        work[r*n + c] ^= gfMul(factor, work[col*n + c]);
        inv[r*n + c] ^= gfMul(factor, inv[col*n + c]);
      }
    }
  }

  free(work);
  return true;
}

// ---------------- PARSE ----------------
bool parseFile(String path) {
  cpuTurbo();
  logD("parse:start " + path);
  delay(50);
  loraToSD();
  if (!SD.exists(path)) { logN("[ERREUR] Impossible de parser " + path + " : fichier introuvable"); return false; }
  File inF = SD.open(path, FILE_READ);
  if (!inF) { logN("[ERREUR] Impossible d'ouvrir " + path + " pour l'encodage"); return false; }

  uint32_t fileSize = inF.size();
  uint32_t fcrc = crc32_file(inF);
  inF.seek(0);

  uint32_t totalDataPackets = (fileSize + PACKET_SIZE - 1) / PACKET_SIZE;
  uint32_t parityGroups = (totalDataPackets + GROUP_K - 1) / GROUP_K;

  if(serialLevel >= LOG_DEBUG){ char _b[64]; snprintf(_b, sizeof(_b), "parse:%u oct CRC=%08X pkt=%u grp=%u", fileSize, fcrc, totalDataPackets, parityGroups); ioOutput(String(_b)); }

  if (SD.exists("/tx.txt")) SD.remove("/tx.txt");
  File tx = SD.open("/tx.txt", FILE_WRITE);
  if (!tx) { inF.close(); logN("[ERREUR SD] Impossible de créer /tx.txt pour l'encodage"); return false; }

  tx.printf("META:%u:%08X:%u:%u\n", totalDataPackets, fcrc, fileSize, parityGroups);
  tx.flush();

  uint8_t *dataBuf[GROUP_K];
  for (int i=0;i<GROUP_K;i++) dataBuf[i] = (uint8_t*)calloc(PACKET_SIZE,1);
  uint8_t *parityBuf[PARITY_M];
  for (int p=0;p<PARITY_M;p++) parityBuf[p] = (uint8_t*)calloc(PACKET_SIZE,1);

  char hexBuf[5]; hexBuf[4] = 0;
  uint8_t chunkData[CHUNK_SIZE];

  for (uint32_t g = 0; g < parityGroups; g++) {
    for (int i=0;i<GROUP_K;i++) {
      memset(dataBuf[i],0,PACKET_SIZE);
      uint32_t pid = g*GROUP_K + i;
      if (pid < totalDataPackets) {
        inF.read(dataBuf[i], PACKET_SIZE);
      }

      int bytesWritten = 0;
      while (bytesWritten < PACKET_SIZE) {
        int toWrite = min(CHUNK_SIZE, PACKET_SIZE - bytesWritten);
        memcpy(chunkData, dataBuf[i] + bytesWritten, toWrite);
        uint16_t lineCRC = crc16(chunkData, toWrite);
        tx.printf("D%02X%02X%02X", (unsigned)(g & 0xFF), i, (bytesWritten & 0xFF));
        for (int b=0; b<toWrite; b++) {
          sprintf(hexBuf, "%02X", chunkData[b]);
          tx.print(hexBuf);
        }
        sprintf(hexBuf, "%04X", lineCRC);
        tx.print(hexBuf);
        tx.println();
        bytesWritten += toWrite;
      }
    }

    encodeGroupParity(dataBuf, parityBuf);
    for (int p=0;p<PARITY_M;p++) {
      int bytesWritten = 0;
      while (bytesWritten < PACKET_SIZE) {
        int toWrite = min(CHUNK_SIZE, PACKET_SIZE - bytesWritten);
        memcpy(chunkData, parityBuf[p] + bytesWritten, toWrite);
        uint16_t lineCRC = crc16(chunkData, toWrite);
        tx.printf("P%02X%02X%02X", (unsigned)(g & 0xFF), p, (bytesWritten & 0xFF));
        for (int b=0; b<toWrite; b++) {
          sprintf(hexBuf, "%02X", chunkData[b]);
          tx.print(hexBuf);
        }
        sprintf(hexBuf, "%04X", lineCRC);
        tx.print(hexBuf);
        tx.println();
        bytesWritten += toWrite;
      }
    }

    if ((g+1) % 20 == 0) {
      tx.flush();
      if(serialLevel >= LOG_DEBUG){ ioOutput("encode:" + String(g+1) + "/" + String(parityGroups)); }
    }
  }

  for (int i=0;i<GROUP_K;i++) free(dataBuf[i]);
  for (int p=0;p<PARITY_M;p++) free(parityBuf[p]);

  tx.flush();
  tx.close();
  inF.close();

  if (SD.exists("/large.cmd") && path == "/large.cmd") SD.remove("/large.cmd");

  sdToLora();
  logV("parse:ok " + path);
  return true;
}

// ---------------- COMPILE ----------------
void compileFile(String fnameced, int origin, int toremof) {
  cpuTurbo();
  logD("compile:start " + fnameced);
  delay(50);
  loraToSD();
  if (!SD.exists("/rx.txt")) { logN("[ERREUR] Compilation impossible : /rx.txt introuvable"); return; }
  File rx = SD.open("/rx.txt", FILE_READ);
  if (!rx) { logN("[ERREUR SD] Impossible d'ouvrir /rx.txt pour la compilation"); return; }

  String meta = rx.readStringUntil('\n'); meta.trim();
  if (!meta.startsWith("META:")) { rx.close(); logN("[ERREUR] Compilation : en-tête META manquant ou corrompu"); return; }

  unsigned int totalDataPackets=0, fileCRC=0, expectedSize=0, parityGroups=0;
  int scanned = sscanf(meta.c_str(), "META:%u:%X:%u:%u", &totalDataPackets, &fileCRC, &expectedSize, &parityGroups);
  if (scanned<4) { logN("[ERREUR] Compilation : impossible de lire les paramètres META"); rx.close(); return; }

  if(serialLevel >= LOG_DEBUG){ char _b[64]; snprintf(_b, sizeof(_b), "compile:%u pkt %u oct CRC=%08X", totalDataPackets, expectedSize, fileCRC); ioOutput(String(_b)); }

  if (SD.exists(fnameced)) SD.remove(fnameced);
  File out = SD.open(fnameced, FILE_WRITE);
  if (!out) { rx.close(); logN("[ERREUR SD] Impossible de créer le fichier de sortie de compilation"); return; }

  uint32_t bytesWritten=0;

  uint8_t *dataBuf[GROUP_K];
  bool hasData[GROUP_K];
  uint8_t *parityBuf[PARITY_M];
  bool hasParity[PARITY_M];
  uint8_t dataBitmap[GROUP_K][4];
  uint8_t parityBitmap[PARITY_M][4];

  for (int i=0; i<GROUP_K; i++) dataBuf[i] = (uint8_t*)calloc(PACKET_SIZE, 1);
  for (int p=0; p<PARITY_M; p++) parityBuf[p] = (uint8_t*)calloc(PACKET_SIZE, 1);

  uint8_t chunkData[CHUNK_SIZE];
  int crcRejects = 0;

  for (unsigned int g=0; g<parityGroups; g++) {
    for (int i=0; i<GROUP_K; i++) {
      memset(dataBuf[i], 0, PACKET_SIZE);
      hasData[i] = false;
      memset(dataBitmap[i], 0, 4);
    }
    for (int p=0; p<PARITY_M; p++) {
      memset(parityBuf[p], 0, PACKET_SIZE);
      hasParity[p] = false;
      memset(parityBitmap[p], 0, 4);
    }

    rx.seek(0);
    rx.readStringUntil('\n');

    while (rx.available()) {
      String line = rx.readStringUntil('\n');
      line.trim();
      if (line.length() < 9) continue;  // 7 header + 2 CRC minimum

      if (line[0] == 'D' || line[0] == 'P') {
        char type = line[0];
        char gidStr[3] = {line[1], line[2], 0};
        char idxStr[3] = {line[3], line[4], 0};
        char offsetStr[3] = {line[5], line[6], 0};

        if (!isxdigit(gidStr[0]) || !isxdigit(gidStr[1]) ||
            !isxdigit(idxStr[0]) || !isxdigit(idxStr[1]) ||
            !isxdigit(offsetStr[0]) || !isxdigit(offsetStr[1])) {
          continue;
        }

        unsigned int gid = strtol(gidStr, NULL, 16);
        unsigned int idx = strtol(idxStr, NULL, 16);
        unsigned int offset = strtol(offsetStr, NULL, 16);

        if (gid != g) continue;

        uint8_t *targetBuf = NULL;
        uint8_t *targetBitmap = NULL;
        bool *targetFlag = NULL;

        if (type == 'D' && idx < GROUP_K) {
          targetBuf = dataBuf[idx];
          targetBitmap = dataBitmap[idx];
          targetFlag = &hasData[idx];
        }
        else if (type == 'P' && idx < PARITY_M) {
          targetBuf = parityBuf[idx];
          targetBitmap = parityBitmap[idx];
          targetFlag = &hasParity[idx];
        }

        if (targetBuf && targetBitmap) {
          int dataLen = line.length() - 7 - 4;  // minus header and CRC16
          if (dataLen < 0 || dataLen % 2 != 0) continue;

          int numBytes = dataLen / 2;
          if (numBytes > CHUNK_SIZE) numBytes = CHUNK_SIZE;

          bool parseOK = true;
          for (int b = 0; b < numBytes; b++) {
            int pos = 7 + b * 2;
            char c1 = line[pos];
            char c2 = line[pos+1];
            if (isxdigit(c1) && isxdigit(c2)) {
              char byteStr[3] = {c1, c2, 0};
              chunkData[b] = (uint8_t)strtol(byteStr, NULL, 16);
            } else {
              parseOK = false;
              break;
            }
          }

          if (!parseOK) continue;

          char crcStr[5] = {line[line.length()-4], line[line.length()-3],
                            line[line.length()-2], line[line.length()-1], 0};
          if (!isxdigit(crcStr[0]) || !isxdigit(crcStr[1]) ||
              !isxdigit(crcStr[2]) || !isxdigit(crcStr[3])) continue;
          uint16_t receivedCRC = (uint16_t)strtol(crcStr, NULL, 16);

          uint16_t calculatedCRC = crc16(chunkData, numBytes);
          if (calculatedCRC != receivedCRC) {
            crcRejects++;
            continue;
          }

          int bufPos = offset;
          for (int b = 0; b < numBytes && bufPos < PACKET_SIZE; b++) {
            targetBuf[bufPos++] = chunkData[b];
          }

          int chunkIdx = offset / CHUNK_SIZE;
          if (chunkIdx < 4) {
            targetBitmap[chunkIdx] = 1;
          }

          int numChunks = (PACKET_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE;
          bool complete = true;
          for (int c = 0; c < numChunks; c++) {
            if (targetBitmap[c] == 0) { complete = false; break; }
          }
          if (complete) *targetFlag = true;
        }
      }
    }

    for (int i=0; i<GROUP_K; i++) {
      unsigned int pid=g*GROUP_K+i;
      if (pid>=totalDataPackets) hasData[i]=true;
    }

    int missingCount=0, missingIdx[GROUP_K];
    for (int i=0;i<GROUP_K;i++) {
      unsigned int pid=g*GROUP_K+i;
      if (pid<totalDataPackets && !hasData[i]) missingIdx[missingCount++]=i;
    }

    if (missingCount>0 && missingCount<=PARITY_M) {
      int availP=0, parityIdxs[PARITY_M];
      for (int p=0;p<PARITY_M;p++) if (hasParity[p]) parityIdxs[availP++]=p;

      if (availP>=missingCount) {
        int n=missingCount;
        uint8_t *mat=(uint8_t*)malloc(n*n);
        uint8_t *inv=(uint8_t*)malloc(n*n);

        if (mat && inv) {
          memset(mat,0,n*n);
          for(int r=0;r<n;r++) {
            uint8_t alpha = (uint8_t)(parityIdxs[r]+1);
            for(int c=0;c<n;c++) mat[r*n+c] = gfPow(alpha, missingIdx[c]);
          }

          if (invertMatrixGF(mat,inv,n)) {
            uint8_t rhs[PARITY_M], sol[PARITY_M];
            for(int b=0;b<PACKET_SIZE;b++){
              for(int r=0;r<n;r++){
                uint8_t val=parityBuf[parityIdxs[r]][b];
                for(int i=0;i<GROUP_K;i++){
                  if (hasData[i]) {
                    uint8_t alpha = (uint8_t)(parityIdxs[r]+1);
                    val ^= gfMul(dataBuf[i][b], gfPow(alpha, i));
                  }
                }
                rhs[r]=val;
              }
              for(int r=0;r<n;r++){
                uint8_t acc=0;
                for(int c=0;c<n;c++) acc^=gfMul(inv[r*n+c],rhs[c]);
                sol[r]=acc;
              }
              for(int m=0;m<n;m++) dataBuf[missingIdx[m]][b]=sol[m];
            }
            for(int m=0;m<n;m++) hasData[missingIdx[m]] = true;
          }
          free(mat); free(inv);
        }
      }
    }

    for (int i=0;i<GROUP_K;i++) {
      unsigned int pid=g*GROUP_K+i;
      if (pid>=totalDataPackets) break;
      uint32_t remain=(expectedSize>bytesWritten)?(expectedSize-bytesWritten):0;
      size_t toWrite=(remain>=PACKET_SIZE)?PACKET_SIZE:remain;
      if (toWrite>0){ out.write(dataBuf[i],toWrite); bytesWritten+=toWrite; }
    }

    if ((g+1) % 10 == 0) {
      out.flush();
      if(serialLevel >= LOG_DEBUG){ ioOutput("decode:" + String(g+1) + "/" + String(parityGroups)); }
    }
  }

  for (int i=0; i<GROUP_K; i++) free(dataBuf[i]);
  for (int p=0; p<PARITY_M; p++) free(parityBuf[p]);

  out.flush();
  out.close();
  rx.close();

  if(serialLevel >= LOG_DEBUG){ ioOutput("compile:CRC rejects=" + String(crcRejects)); }

  File outf = SD.open(fnameced, FILE_READ);
  if (!outf) { logN("[ERREUR] Fichier compilé introuvable après écriture"); sdToLora(); return; }
  uint32_t finalCRC=crc32_file(outf);
  uint32_t finalSize=outf.size();
  SD.remove("/rx.txt");
  outf.close();
  outf = SD.open(fnameced, FILE_READ);
  String tointlarge;
  if(fnameced == "/large.cmd"){
    logD("large.cmd: lecture");
    while (outf.available()) {
      tointlarge += (char)outf.read();
    }
    if (SD.exists("/large.cmd")) SD.remove("/large.cmd");
  }
  outf.close();
  sdToLora();

  if(serialLevel >= LOG_DEBUG){ char _b[80]; snprintf(_b, sizeof(_b), "compile:%u/%u oct CRC=%08X/%08X", finalSize, expectedSize, finalCRC, fileCRC); ioOutput(String(_b)); }
  if(finalSize == expectedSize && finalCRC == fileCRC){
    logN("[FICHIER RX] Fichier " + fnameced + " reçu et compilé avec succès" + (origin >= 0 ? " (depuis noeud #" + String(origin) + ")" : " (diffusion réseau)"));
    if(origin >= 0){
      String tempfeok = "send:";
      tempfeok += origin;
      tempfeok += ":feok:";
      tempfeok += localAddress;
      tempfeok += ":";
      tempfeok += fnameced;
      tempfeok += ":";
      tempfeok += toremof;
      logD("compile:feok " + tempfeok);
      interpreter(tempfeok);
    }
    // Diffusion de la réussite de compilation uniquement pour les fichiers reçus via diff
    if (origin == -1) {
      String bcokCmd = "bcok:";
      bcokCmd += fnameced;
      scheduleCommand(300000, bcokCmd);  // 5 min = 300 000 ms
    }
    if(fnameced == "/large.cmd"){
      logD("large.cmd: " + tointlarge);
      scheduleCommand(500, tointlarge);
    }
  }
}

void addVertex(int v) {
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i] == v) {
      return;
    }
  }
  if (numVertices < MAX_VERTICES) {
    vertices[numVertices++] = v;
  }
}

void updateVertices() {
  numVertices = 0;
  for (int i = 0; i < numEdges; i++) {
    addVertex(edges[i].vertex1);
    addVertex(edges[i].vertex2);
  }
}

String exportEdgesAstmapCommand() {
  String tmapCommand = "tmap";
  
  for (int i = 0; i < numEdges; i++) {
    tmapCommand += ":";
    tmapCommand += edges[i].vertex1;
    tmapCommand += ",";
    tmapCommand += edges[i].vertex2;
    tmapCommand += ",";
    tmapCommand += edges[i].weight;
  }
  
  return tmapCommand;
}

String exportEdgesContainingVertex(int vertex) {
  String tempid = generateid();
  String umapCommand = "umap:";
  umapCommand += tempid;
  umapCommand += ":";
  umapCommand += vertex;
  
  for (int i = 0; i < numEdges; i++) {
    if (edges[i].vertex1 == vertex || edges[i].vertex2 == vertex) {
      umapCommand += ":";
      umapCommand += edges[i].vertex1;
      umapCommand += ",";
      umapCommand += edges[i].vertex2;
      umapCommand += ",";
      umapCommand += edges[i].weight;
    }
  }
  
  if (umapCommand.length() == 4) { // "umap" only
    logD("edge:aucune pour " + String(vertex));
    return "";
  }
  
  return umapCommand;
}

void addOrUpdateEdge(int v1, int v2, int weight) {
  bool edgeExists = false;

  for (int i = 0; i < numEdges; i++) {
    if ((edges[i].vertex1 == v1 && edges[i].vertex2 == v2) || (edges[i].vertex1 == v2 && edges[i].vertex2 == v1)) {
      // Calculate the average of the current weight and the new weight
      edges[i].weight = (edges[i].weight + weight) / 2;
      edgeExists = true;
      logD("edge:upd " + String(v1) + "-" + String(v2) + " w=" + String(edges[i].weight));
      break;
    }
  }

  if (!edgeExists && numEdges < MAX_EDGES) {
    edges[numEdges].vertex1 = v1;
    edges[numEdges].vertex2 = v2;
    edges[numEdges].weight = weight;
    numEdges++;
    logD("edge:add " + String(v1) + "-" + String(v2) + " w=" + String(weight));
    updateVertices();
  } else if (numEdges >= MAX_EDGES) {
    logN("[ERREUR] Table de routage pleine, nouveau lien ignoré");
  }
}

void clearEdges() {
  numEdges = 0;
  logV("edge:clear");
  updateVertices();
}

void printEdges() {
  if(serialLevel < LOG_NORMAL) return;
  ioOutput("edges:");
  for (int i = 0; i < numEdges; i++) {
    ioOutput(String(edges[i].vertex1) + "-" + String(edges[i].vertex2) + " w:" + String(edges[i].weight));
  }
}

int getVertexIndex(int v) {
  for (int i = 0; i < numVertices; i++) {
    if (vertices[i] == v) {
      return i;
    }
  }
  return -1;
}

int minDistance(int dist[], bool sptSet[]) {
  int min = INT_MAX, minIndex;
  for (int v = 0; v < numVertices; v++) {
    if (!sptSet[v] && dist[v] <= min) {
      min = dist[v];
      minIndex = v;
    }
  }
  return minIndex;
}

void printPath(int parent[], int j, String& path) {
  if (parent[j] == -1) {
    path += vertices[j];
    return;
  }
  printPath(parent, parent[j], path);
  path += " -> ";
  path += vertices[j];
}

int getNextStep(int parent[], int destIndex, int srcIndex) {
  int current = destIndex;
  while (parent[current] != srcIndex) {
    current = parent[current];
  }
  return vertices[current];
}

int nextstep(int src, int dest) {
  updateVertices(); // Update vertices before running Dijkstra

  int dist[MAX_VERTICES];
  bool sptSet[MAX_VERTICES];
  int parent[MAX_VERTICES];

  for (int i = 0; i < numVertices; i++) {
    dist[i] = INT_MAX;
    sptSet[i] = false;
    parent[i] = -1;
  }

  int srcIndex = getVertexIndex(src);
  int destIndex = getVertexIndex(dest);

  if (srcIndex == -1 || destIndex == -1) {
    return -1;
  }

  dist[srcIndex] = 0;

  for (int count = 0; count < numVertices - 1; count++) {
    int u = minDistance(dist, sptSet);
    sptSet[u] = true;

    for (int i = 0; i < numEdges; i++) {
      int vIndex = (edges[i].vertex1 == vertices[u]) ? getVertexIndex(edges[i].vertex2) : 
                   (edges[i].vertex2 == vertices[u]) ? getVertexIndex(edges[i].vertex1) : -1;

      if (vIndex != -1 && !sptSet[vIndex] && dist[u] != INT_MAX && dist[u] + edges[i].weight < dist[vIndex]) {
        dist[vIndex] = dist[u] + edges[i].weight;
        parent[vIndex] = u;
      }
    }
  }

  if (dist[destIndex] == INT_MAX) {
    return -1;
  }
  
  int nextStep = getNextStep(parent, destIndex, srcIndex); 
  return nextStep;
}

bool dijkstra(int src, int dest, String outgoing) {
  updateVertices(); // Update vertices before running Dijkstra

  int dist[MAX_VERTICES];
  bool sptSet[MAX_VERTICES];
  int parent[MAX_VERTICES];

  for (int i = 0; i < numVertices; i++) {
    dist[i] = INT_MAX;
    sptSet[i] = false;
    parent[i] = -1;
  }

  int srcIndex = getVertexIndex(src);
  int destIndex = getVertexIndex(dest);

  if (srcIndex == -1 || destIndex == -1) {
    logD("dijk:err src/dest invalide");
    return false;
  }

  dist[srcIndex] = 0;

  for (int count = 0; count < numVertices - 1; count++) {
    int u = minDistance(dist, sptSet);
    sptSet[u] = true;

    for (int i = 0; i < numEdges; i++) {
      int vIndex = (edges[i].vertex1 == vertices[u]) ? getVertexIndex(edges[i].vertex2) : 
                   (edges[i].vertex2 == vertices[u]) ? getVertexIndex(edges[i].vertex1) : -1;

      if (vIndex != -1 && !sptSet[vIndex] && dist[u] != INT_MAX && dist[u] + edges[i].weight < dist[vIndex]) {
        dist[vIndex] = dist[u] + edges[i].weight;
        parent[vIndex] = u;
      }
    }
  }

  if (dist[destIndex] == INT_MAX) {
    logD("dijk:no path " + String(src) + "->" + String(dest));
    return false;
  }

  String path = "";
  printPath(parent, destIndex, path);
  int nextStep = getNextStep(parent, destIndex, srcIndex);
  logD("dijk:" + String(src) + "->" + String(dest) + " next=" + String(nextStep) + " d=" + String(dist[destIndex]));
  String tosenddijk = "trsm:";
  tosenddijk += nextStep;
  tosenddijk += ":";
  tosenddijk += outgoing;
  scheduleCommand(50, tosenddijk);
  return true;
}

void removeEdgesByVertex(int v) {
  int newNumEdges = 0;
  for (int i = 0; i < numEdges; i++) {
    if (edges[i].vertex1 != v && edges[i].vertex2 != v) {
      edges[newNumEdges++] = edges[i];
    }
  }
  numEdges = newNumEdges;
  logD("edge:clear vertex=" + String(v));
  updateVertices();
}

void removeEdge(int v1, int v2) {
  bool edgeFound = false;

  for (int i = 0; i < numEdges; i++) {
    if ((edges[i].vertex1 == v1 && edges[i].vertex2 == v2) || (edges[i].vertex1 == v2 && edges[i].vertex2 == v1)) {
      // Décaler les arêtes suivantes pour écraser celle à supprimer
      for (int j = i; j < numEdges - 1; j++) {
        edges[j] = edges[j + 1];
      }
      numEdges--;
      edgeFound = true;
      logD("edge:rm " + String(v1) + "-" + String(v2));
      break;
    }
  }
}

int findNearestVertex(int src) {
  updateVertices(); // Update vertices to ensure they are up-to-date

  int srcIndex = getVertexIndex(src);

  if (srcIndex == -1) { return -1; }

  int minWeight = INT_MAX;
  int nearestVertex = -1;

  for (int i = 0; i < numEdges; i++) {
    if (edges[i].vertex1 == src && edges[i].weight < minWeight) {
      minWeight = edges[i].weight;
      nearestVertex = edges[i].vertex2;
    } else if (edges[i].vertex2 == src && edges[i].weight < minWeight) {
      minWeight = edges[i].weight;
      nearestVertex = edges[i].vertex1;
    }
  }

  logD("nearest:" + String(src) + "->" + String(nearestVertex) + " w=" + String(minWeight));
  return nearestVertex;
}

void addValue(String newValue) {
  if (dataCount < MAX_SIZE) {
    dataArray[dataCount].value = newValue;
    dataArray[dataCount].time = millis();
    dataCount++;
  }
}

void removeExpiredValues() {
  unsigned long currentTime = millis();

  for (int i = 0; i < dataCount; i++) {
    if (currentTime - dataArray[i].time >= DELAY) {
      
      // Déplace toutes les valeurs suivantes d'une place vers l'avant
      for (int j = i; j < dataCount - 1; j++) {
        dataArray[j] = dataArray[j + 1];
      }
      
      dataCount--; // Réduit le nombre de valeurs stockées
      i--; // Pour vérifier la nouvelle valeur à cet indice
    }
  }
}

void printDataArray() {
  if(serialLevel < LOG_NORMAL) return;
  ioOutput("cache:");
  for (int i = 0; i < dataCount; i++) {
    ioOutput(dataArray[i].value + " +" + String(DELAY - (millis() - dataArray[i].time)) + "ms");
  }
}

// Fonction pour rechercher une valeur dans le tableau
bool findValue(String valueToFind) {
  for (int i = 0; i < dataCount; i++) {
    if (dataArray[i].value == valueToFind) {
      return true; // Valeur trouvée
    }
  }
  return false; // Valeur non trouvée
}

String generateid(){
  String msgid = "";
  msgid += formatNumber(msgCount % (int)pow(10, 2), 3);
  msgid += formatNumber(localAddress % (int)pow(10, 2), 3);
  msgid += formatNumber(random(99) % (int)pow(10, 2), 2);
  return msgid;
}

String formatNumber(int number, int digits) {
  // Crée un buffer de la taille requise pour contenir les digits + le caractère de fin de chaîne '\0'
  char buffer[digits + 1];

  // Crée le format de chaîne avec des zéros devant, ex: "%04d"
  String format = "%0" + String(digits) + "d";

  // Utilise sprintf pour formater le nombre avec des zéros devant
  sprintf(buffer, format.c_str(), number);

  return String(buffer);
}

void startprocedure(){
  if(startstat == 1){
    interpreter("pigo");
    startstat = 2;
    logN("[DÉMARRAGE] Lancement de la découverte réseau (ping)...");
  }
  if(startstat == 2 && pingphase == 0 && ((millis()/1000) - fpingdel > 30)){
    startstat = 3;
  }
  if(startstat == 3){
   nearestforstart = findNearestVertex(localAddress);
   logV("start:3 nearest=" + String(nearestforstart));
   if(nearestforstart == -1){
    startstat = 9;
    logN("[ERREUR] Démarrage échoué : aucun voisin détecté après le ping");
    return;
   }
   else{
    sendMessage(1, "gmap", nearestforstart);
    starttime = millis() + starttimeout;
    startstat = 4;
    starttry = 1;
    logN("[DÉMARRAGE] Récupération de la carte réseau auprès du noeud #" + String(nearestforstart) + "...");
   logV("start:4 gmap->" + String(nearestforstart));
   }
  }
  if(startstat == 4 && millis() >= starttime && starttry < 3){
    sendMessage(1, "gmap", nearestforstart);
    starttry ++;
    starttime = millis() + starttimeout;
    logV("start:4 retry gmap");
  }
  if(startstat == 4 && millis() >= starttime && starttry >= 3){
    startstat = 3;
    logV("start:3 map timeout, retry");
    removeEdge(localAddress, nearestforstart);
  }
  if(startstat == 5 && ((millis()/1000) - rtmapdel > 40)){
    startstat = 6;
    logV("start:6 map ok");
    }
  if(startstat == 6){
   delay(3000);
   nearestforstart = findNearestVertex(localAddress);
   logV("start:6 nearest=" + String(nearestforstart));
   if(nearestforstart == -1){
    startstat = 9;
    logN("[ERREUR] Démarrage échoué : aucun voisin trouvé après réception de la carte réseau");
    return;
   }
   else{
    timegeth = millis();
    sendMessage(1, "geth", nearestforstart);
    starttime = millis() + starttimeout;
    startstat = 7;
    starttry = 1;
   logN("[DÉMARRAGE] Synchronisation de l'heure avec le noeud #" + String(nearestforstart) + "...");
   logV("start:7 geth->" + String(nearestforstart));
   }
  }
  if(startstat == 7 && millis() >= starttime && starttry < 3){
    timegeth = millis();
    sendMessage(1, "geth", nearestforstart);
    starttry ++;
    starttime = millis() + starttimeout;
    logV("start:7 retry geth");
  }
  if(startstat == 7 && millis() >= starttime && starttry >= 3){
    startstat = 6;
    logV("start:6 time timeout, retry");
    removeEdge(localAddress, nearestforstart);
  }
}

String exportdata(String ver){
  if(ver == 0){
    interpreter("gmea:0");
  }
  delay(50);
  loraToSD();
  File myFile;

  myFile = SD.open("/data.ver", FILE_READ);       
  String datastructver = "";
  if (myFile) {
    while (myFile.available()) {
      datastructver += (char)myFile.read();
    }
    myFile.close();
  }

  String path = "/";
  path += String(ver);
  path += ".datadump";
  myFile = SD.open(path, FILE_READ);       
  String indumpe = "";
  if (myFile) {
    while (myFile.available()) {
      indumpe += (char)myFile.read();
    }
    myFile.close();
  }

  myFile = SD.open(path, FILE_WRITE);   
    if (myFile) {
      myFile.print("");
      myFile.close();
    }
    else{
      logN("[ERREUR SD] Impossible d'écrire le fichier de mesures capteur");
    }

    sdToLora();

  int i = 0;
  while(getValue(datastructver, ';', i) != "") {
    String parsver = getValue(datastructver, ';', i);
    if(getValue(parsver, ' ', 0) == ver){
      String tortdat = "data:";
      tortdat += localAddress;
      tortdat += ":";
      tortdat += String(ver);
      tortdat += compildata(getValue(parsver, ' ', 1), indumpe);
      return(tortdat);
    }
    i ++;
  } 
}

String compildata(String outstandstruct, String indump){
  int i = 0;
  String compileddata = "";
  while(getValue(outstandstruct, ':', i) != "") {
    compileddata += ":";
    compileddata += extractvalue(indump, getValue(outstandstruct, ':', i));
    i ++;
  }
  return(compileddata);
}

String extractvalue(String indump, String rshval){

  if(rshval.substring(0, 1) == "n"){
    float temprecoval = 0;
    int temprecocon = 0;
    int i = 0;
    while(getValue(indump, ';', i) != "") {
      String toandump = getValue(indump, ';', i);
      if(getValue(toandump, ':', 0) == rshval){
        temprecoval = temprecoval + getValue(toandump, ':', 1).toFloat();
        temprecocon ++;
      }
      i ++;
    }
    return(String(temprecoval/temprecocon));
  }
  if(rshval.substring(0, 1) == "t"){
    String temprecoval = "";
    int i = 0;
    while(getValue(indump, ';', i) != "") {
      String toandump = getValue(indump, ';', i);
      if(getValue(toandump, ':', 0) == rshval){
        String tempvalrc = getValue(toandump, ':', 1);
        if(tempvalrc.substring(tempvalrc.length()-1, tempvalrc.length()) == ";"){
          temprecoval = tempvalrc.substring(0, tempvalrc.length()-1);
        }
        else{
          temprecoval = tempvalrc;
        }
      }
      i ++;
    }
    return(temprecoval);
  }
}

void startsensor(){
  if (sensor_bme680) {
    if (!bme.begin()) { logV("sensor:BME680 err"); } else { logV("sensor:BME680 ok"); }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
  }

  if (sensor_aht20) {
    if (aht.begin() != true) { logV("sensor:AHT20 err"); } else { logV("sensor:AHT20 ok"); }
  }

  if (sensor_bmp280) {
    if (!bmp.begin()) { logV("sensor:BMP280 err"); } else { logV("sensor:BMP280 ok"); }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,      /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,     /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,       /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500);  /* Standby time. */
  }
  if (sensor_ds18b20) {
    sensors.begin();  
    logV("sensor:DS18b20 ok");
  }
}

void measuretodump(int ver){
  if(sensorstart == false){
    startsensor();
    sensorstart = true;
  }
  String tosdarg = "";
  if(ver == 0){    
    float batt = (analogReadMilliVolts(0));
    tosdarg += "nbatt:";
    tosdarg += String(batt/500);
    tosdarg += ";";
    tosdarg += "tstartstat:";
    tosdarg += String(startstat);
    tosdarg += ";";
    tosdarg += "tmaintmode:";
    tosdarg += String(maintmode);
    tosdarg += ";";
    tosdarg += "trxglob:";
    tosdarg += rxglob;
    tosdarg += ";";
    tosdarg += "trxbrd:";
    tosdarg += rxbrd;
    tosdarg += ";";
    tosdarg += "trxloc:";
    tosdarg += rxloc;
    tosdarg += ";";
    tosdarg += "tdijktx:";
    tosdarg += dijktx;
    tosdarg += ";";
    tosdarg += "tdijkrx:";
    tosdarg += dijkrx;
    tosdarg += ";";
    tosdarg += "tdijkhop:";
    tosdarg += dijkhop;
    tosdarg += ";";
    tosdarg += "tresend1:";
    tosdarg += resend1;
    tosdarg += ";";
    tosdarg += "tresend2:";
    tosdarg += resend2;
    tosdarg += ";";
    tosdarg += "ttxcnt:";
    tosdarg += txcnt;
    tosdarg += ";";
    tosdarg += "ntime:";
    tosdarg += (rtc.getLocalEpoch());
    tosdarg += ";";

    logV("sensor:v0 " + tosdarg);
  }
  if(ver == 1){
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
      logV("sensor:BME680 begin err");
      return;
    }

    if (!bme.endReading()) {
      logV("sensor:BME680 read err");
      return;
    }
    tosdarg += "ntemp:";
    tosdarg += bme.temperature;
    tosdarg += ";";
    tosdarg += "nhum:";
    tosdarg += bme.humidity;
    tosdarg += ";";
    tosdarg += "npres:";
    tosdarg += (bme.pressure / 100.0);
    tosdarg += ";";
    tosdarg += "nres:";
    tosdarg += (bme.gas_resistance / 1000.0);
    tosdarg += ";";
    tosdarg += "ntime:";
    tosdarg += (rtc.getLocalEpoch());
    tosdarg += ";";

    logV("sensor:v1 " + tosdarg);
  }

  if(ver == 2){
    tosdarg += "test:ok;tsp:okb;";
    logV("sensor:v2 " + tosdarg);
  }
  if(ver == 3){
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  tosdarg += "ntemp:";
    tosdarg += temp.temperature;
    tosdarg += ";nhum:";
    tosdarg += humidity.relative_humidity;
    tosdarg += ";npres:";
    tosdarg += (bmp.readPressure());
    tosdarg += ";ntime:";
    tosdarg += (rtc.getLocalEpoch());
    tosdarg += ";";
    logV("sensor:v3 " + tosdarg);
  }
  if(ver == 4){
    sensors.requestTemperatures(); 
    delay(50);
    tosdarg += "ntemp:";
    tosdarg += sensors.getTempCByIndex(0);
    tosdarg += ";ntime:";
    tosdarg += (rtc.getLocalEpoch());
    tosdarg += ";";
    logV("sensor:v4 " + tosdarg);
  }

    delay(50);
    loraToSD();
    File myFile;
    String path = "/";
    path += String(ver);
    path += ".datadump";
    myFile = SD.open(path, FILE_APPEND);
    if (myFile) {
      myFile.print(tosdarg);
      myFile.close();
    }
    else{
      logN("[ERREUR SD] Impossible d'écrire dans " + path);
    }

    sdToLora();

}

void readsd(bool allrecover){
      loraToSD();
       File myFile;
       
      if (allrecover == 1){ 
      myFile = SD.open("/map.cfg", FILE_READ);       
        String sdtomap = "";
        if (myFile) {
          while (myFile.available()) {
            sdtomap += (char)myFile.read();
          }
          myFile.close();
        }

        int pcount = 1;
        while (getValue(sdtomap, ':', pcount) != "") {
          addOrUpdateEdge(
            (getValue(getValue(sdtomap, ':', pcount), ',', 0)).toInt(),
            (getValue(getValue(sdtomap, ':', pcount), ',', 1)).toInt(),
            (getValue(getValue(sdtomap, ':', pcount), ',', 2)).toInt()
          );
          pcount++;
        }
        
        myFile = SD.open("/e.cfg", FILE_READ);
        String sdtoenv = "";
        if (myFile) {
          while (myFile.available()) {
            sdtoenv += (char)myFile.read();
          }
          myFile.close();
        }
      
        startstat = getValue(sdtoenv, ':', 0).toInt();
        msgCount = getValue(sdtoenv, ':', 1).toInt();
      }
            
      myFile = SD.open("/p.cfg", FILE_READ);
      String sdtopar = "";
      if (myFile) {
        while (myFile.available()) {
          sdtopar += (char)myFile.read();
        }
        myFile.close();
      }
      MAX_PING_AGE = getValue(sdtopar, ':', 0).toInt() * 1000L;
      starttimeout = getValue(sdtopar, ':', 1).toInt() * 1000L;
      localAddress = getValue(sdtopar, ':', 2).toInt();
      DELAY = getValue(sdtopar, ':', 3).toInt() * 1000L;
      MAX_ENTRY_AGE = getValue(sdtopar, ':', 4).toInt() * 1000L;
      actiontimerdel = getValue(sdtopar, ':', 5).toInt();
      maintmode = getValue(sdtopar, ':', 6).toInt();
      stationgateway = getValue(sdtopar, ':', 7).toInt();
      TOGATE_COMMAND_TIMEOUT = getValue(sdtopar, ':', 8).toInt() * 1000L;
      { int sl = getValue(sdtopar, ':', 9).toInt(); if(sl >= LOG_NONE && sl <= LOG_DEBUG) serialLevel = sl; }
      { int im = getValue(sdtopar, ':', 10).toInt(); if(im >= IO_USB && im <= IO_BLUETOOTH) ioMode = im; }
      { long nt = getValue(sdtopar, ':', 11).toInt(); if(nt > 0) NETIO_TIMEOUT = nt * 1000L; }
      { int ft = getValue(sdtopar, ':', 12).toInt(); if(ft > 0) filetimeout = ft; }
      { int fxt = getValue(sdtopar, ':', 13).toInt(); if(fxt > 0) filetxtimeout = fxt; }

      myFile = SD.open("/crontab.cfg", FILE_READ);
      String sdtocron = "";
      if (myFile) {
        while (myFile.available()) {
          sdtocron += (char)myFile.read();
        }
        myFile.close();
      }
      sdtocron = getValue(sdtocron, '\n', 0);
      logD("cron:" + sdtocron);
      crontabString = sdtocron;

      myFile = SD.open("/sensor.cfg", FILE_READ);
      String sdtosensor = "";
      if (myFile) {
        while (myFile.available()) {
          sdtosensor += (char)myFile.read();
        }
        myFile.close();
      }
      sensor_bme680 = getValue(sdtosensor, ':', 1).toInt() == 1;
      sensor_aht20  = getValue(sdtosensor, ':', 3).toInt() == 1;
      sensor_bmp280 = getValue(sdtosensor, ':', 5).toInt() == 1;
      sensor_ds18b20 = getValue(sdtosensor, ':', 7).toInt() == 1;
      logD("sensor cfg bme680:" + String(sensor_bme680) + " aht20:" + String(sensor_aht20) + " bmp280:" + String(sensor_bmp280) + " DS18b20:" + String(sensor_ds18b20));

      initGaloisField();

      sdToLora();

}

void writetosd(){
    interpreter("gmea:0");
    delay(50);
    loraToSD();
    String vartosd = exportEdgesAstmapCommand();

    File testFile = SD.open("/map.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(vartosd);
      testFile.close();
    }
    else{
      logN("[ERREUR SD] Impossible d'écrire /map.cfg (carte réseau)");
    }

    String varptosd;
    varptosd += MAX_PING_AGE / 1000;
    varptosd += ":";
    varptosd += starttimeout / 1000;
    varptosd += ":";
    varptosd += localAddress;
    varptosd += ":";
    varptosd += DELAY / 1000;
    varptosd += ":";
    varptosd += MAX_ENTRY_AGE / 1000;
    varptosd += ":";
    varptosd += actiontimerdel;
    varptosd += ":";
    varptosd += maintmode;
    varptosd += ":";
    varptosd += stationgateway;
    varptosd += ":";
    varptosd += TOGATE_COMMAND_TIMEOUT / 1000;
    varptosd += ":";
    varptosd += serialLevel;
    varptosd += ":";
    // En mode tunnel réseau (IO_NETIO), on sauvegarde le canal physique précédent
    // pour qu'au redémarrage la station revienne dans son mode normal (USB ou BT).
    varptosd += (ioMode == IO_NETIO ? ntioPrevIoMode : ioMode);
    varptosd += ":";
    varptosd += NETIO_TIMEOUT / 1000;
    varptosd += ":";
    varptosd += filetimeout;
    varptosd += ":";
    varptosd += filetxtimeout;
    varptosd += ":";

    testFile = SD.open("/p.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(varptosd);
      testFile.close();
    }
    else{
      logN("[ERREUR SD] Impossible d'écrire /p.cfg (paramètres)");
    }

        testFile = SD.open("/crontab.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(crontabString);
      testFile.close();
    }
    else{
      logN("[ERREUR SD] Impossible d'écrire /crontab.cfg");
    }

    String varetosd;
    varetosd += startstat;
    varetosd += ":";
    varetosd += msgCount;
    varetosd += ":";

    testFile = SD.open("/e.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(varetosd);
      testFile.close();
    }
    else{
      logN("[ERREUR SD] Impossible d'écrire /e.cfg (état)");
    }

    writeSensorCfg();

    sdToLora();
}

void writeSensorCfg(){
  String varstosd = "bme680:";
  varstosd += sensor_bme680 ? "1" : "0";
  varstosd += ":aht20:";
  varstosd += sensor_aht20  ? "1" : "0";
  varstosd += ":bmp280:";
  varstosd += sensor_bmp280 ? "1" : "0";
  varstosd += ":ds18b20:";
  varstosd += sensor_ds18b20 ? "1" : "0";

  File sensorFile = SD.open("/sensor.cfg", FILE_WRITE);
  if (sensorFile) {
    sensorFile.println(varstosd);
    sensorFile.close();
  } else {
    logN("[ERREUR SD] Impossible d'écrire /sensor.cfg");
  }
}

bool exportcache() {
  delay(50);
  loraToSD();
  File fichier = SD.open("/togate.cache", FILE_READ);
  if (!fichier) {
    logV("cache:vide");
    prefs.putBool("incache", false);
    return false;
  }

  uint32_t offset = prefs.getUInt("offset", 0);
  if (offset >= fichier.size()) {
    fichier.close();
    logV("cache:fin");
    SD.remove("/togate.cache");
    prefs.remove("offset");
    prefs.putBool("incache", false);
    return false;
  }

  fichier.seek(offset);

  char ligne[TAILLE_TAMPON];
  int index = 0;
  bool ligneTrouvee = false;

  while (fichier.available()) {
    char c = fichier.read();
    offset++;

    if (c == '\n' || index >= TAILLE_TAMPON - 1) {
      ligne[index] = '\0';
      ligneTrouvee = true;
      break;
    } else if (c != '\r') {
      ligne[index++] = c;
    }
  }

  if (!ligneTrouvee || index > 0) {
    ligne[index] = '\0';
    logD("cache:ligne " + String(ligne));
    prefs.putUInt("offset", offset);
    fichier.close();
    sdToLora();
    String tempstr = ligne;
    int tempid = getValue(tempstr, ':', 5).toInt();
    togateAddCommand(tempid, tempstr);
    interpreter(ligne);
    return true;
  } else {
    fichier.close();
    sdToLora();
    return false;
  }

 
}

// ---------------- BROADCAST EXPORT LINE ----------------
// Lit une ligne de tx.txt et la diffuse en brdf:SEQ:LINE vers le réseau (dest=0)
// Appelée périodiquement depuis loop() tant que broadcastEmitter && broadcastFileSending
void broadcastExportLine() {
  delay(50);
  loraToSD();

  File fichier = SD.open("/tx.txt", FILE_READ);
  if (!fichier) {
    logN("[ERREUR] Diffusion réseau interrompue : /tx.txt introuvable");
    sdToLora();
    broadcastFileSending = false;
    broadcastMode = false;
    broadcastEmitter = false;
    return;
  }

  uint32_t offsetfile = prefs.getUInt("brdoffset", 0);

  if (offsetfile >= fichier.size()) {
    fichier.close();
    SD.remove("/tx.txt");
    prefs.remove("brdoffset");
    sdToLora();
    // Diffusion du signal de fin
    String tempid = generateid();
    String brde = "brde:";
    brde += tempid;
    brde += ":";
    brde += broadcastPath;
    addValue(tempid);  // Éviter de retraiter notre propre brde
    sendMessage(0, brde, 0);  // Pas de réveil
    broadcastFileSending = false;
    broadcastMode = false;
    broadcastEmitter = false;
    lastBrdfSeq = -1;
    logV("diff:done");
    return;
  }

  fichier.seek(offsetfile);

  char ligne[TAILLE_TAMPON];
  int index = 0;
  bool ligneTrouvee = false;

  while (fichier.available()) {
    char c = fichier.read();
    offsetfile++;
    if (c == '\n' || index >= TAILLE_TAMPON - 1) {
      ligne[index] = '\0';
      ligneTrouvee = true;
      break;
    } else if (c != '\r') {
      ligne[index++] = c;
    }
  }

  fichier.close();

  if (ligneTrouvee || index > 0) {
    ligne[index] = '\0';
    prefs.putUInt("brdoffset", offsetfile);
    sdToLora();

    String ligneStr = String(ligne);
    if (ligneStr.length() > 0) {
      String brdf = "brdf:";
      brdf += String(brdfSeqCounter);
      brdf += ":";
      brdf += ligneStr;
      lastBrdfSeq = brdfSeqCounter;  // Marquer comme déjà traité (ignore rétransmissions)
      brdfSeqCounter++;
      sendMessage(0, brdf, 0);  // Pas de réveil
      logD("diff:seq=" + String(brdfSeqCounter - 1));
    }
  } else {
    sdToLora();
  }
}

bool exportfile() {
  delay(50);
  loraToSD();

  File fichier = SD.open("/tx.txt", FILE_READ);
  if (!fichier) {
    logV("ftx:tx.txt vide");
    infilecache = false;
    sdToLora();
    return false;
  }

  uint32_t offsetfile = prefs.getUInt("offsetfile", 0);
  if (offsetfile >= fichier.size()) {
    fichier.close();
    logV("ftx:fin");
    SD.remove("/tx.txt");
    prefs.remove("offsetfile");
    infilecache = false;
    sdToLora();
    delay(200);
    String tempfend = "send:";
    tempfend += filereceivientstation;
    tempfend += ":fend:";
    tempfend += localAddress;
    tempfend += ":";
    tempfend += ftfpath;
    tempfend += ":";
    tempfend += remof;
    interpreter(tempfend);
    logN("[FICHIER TX] Transfert de " + ftfpath + " terminé, attente de confirmation du noeud #" + String(filereceivientstation) + "...");
    filereceivientstation = -1;
    isfdeson = false;
    return false;
  }

  fichier.seek(offsetfile);

  char ligne[TAILLE_TAMPON];
  int index = 0;
  bool ligneTrouvee = false;

  while (fichier.available()) {
    char c = fichier.read();
    offsetfile++;

    if (c == '\n' || index >= TAILLE_TAMPON - 1) {
      ligne[index] = '\0';
      ligneTrouvee = true;
      break;
    } else if (c != '\r') {
      ligne[index++] = c;
    }
  }

  fichier.close();

  if (ligneTrouvee || index > 0) {
    ligne[index] = '\0';
    
    prefs.putUInt("offsetfile", offsetfile);

    sdToLora();

    String ligneStr = String(ligne);
    if (ligneStr.length() > 0) {
      logD("ftx:ligne " + ligneStr);
      
      // Extraire l'ID de la commande (format identique à exportcache)
      String tempid = generateid();
      togateAddCommandFile(tempid.toInt(), ligneStr);

      String outglignestr = "file:";
      outglignestr += ligneStr;
      // Passer la ligne à l'interpréteur      
      String load = "send:";
      load += filereceivientstation;
      load += ":load:";
      load += localAddress;
      load += ":";
      load += outglignestr.length();
      load += ":";
      load += tempid;
      load += ":";
      load += outglignestr;
      interpreter(load);
      return true;
    }
  }

  sdToLora();
  return false;
}

void importfile(String file, String input){
  delay(50);
  loraToSD();
  File myFile;
  myFile = SD.open(file, FILE_APPEND);
  if (myFile) {
    myFile.println(input);
    myFile.close();
  }
  else{
    logN("[ERREUR SD] Écriture impossible dans " + file);
  }
  sdToLora();
}

bool remfromsd(String rmpath){
  delay(50);
  loraToSD();

  if(SD.remove(rmpath)){
    delay(100);
    sdToLora();
    delay(200);
    return true;
  }
  else{
    delay(100);
    sdToLora();
    delay(200);
    return false;
  }
}

bool mkdirsd(String rmpath){
  delay(50);
  loraToSD();

  if(SD.mkdir(rmpath)){
    delay(100);
    sdToLora();
    delay(200);
    return true;
  }
  else{
    delay(100);
    sdToLora();
    delay(200);
    return false;
  }
}

bool rmdirsd(String rmpath){
  delay(50);
  loraToSD();

  if(SD.rmdir(rmpath)){
    delay(100);
    sdToLora();
    delay(200);
    return true;
  }
  else{
    delay(100);
    sdToLora();
    delay(200);
    return false;
  }
}

bool cpsd(String src, String dst){
  delay(50);
  loraToSD();

  File srcFile = SD.open(src, FILE_READ);
  if(!srcFile){
    sdToLora();
    delay(200);
    return false;
  }

  File dstFile = SD.open(dst, FILE_WRITE);
  if(!dstFile){
    srcFile.close();
    sdToLora();
    delay(200);
    return false;
  }

  uint8_t buf[128];
  while(srcFile.available()){
    int n = srcFile.read(buf, sizeof(buf));
    dstFile.write(buf, n);
  }
  srcFile.close();
  dstFile.close();
  sdToLora();
  delay(200);
  return true;
}

bool mvsd(String src, String dst){
  delay(50);
  loraToSD();

  File srcFile = SD.open(src, FILE_READ);
  if(!srcFile){
    sdToLora();
    delay(200);
    return false;
  }

  File dstFile = SD.open(dst, FILE_WRITE);
  if(!dstFile){
    srcFile.close();
    sdToLora();
    delay(200);
    return false;
  }

  uint8_t buf[128];
  while(srcFile.available()){
    int n = srcFile.read(buf, sizeof(buf));
    dstFile.write(buf, n);
  }
  srcFile.close();
  dstFile.close();
  SD.remove(src);
  sdToLora();
  delay(200);
  return true;
}

void lssd(String path){
  delay(50);
  loraToSD();

  File dir = SD.open(path);
  if(!dir || !dir.isDirectory()){
    if(dir) dir.close();
    logN("[SD] Répertoire " + path + " introuvable ou invalide");
    sdToLora();
    delay(200);
    return;
  }

  if(serialLevel < LOG_NORMAL) { dir.close(); sdToLora(); delay(200); return; }
  String _listing = "";
  File entry = dir.openNextFile();
  while(entry){
    _listing += entry.name();
    if(entry.isDirectory()) _listing += "/";
    _listing += " ";
    entry.close();
    entry = dir.openNextFile();
  }
  ioOutput(_listing);
  dir.close();
  sdToLora();
  delay(200);
}

void large(String tosdlarge, int tosendlarge){
  if(filereceivientstation == -1 && filesender == -1){
      delay(50);
      loraToSD();

      if (SD.exists("/large.cmd")) SD.remove("/large.cmd");
      File testFile = SD.open("/large.cmd", FILE_WRITE);
      if (testFile) {
        testFile.println(tosdlarge);
        testFile.close();
      }

      sdToLora();

      String tofslarge = "stft:";
      tofslarge += tosendlarge;
      tofslarge += ":";
      tofslarge += "/large.cmd";
      tofslarge += ":";
      tofslarge += "0";
      interpreter(tofslarge);
  }
}

bool doFirmwareUpdate() {
  delay(50);
  loraToSD();
  File updateFile = SD.open(UPDATE_FILE);
  if (!updateFile) {
    logN("[ERREUR MÀJU] Fichier firmware introuvable sur la carte SD");
    return false;
  }

  size_t updateSize = updateFile.size();
  logN("[MISE À JOUR] Démarrage de la mise à jour, fichier de " + String(updateSize) + " octets...");

  if (updateSize == 0) {
    logN("[ERREUR MÀJU] Le fichier firmware est vide");
    updateFile.close();
    return false;
  }

  if (!Update.begin(updateSize)) {  // vérifie la partition
    logN("[ERREUR MÀJU] Impossible de démarrer la mise à jour (partition insuffisante ?)");
    Update.printError(Serial);
    updateFile.close();
    return false;
  }

  uint8_t buffer[BUF_SIZE];
  size_t written = 0;

  while (updateFile.available()) {
    size_t len = updateFile.read(buffer, BUF_SIZE);
    if (len <= 0) break;

    size_t w = Update.write(buffer, len);
    written += w;

    if (w != len) {
      logN("[ERREUR MÀJU] Écriture partielle du firmware, mise à jour annulée");
      Update.printError(Serial);
      updateFile.close();
      Update.abort();
      return false;
    }
  }

  updateFile.close();

  if (!Update.end()) {
    logN("[ERREUR MÀJU] Erreur lors de la finalisation de la mise à jour");
    Update.printError(Serial);
    return false;
  }

  if (!Update.isFinished()) {
    logN("[ERREUR MÀJU] Mise à jour incomplète, redémarrage annulé");
    return false;
  }

  logN("[MISE À JOUR] Succès — " + String(written) + " octets écrits, redémarrage dans 1s...");
  prefs.putBool("justrestarted", true);  // Signale qu'une diffusion de version est requise au boot
  delay(1000);
  ESP.restart();
  return true;
}

// --- CPU Frequency Scaling ---
// idle : 20 MHz en mode USB, 80 MHz minimum en mode BLE (contrainte du stack BLE ESP32-C3)
// turbo : 160 MHz pour les traitements intensifs (encodage RS, compilation fichier…)

void cpuTurbo() {
  setCpuFrequencyMhz(CPU_FREQ_TURBO);
}

void cpuIdle() {
  if (ioMode == IO_BLUETOOTH || (ioMode == IO_NETIO && ntioPrevIoMode == IO_BLUETOOTH)){
    setCpuFrequencyMhz(CPU_FREQ_BLE);
  }
  else{
    setCpuFrequencyMhz(CPU_FREQ_IDLE);
  }
}

void setup() {
  Serial.begin(9600);

  prefs.begin("mycromesh", false);
  
  clearEdges();

  LiteLoraConfig cfg = LiteLora::defaultConfig();
  cfg.csPin = csPin;
  cfg.dio0Pin = irqPin;
  cfg.frequency = 433000000;
  cfg.bandwidth = 125000;

  if (!lora.begin(cfg)) {
    ioOutput("err:lora init");
    while (true);
  }

  if(rtc.getLocalEpoch() > 10){
    readsd(1);
  }
  else{
    readsd(0);
    if(maintmode == 0){
      startstat = 1;      
    }
  }

  if (localAddress == 0){
    MAX_PING_AGE = 20000;      // Durée maximale (en millisecondes) avant traitement d'une entrée
    starttimeout = 15000;      // Durée timout procudure start
    localAddress = 61;          // address of this device
    DELAY = 3600000;           // Délai en millisecondes (ici 5 secondes)
    MAX_ENTRY_AGE = 20000;     // Durée maximale (en millisecondes) avant traitement d'une entrée
    actiontimerdel = 30;
    maintmode = true;
  }

  if (ioMode == IO_BLUETOOTH) startBLE();

  logN("[OK] Noeud #" + String(localAddress) + " démarré — firmware " + FIRMWARE_VERSION);
  actiontimer = (millis()/1000);

  // Si ce démarrage fait suite à une mise à jour OTA, planifie la diffusion de la version
  // firmware à l'ensemble du réseau 30 secondes après le boot
  if (prefs.getBool("justrestarted", false)) {
    prefs.putBool("justrestarted", false);
    scheduleCommand(30000 + (80 * localAddress), "fwver");  // 5 min + délai proportionnel au rang
    logN("[MÀJU RÉSEAU] Post-mise à jour — diffusion de version planifiée dans 30 sec");
  }

  lora.receive();
}

void loop() {
  handleSerialInput();
  if (lora.available()) { onReceive(); }
      
  if(pingphase == 1 && ((millis()/1000) - tmps >= 4 || (millis()/1000) < tmps)){         
    sendMessage(1, "ping", 0);
    pingphase = 2;
    logV("ping:2");
    tmps = (millis()/1000);
  }
  if(pingphase == 2 && ((millis()/1000) - tmps >= 4 || (millis()/1000) < tmps)){
    sendMessage(1, "ping", 0);
    pingphase = 3;
    logV("ping:3");
    tmps = (millis()/1000);
  }
  if(pingphase == 3 && ((millis()/1000) - tmps >= 4 || (millis()/1000) < tmps)){
    pingphase = 0;
    String outgoingumap = exportEdgesContainingVertex(localAddress);
    logD("umap:" + outgoingumap);
    addValue(getValue(outgoingumap, ':', 1));
    sendMessage(1, outgoingumap, 0);
    logN("[PING] Découverte terminée — " + String(numEdges) + " lien(s) dans la table de routage");
    fpingdel = millis() / 1000;
  }

  if(startstat < 7 && startstat > 0 && maintmode == false){
    startprocedure();
  }

  if(prefs.getBool("incache", 0) == true && prefs.getBool("isgateonline", 0) == true && togateCount == 0 && ((millis()/1000) > (ltgdel + 2) || ltgdel > (millis()/1000))){
    ltgdel = (millis()/1000);
    exportcache();
  }

  if(infilecache == true && isfdeson == true && togateCountFile == 0 && ((millis()/1000) > (ltgdelfile + 2) || ltgdelfile > (millis()/1000))){
    ltgdelfile = (millis()/1000);
    filetxdelai = (millis()/1000);
    exportfile();
  }

  // --- Diffusion réseau : envoi périodique d'une ligne brdf (2s entre chaque ligne) ---
  if (broadcastEmitter && broadcastFileSending) {
    if (millis() >= lastBrdfSendMs + 2000 || millis() < lastBrdfSendMs) {
      lastBrdfSendMs = millis();
      broadcastExportLine();
    }
  }

  // --- Timeout 5 minutes en mode réception diffusion (stations réceptrices) ---
  if (broadcastMode && !broadcastEmitter) {
    unsigned long nowSec = millis() / 1000;
    if ((nowSec - lastBroadcastRecv) > 300 || (nowSec < lastBroadcastRecv)) {
      broadcastMode = false;
      lastBrdfSeq = -1;
      logV("diff:timeout rx");
    }
  }

  // --- Timeout du tunnel réseau E/S ---
  // Si aucun échange ne transite pendant NETIO_TIMEOUT ms, on ferme le tunnel proprement.
  if ((netioMaster || netioSlave) && netioLastActivity > 0 &&
      (millis() - netioLastActivity > (unsigned long)NETIO_TIMEOUT)) {
    int  savedRemote = netioRemote;
    bool wasMaster   = netioMaster;
    closeNetioTunnel();
    ioOutput("[NETIO] Timeout — tunnel fermé (inactivité)");
    // Notifier la station distante via trsp (paquet de fin fiable)
    interpreter("trsp:" + String(savedRemote) + ":nticlose");
  }

  if(pingphase == 0 && filesender == -1 && filereceivientstation == -1 && !broadcastMode && (startstat == 0 || startstat == 7 || startstat == 8) && entryCount == 0 && pingCount == 0 && togateCount == 0 && ((millis()/1000) - actiontimer >= actiontimerdel || (millis()/1000) < actiontimer && togateCount == 0)){
    if(stationstat == 0 && maintmode == false){      
      stationstat = 1;
      logN("[VEILLE] Aucune activité détectée, mise en veille imminente...");
      writetosd();
      lora.receive();
      delay(100);
      int nextwup = nextWakeup() - 5;
      logN("[VEILLE] Entrée en sommeil profond — réveil dans " + String(nextwup/60) + " min (" + String(nextwup) + "s)");
      if(nextwup > 0){        
        esp_sleep_enable_timer_wakeup((nextWakeup() - 5) * uS_TO_S_FACTOR);
      }
      esp_deep_sleep_enable_gpio_wakeup(1 << 1, ESP_GPIO_WAKEUP_GPIO_HIGH);
      gpio_set_direction((gpio_num_t)1, GPIO_MODE_INPUT);  // <<<=== Add this line
      esp_deep_sleep_start();
    }
  }
  else{
    if(stationstat == 1){
      logN("[RÉVEIL] Reprise de l'activité après la veille");
    }
    stationstat = 0;
  }
  if (filereceivientstation > -1 && (isfdeson == 0 || infilecache == 0) && (((millis()/1000) - filetxdelai) > filetxtimeout || filetxdelai > (millis()/1000))){
    filereceivientstation = -1;
    logN("[ERREUR] Transfert fichier échoué : le noeud destinataire ne répond plus");
  }
  if (filesender > -1 && (((millis()/1000) - filedelai) > filetimeout || filedelai > (millis()/1000))){
    filesender = -1;
    logN("[ERREUR] Réception fichier interrompue : timeout dépassé, transfert annulé");
  }
  if(millis() >= (lastair + 15) || millis() < lastair){
    checkDelayedCommands();
  }
  if(millis() >= (lastair + 1000) || millis() < lastair){
    checkAndRemoveOldEntries();
    checkAndRemoveOldPingEntries(); 
    executeCronTasks();   
  }
  
  togatePurgeOld();
  purgeToOldFile();
  removeExpiredValues();
  cpuIdle();
}

void scheduleCommand(unsigned long delayMs, const String& command) {
  unsigned long triggerTime = millis() + delayMs;

  for (int i = 0; i < MAX_COMMANDS; i++) {
    if (!commandQueue[i].active) {
      commandQueue[i].triggerTime = triggerTime;
      commandQueue[i].command = command;
      commandQueue[i].active = true;
      return;
    }
  }

  logN("[ERREUR] File de commandes interne pleine, commande ignorée");
}

void checkDelayedCommands() {
  unsigned long currentMillis = millis();
  String tmpcmdd;
  for (int i = 0; i < MAX_COMMANDS; i++) {
    if (commandQueue[i].active && currentMillis >= commandQueue[i].triggerTime) {
      tmpcmdd = commandQueue[i].command;
      commandQueue[i].active = false;
      interpreter(tmpcmdd);
      return;
    }
  }
}

void sendMessage(bool wake, String outgoing, int destination) {
  if(millis() < (lastair+15)){
    if(wake == true){
      String temptosend = "trsm:";
      temptosend += destination;
      temptosend += ":";
      temptosend += outgoing;
      scheduleCommand(15, temptosend);
    }
    else{
      String temptosend = "trsms:";
      temptosend += destination;
      temptosend += ":";
      temptosend += outgoing;
      scheduleCommand(15, temptosend);
    }
    return;
  }
  
  if(wake == true){
    uint8_t wakeBuf[] = {'w', 'a', 'k', 'e'};
    lora.transmit(wakeBuf, 4);
    lora.waitTransmitDone();
    lora.receive();
    delay(500);
  }

  uint8_t payloadLen = outgoing.length();
  uint8_t txBuf[255];
  txBuf[0] = (uint8_t)destination;
  txBuf[1] = (uint8_t)localAddress;
  txBuf[2] = msgCount;
  txBuf[3] = payloadLen;
  memcpy(&txBuf[4], outgoing.c_str(), payloadLen);
  lora.transmit(txBuf, 4 + payloadLen);
  lora.waitTransmitDone();
  lora.receive();
  msgCount++;                           // increment message ID
  txcnt ++;
  actiontimer = (millis()/1000);
  lastair = millis();
}

void onReceive() {
  uint8_t rxBuf[255];
  uint8_t packetSize = lora.readPacket(rxBuf, sizeof(rxBuf));
  if (packetSize == 0) return;
  lora.receive();

  actiontimer = (millis()/1000);

  // read packet header bytes:
  int recipient = rxBuf[0];
  sender = rxBuf[1];
  byte incomingMsgId = rxBuf[2];
  byte incomingLength = rxBuf[3];

  String incoming = "";
  for (uint8_t i = 4; i < packetSize; i++) {
    incoming += (char)rxBuf[i];
  }

  if (incomingLength != incoming.length()) {   // check length for error
    //Serial.println("error: message length does not match length");
    ;
    return;                             // skip rest of function
  }
  rxglob ++;
  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0) {    
    return;                             // skip rest of function
  }
  
  if (recipient == 0) {
    rxbrd ++;
    if (getValue(incoming, ':', 0) == "umap") {
      if (!findValue(getValue(incoming, ':', 1))) {
        int pcount = 3;
        removeEdgesByVertex(getValue(incoming, ':', 2).toInt());
        while (getValue(incoming, ':', pcount) != "") {
          addOrUpdateEdge(
            (getValue(getValue(incoming, ':', pcount), ',', 0)).toInt(),
            (getValue(getValue(incoming, ':', pcount), ',', 1)).toInt(),
            (getValue(getValue(incoming, ':', pcount), ',', 2)).toInt()
          );
          pcount++;
        }
        addValue(getValue(incoming, ':', 1));
        String rumap = "trsm:";
        rumap += "0:";
        rumap += incoming;
        scheduleCommand((20*localAddress), rumap);
    }
  }
    
    if(getValue(incoming, ':', 0) == "ping"){
      logV("ping:rx rssi=" + String(lora.packetRssi()));
      String rping = "trsms:";
      rping += String(sender);
      rping += ":rpin:";
      rping += String(lora.packetRssi());
      scheduleCommand((50*localAddress), rping);                         // skip rest of function
    }        
    if(getValue(incoming, ':', 0) == "difh"){
      if (!findValue(getValue(incoming, ':', 1))) {
        delay(1000-960);
        rtc.setTime(((getValue(incoming, ':', 2)).toInt())+1);
        logD("difh:rx set=" + getValue(incoming, ':', 2));
        addValue(getValue(incoming, ':', 1));
        delay(20*localAddress);
        String tosendtimestamp = "difh:";
        tosendtimestamp += getValue(incoming, ':', 1);
        tosendtimestamp += ":";
        tosendtimestamp += String(rtc.getLocalEpoch());
        logD("difh:fwd " + tosendtimestamp);
        sendMessage(1, tosendtimestamp, 0);
      }
   }

    // --- Diffusion réussite de compilation : bcok:ID:STATION:CHEMIN ---
    if (getValue(incoming, ':', 0) == "bcok") {
      String bcokid = getValue(incoming, ':', 1);
      if (!findValue(bcokid)) {
        addValue(bcokid);
        logV("bcok:station " + getValue(incoming, ':', 2) + " compile ok " + getValue(incoming, ':', 3));
        // Rétransmission sans réveil avec délai proportionnel au rang
        String rbcok = "trsms:0:";
        rbcok += incoming;
        scheduleCommand(20 * localAddress, rbcok);
      }
    }

    // --- Diffusion de mise à jour firmware : bupd:ID ---
    // Reçu par toutes les stations : planifie l'upgrade 60 s après réception et rétransmet
    if (getValue(incoming, ':', 0) == "bupd") {
      String bupdid = getValue(incoming, ':', 1);
      if (!findValue(bupdid)) {
        addValue(bupdid);
        logN("[MÀJU RÉSEAU] Déclenchement de mise à jour reçu — upgrade dans 30 s");
        scheduleCommand(30000, "upgrade");
        String rbupd = "trsms:0:";
        rbupd += incoming;
        scheduleCommand(20 * localAddress, rbupd);
      }
    }

    // --- Diffusion de version firmware après mise à jour : fwver:ID:VERSION:STATION ---
    // Reçu par toutes les stations après le redémarrage post-mise à jour d'une station
    if (getValue(incoming, ':', 0) == "fwver") {
      String fwverid = getValue(incoming, ':', 1);
      if (!findValue(fwverid)) {
        addValue(fwverid);
        logN("[MÀJU RÉSEAU] Station #" + getValue(incoming, ':', 3) + " tourne maintenant sur le firmware " + getValue(incoming, ':', 2));
        String rfwver = "trsms:0:";
        rfwver += incoming;
        scheduleCommand(20 * localAddress, rfwver);
      }
    }

    // --- Commande de verrouillage diffusion : brdl:ID:chemin ---
    if (getValue(incoming, ':', 0) == "brdl") {
      if (!findValue(getValue(incoming, ':', 1))) {
        addValue(getValue(incoming, ':', 1));
        if (!broadcastMode) {
          broadcastMode = true;
          broadcastPath = getValue(incoming, ':', 2);
          lastBroadcastRecv = millis() / 1000;
          lastBrdfSeq = -1;
          // Nettoyage rx.txt et préparation réception (via interpreter, hors ISR)
          scheduleCommand(10, "brdinit");
          logV("diff:rx on " + broadcastPath);
        }
        // Rétransmission avec délai proportionnel au rang
        String rbrdl = "trsm:0:";
        rbrdl += incoming;
        scheduleCommand(20 * localAddress, rbrdl);
      }
    }

    // --- Ligne de données diffusée : brdf:SEQ:LINEDATA ---
    if (getValue(incoming, ':', 0) == "brdf") {
      if (broadcastMode && !broadcastEmitter) {
        int seq = getValue(incoming, ':', 1).toInt();
        if (seq != lastBrdfSeq) {
          lastBrdfSeq = seq;
          lastBroadcastRecv = millis() / 1000;
          // Traitement complet (écriture SD + rétransmission) via interpreter
          scheduleCommand(5, incoming);
        }
      }
    }

    // --- Signal de fin de diffusion : brde:ID:chemin ---
    if (getValue(incoming, ':', 0) == "brde") {
      if (broadcastMode && !broadcastEmitter) {
        String brdeid = getValue(incoming, ':', 1);
        if (!findValue(brdeid)) {
          addValue(brdeid);
          // Rétransmission sans réveil
          String rbrde = "trsms:0:";
          rbrde += incoming;
          scheduleCommand(20 * localAddress, rbrde);
          // Compilation différée (laisser les rétransmissions se propager)
          scheduleCommand(2100, incoming);
        }
      }
    }

    return;
  }
  rxloc ++;
  scheduleCommand(15, incoming);
  lastair = millis();
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;
 
    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void acth(){
  delay(50);
  timegeth = millis();
  sendMessage(1, "geth", findNearestVertex(localAddress));
}

bool checkCronField(const String& field, int currentValue, int moduloBase) {
  if (field == "*") return true;
  if (field == "a") return currentValue == localAddress;
  if (field.startsWith("*/")) {
    int step = field.substring(2).toInt();
    if (step <= 0) return false; 
    return (currentValue % step == 0); 
  }
  int fieldVal = field.toInt();
  return fieldVal == currentValue;
}

void executeCronTasks() {
  if (broadcastMode) return;  // Bloquer les cronjobs pendant la diffusion réseau
  time_t now = rtc.getEpoch();
  struct tm timeinfo = rtc.getTimeStruct();
  if (timeinfo.tm_min == lastMinuteChecked) return;
  lastMinuteChecked = timeinfo.tm_min;
  
  if (timeinfo.tm_sec != 0) return; 

  int start = 0;
  int end = crontabString.indexOf(';');

  while (end != -1) {
    String cronEntry = crontabString.substring(start, end);
    cronEntry.trim();

    int nextStart = end + 1;
    end = crontabString.indexOf(';', nextStart);

    int lastSpace = cronEntry.lastIndexOf(' ');
    if (lastSpace == -1 || lastSpace == 0 || lastSpace == cronEntry.length() - 1) {
      logV("cron:err format " + cronEntry.substring(0, 20));
      start = nextStart;
      continue;
    }

    String timeFields = cronEntry.substring(0, lastSpace);
    String taskName = cronEntry.substring(lastSpace + 1);
    taskName.trim();

    String fields[5];
    int fieldCount = 0, currentPos = 0, lastPos = 0;

    while (currentPos < timeFields.length() && fieldCount < 5) {
      if (timeFields.charAt(currentPos) == ' ') {
        if (currentPos > lastPos) {
          fields[fieldCount++] = timeFields.substring(lastPos, currentPos);
          fields[fieldCount-1].trim();
        } else {
          fieldCount = -1;
          break;
        }
        lastPos = currentPos + 1;
      }
      currentPos++;
    }
    
    if (fieldCount >= 0 && fieldCount < 5 && lastPos < timeFields.length()) {
      fields[fieldCount++] = timeFields.substring(lastPos);
      fields[fieldCount-1].trim(); // Trim the last field
    }

    if (fieldCount != 5) {
      logV("cron:err champs=" + String(fieldCount) + " " + timeFields);
      start = nextStart;
      continue;
    }

    bool minuteMatch     = checkCronField(fields[0], timeinfo.tm_min, 60);
    bool hourMatch       = checkCronField(fields[1], timeinfo.tm_hour, 24);
    bool dayOfMonthMatch = checkCronField(fields[2], timeinfo.tm_mday, 31);
    bool monthMatch      = checkCronField(fields[3], timeinfo.tm_mon + 1, 12);
    bool dayOfWeekMatch  = checkCronField(fields[4], timeinfo.tm_wday, 7);

    if (minuteMatch && hourMatch && dayOfMonthMatch && monthMatch && dayOfWeekMatch) {
      interpreter(taskName); 
    }
    start = nextStart;
  }
}

bool cronFieldMatch(const String& field, int value) {
  if (field == "*") return true;
  if (field == "a") return value == localAddress;
  if (field.startsWith("*/")) {
    int step = field.substring(2).toInt();
    return step > 0 && (value % step == 0);
  }
  return field.toInt() == value; 
}

int nextWakeup() {
  time_t now = rtc.getEpoch();
  struct tm tm_candidate;
  struct tm tm_now = rtc.getTimeStruct(); 

  for (int offset = 1; offset <= 86400; offset++) {
    time_t candidate_time = now + offset;
    localtime_r(&candidate_time, &tm_candidate);

    if (tm_candidate.tm_sec != 0) {
        continue; 
    }

    int mm = tm_candidate.tm_min;
    int hh = tm_candidate.tm_hour;
    int jj = tm_candidate.tm_mday;
    int MM = tm_candidate.tm_mon + 1;
    int JJ = tm_candidate.tm_wday;

    int start = 0;
    int end = crontabString.indexOf(';');

    while (end != -1) {
      String cronEntry = crontabString.substring(start, end);
      cronEntry.trim();
      int nextStart = end + 1;
      end = crontabString.indexOf(';', nextStart);

      int lastSpace = cronEntry.lastIndexOf(' ');
      if (lastSpace == -1) {
        start = nextStart;
        continue;
      }

      String timeFields = cronEntry.substring(0, lastSpace);
      String fields[5]; // Expect 5 fields
      int fieldCount = 0, currentPos = 0, lastPos = 0;

      while (currentPos < timeFields.length() && fieldCount < 5) {
          if (timeFields.charAt(currentPos) == ' ') {
              if (currentPos > lastPos) {
                  fields[fieldCount++] = timeFields.substring(lastPos, currentPos);
                  fields[fieldCount-1].trim();
              } else {
                  fieldCount = -1; break;
              }
              lastPos = currentPos + 1;
          }
          currentPos++;
      }
      if (fieldCount >= 0 && fieldCount < 5 && lastPos < timeFields.length()) {
          fields[fieldCount++] = timeFields.substring(lastPos);
          fields[fieldCount-1].trim();
      }

      if (fieldCount != 5) {
        start = nextStart;
        continue;
      }

      if (
        // No seconds check
        cronFieldMatch(fields[0], mm) &&
        cronFieldMatch(fields[1], hh) &&
        cronFieldMatch(fields[2], jj) &&
        cronFieldMatch(fields[3], MM) &&
        cronFieldMatch(fields[4], JJ)
      ) {
        return offset; 
      }

      start = nextStart;
    }
  }
  return -1; 
}

void changepval(String parn, String parv){
  if(parn == "stationgateway"){
    stationgateway = parv.toInt();
    logN("[CONFIG] Paramètre 'stationgateway' mis à jour : " + String(stationgateway));
    return;
  }
  if(parn == "MAX_PING_AGE"){
    MAX_PING_AGE = parv.toInt();
    logN("[CONFIG] Paramètre 'MAX_PING_AGE' mis à jour : " + String(MAX_PING_AGE) + " ms");
  }
  if(parn == "starttimeout"){
    starttimeout = parv.toInt();
    logN("[CONFIG] Paramètre 'starttimeout' mis à jour : " + String(starttimeout) + " ms");
  }
  if(parn == "localAddress"){
    localAddress = parv.toInt();
    logN("[CONFIG] Paramètre 'localAddress' mis à jour : " + String(localAddress));
  }
  if(parn == "DELAY"){
    DELAY = parv.toInt();
    logN("[CONFIG] Paramètre 'DELAY' (déduplication) mis à jour : " + String(DELAY) + " ms");
  }
  if(parn == "MAX_ENTRY_AGE"){
    MAX_ENTRY_AGE = parv.toInt();
    logN("[CONFIG] Paramètre 'MAX_ENTRY_AGE' mis à jour : " + String(MAX_ENTRY_AGE) + " ms");
  }
  if(parn == "actiontimerdel"){
    actiontimerdel = parv.toInt();
    logN("[CONFIG] Délai d'inactivité avant veille mis à jour : " + String(actiontimerdel) + " s");
  }
  if(parn == "TOGATE_COMMAND_TIMEOUT"){
    TOGATE_COMMAND_TIMEOUT = parv.toInt();
    logN("[CONFIG] Timeout gateway mis à jour : " + String(TOGATE_COMMAND_TIMEOUT) + " ms");
  }
  if(parn == "serialLevel"){
    int sl = parv.toInt();
    if(sl >= LOG_NONE && sl <= LOG_DEBUG) serialLevel = sl;
    ioOutput("[CONFIG] Niveau de log via parm réglé sur " + String(serialLevel));
  }
  if(parn == "NETIO_TIMEOUT"){
    NETIO_TIMEOUT = parv.toInt();
    logN("[CONFIG] Timeout tunnel réseau E/S mis à jour : " + String(NETIO_TIMEOUT) + " ms");
  }
}

void interpreter(String msg){  
  String cmd = getValue(msg, ':', 0);
  actiontimer = (millis()/1000);

  if(cmd == "read"){
      readsd(1);
      logN("[OK] Carte réseau et paramètres rechargés depuis la carte SD");
  }
  if(cmd == "write"){
     writetosd();
     logN("[OK] Configuration sauvegardée sur la carte SD");
  }
  // ── getcfg : lit p.cfg et l'envoie sur le port actif (pour le configurateur web) ──
  if(cmd == "getcfg"){
    String cfg;
    cfg += MAX_PING_AGE / 1000;           cfg += ":";
    cfg += starttimeout / 1000;           cfg += ":";
    cfg += localAddress;                  cfg += ":";
    cfg += DELAY / 1000;                  cfg += ":";
    cfg += MAX_ENTRY_AGE / 1000;          cfg += ":";
    cfg += actiontimerdel;                cfg += ":";
    cfg += maintmode;                     cfg += ":";
    cfg += stationgateway;                cfg += ":";
    cfg += TOGATE_COMMAND_TIMEOUT / 1000; cfg += ":";
    cfg += serialLevel;                   cfg += ":";
    cfg += (ioMode == IO_NETIO ? ntioPrevIoMode : ioMode); cfg += ":";
    cfg += NETIO_TIMEOUT / 1000;          cfg += ":";
    cfg += filetimeout;                   cfg += ":";
    cfg += filetxtimeout;                 cfg += ":";
    ioOutput("CFG:" + cfg);
  }
  // ── setcfg : reçoit p.cfg depuis le configurateur, applique et sauvegarde ──
  // Format attendu : setcfg:MAX_PING_AGE:starttimeout:...:filetxtimeout:
  if(cmd == "setcfg"){
    int fieldCount = 0;
    for(int i = 0; i < (int)msg.length(); i++) if(msg[i] == ':') fieldCount++;
    if(fieldCount < 14){
      ioOutput("CFG:ERR:format invalide (" + String(fieldCount) + "/14 champs)");
    } else {
      MAX_PING_AGE           = getValue(msg,':',1).toInt() * 1000L;
      starttimeout           = getValue(msg,':',2).toInt() * 1000L;
      localAddress           = getValue(msg,':',3).toInt();
      DELAY                  = getValue(msg,':',4).toInt() * 1000L;
      MAX_ENTRY_AGE          = getValue(msg,':',5).toInt() * 1000L;
      actiontimerdel         = getValue(msg,':',6).toInt();
      maintmode              = getValue(msg,':',7).toInt();
      stationgateway         = getValue(msg,':',8).toInt();
      TOGATE_COMMAND_TIMEOUT = getValue(msg,':',9).toInt() * 1000L;
      { int sl = getValue(msg,':',10).toInt(); if(sl >= LOG_NONE && sl <= LOG_DEBUG) serialLevel = sl; }
      { int im = getValue(msg,':',11).toInt(); if(im >= IO_USB  && im <= IO_BLUETOOTH) ioMode = im; }
      { long nt = getValue(msg,':',12).toInt(); if(nt > 0) NETIO_TIMEOUT = nt * 1000L; }
      { int ft  = getValue(msg,':',13).toInt(); if(ft  > 0) filetimeout  = ft; }
      { int fxt = getValue(msg,':',14).toInt(); if(fxt > 0) filetxtimeout = fxt; }
      writetosd();
      ioOutput("CFG:OK");
    }
  }
  // ── getsensor : envoie l'état des capteurs au configurateur ──
  if(cmd == "getsensor"){
    String s;
    s += sensor_bme680 ? 1 : 0; s += ":";
    s += sensor_aht20  ? 1 : 0; s += ":";
    s += sensor_bmp280 ? 1 : 0; s += ":";
    s += sensor_ds18b20 ? 1 : 0; s += ":";
    ioOutput("SENSOR:" + s);
  }
  // ── setsensor : reçoit la config capteurs, applique et sauvegarde ──
  // Format attendu : setsensor:<bme680>:<aht20>:<bmp280>
  if(cmd == "setsensor"){
    int fieldCount = 0;
    for(int i = 0; i < (int)msg.length(); i++) if(msg[i] == ':') fieldCount++;
    if(fieldCount < 3){
      ioOutput("SENSOR:ERR:format invalide (" + String(fieldCount) + "/3 champs)");
    } else {
      sensor_bme680 = getValue(msg,':',1).toInt() == 1;
      sensor_aht20  = getValue(msg,':',2).toInt() == 1;
      sensor_bmp280 = getValue(msg,':',3).toInt() == 1;
      sensor_ds18b20 = getValue(msg,':',4).toInt() == 1;
      writeSensorCfg();
      ioOutput("SENSOR:OK");
    }
  }
  if(cmd == "data"){
    logN(msg.substring(5, msg.length()));
  }
    if(cmd == "trsm"){
     delay(10);
     sendMessage(1, msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length()), (getValue(msg, ':', 1)).toInt());
  }
  if(cmd == "trsms"){
      delay(10);
     sendMessage(0, msg.substring(6+((getValue(msg, ':', 1)).length()+1), msg.length()), (getValue(msg, ':', 1)).toInt());
  }
    if(cmd == "ping"){
      logV("ping:rx rssi=" + String(lora.packetRssi()));
      String rping = "trsms:";
      rping += String(sender);
      rping += ":rpin:";
      rping += String(lora.packetRssi());
      scheduleCommand(50, rping);
    }
    if(cmd == "rpin"){
      int glob = abs(((lora.packetRssi())+((msg.substring(5, msg.length())).toInt()))/2);
      if(serialLevel >= LOG_DEBUG)
        logD("rpin:" + String(sender) + " tx=" + msg.substring(5,msg.length()) + " rx=" + String(lora.packetRssi()) + " glob=" + String(glob));
      else
        logV("rpin:" + String(sender) + " glob=" + String(glob));
      addOrUpdateEdge(localAddress, sender, glob);
      removePingEntryByNbtoping(sender);
    }
    if (cmd == "umap") {
      if (!findValue(getValue(msg, ':', 1))) {
        int pcount = 3;
        removeEdgesByVertex(getValue(msg, ':', 2).toInt());
        while (getValue(msg, ':', pcount) != "") {
          addOrUpdateEdge(
            (getValue(getValue(msg, ':', pcount), ',', 0)).toInt(),
            (getValue(getValue(msg, ':', pcount), ',', 1)).toInt(),
            (getValue(getValue(msg, ':', pcount), ',', 2)).toInt()
          );
          pcount++;
        }
        addValue(getValue(msg, ':', 1));
        sendMessage(1, msg, 0);
    }  
  }
  if(cmd == "prnt"){
      printEdges();
    }
  if(cmd == "pigo"){
      removeEdgesByVertex(localAddress);
      sendMessage(1, "ping", 0);
      pingphase = 1;
      logN("[PING] Envoi des sondes de découverte réseau...");
      tmps = (millis()/1000);
    }  
    if(cmd == "dijk"){
      if(getValue(msg, ':', 2).toInt() != localAddress){
        dijkrx ++;
        String tempinload = (msg.substring((5+9+(getValue(msg, ':', 1).length())+(getValue(msg, ':', 2).length())+(getValue(msg, ':', 3).length())+3), msg.length()));
        logD("dijk:rx id=" + getValue(msg, ':', 4) + " len=" + String(tempinload.length()) + "/" + getValue(msg, ':', 3));
        String rxok = "trsms:";
        rxok += getValue(msg, ':', 2);
        rxok += ":rxok:";
        rxok += getValue(msg, ':', 4);
        rxok += ":";
        rxok += localAddress;
        if(tempinload.length() == getValue(msg, ':', 3).toInt()){
          scheduleCommand(30, rxok);
          if(getValue(msg, ':', 1).toInt() == localAddress){
            scheduleCommand(80, tempinload);
          }
        }
      }
      if(getValue(msg, ':', 1).toInt() != localAddress){
        dijkhop ++;
        String msgrt = "dijk:";
        msgrt += getValue(msg, ':', 1);
        msgrt += ":";
        msgrt += localAddress;
        msgrt += ":";
        msgrt += getValue(msg, ':', 3);
        msgrt += ":";
        msgrt += getValue(msg, ':', 4);
        msgrt += ":";
        msgrt += msg.substring((5+9+(getValue(msg, ':', 1).length())+(getValue(msg, ':', 2).length())+(getValue(msg, ':', 3).length())+3), msg.length());
        logD("dijk:fwd " + msgrt);
        if(dijkstra(localAddress, getValue(msg, ':', 1).toInt(), msgrt)){
          logD("dijk:ok ->" + getValue(msg, ':', 1));
          addEntry(getValue(msg, ':', 4), 1, msgrt);
        }
      }
    }
    if(cmd == "expv"){  
      for (int r = 0; r <= MAX_VERTICES;) {      
        String tmapCommand = "tmap";
        bool good = 0;
        for (int i = 0; i < numEdges; i++) {
          if (edges[i].vertex1 == r || edges[i].vertex2 == r) {
            good = 1;
            tmapCommand += ":";
            tmapCommand += edges[i].vertex1;
            tmapCommand += ",";
            tmapCommand += edges[i].vertex2;
            tmapCommand += ",";
            tmapCommand += edges[i].weight;
          }
        } 
      if(good == true){
        logN(tmapCommand);
      }
      r++;
      }

    }
    if(cmd == "load"){
      String tempinload = msg.substring((5+9+(getValue(msg, ':', 1).length())+(getValue(msg, ':', 2).length())+2), msg.length());
      logD("load:id=" + getValue(msg, ':', 3) + " len=" + String(tempinload.length()) + "/" + getValue(msg, ':', 2));
      String rxok = "send:";
      rxok += getValue(msg, ':', 1);
      rxok += ":arok:";
      rxok += getValue(msg, ':', 3);
      rxok += ":";
      rxok += localAddress;
      if(tempinload.length() == getValue(msg, ':', 2).toInt()){
        scheduleCommand(30, rxok);
        scheduleCommand(80, tempinload);
      }
    }
    if(cmd == "arok"){
      logN("[MESSAGE] Accusé de réception — message #" + getValue(msg, ':', 1) + " bien livré au noeud #" + getValue(msg, ':', 2));
      togateRemoveById(getValue(msg, ':', 1).toInt());
      togateRemoveByIdFile(getValue(msg, ':', 1).toInt());
    }
    if(cmd == "rxok"){
      logV("msg:hop id=" + getValue(msg, ':', 1) + " via " + getValue(msg, ':', 2));
      removeEntryByID(getValue(msg, ':', 1));
    }
    if(cmd == "trsp"){
      String tempid = generateid();
      String load = "send:";
      load += (getValue(msg, ':', 1)).toInt();
      load += ":load:";
      load += localAddress;
      load += ":";
      load += (msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length())).length();
      load += ":";
      load += tempid;
      load += ":";
      load += msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length()), (getValue(msg, ':', 1)).toInt();
      logD("trsp:" + load);
      interpreter(load);
    }
    if(cmd == "send"){
      if((msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length())).length() >= 225){
        String dijk;
        dijk += msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length()), (getValue(msg, ':', 1)).toInt();
        logD("send:large ->" + getValue(msg, ':', 1));
        large(dijk, getValue(msg, ':', 1).toInt());
      }
      else{
        dijktx ++;
        String tempid = generateid();
        String dijk = "dijk:";
        dijk += getValue(msg, ':', 1).toInt();
        dijk += ":";
        dijk += localAddress;
        dijk += ":";
        dijk += (msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length())).length();
        dijk += ":";
        dijk += tempid;
        dijk += ":";
        dijk += msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length()), (getValue(msg, ':', 1)).toInt();
        logD("send:dijk " + dijk);
        interpreter(dijk);        
      }
    }
    if(cmd == "gmap"){ 
      for (int r = 0; r <= MAX_VERTICES;) {      
        String tmapCommand = "tmap";
        bool good = 0;
        for (int i = 0; i < numEdges; i++) {
          if (edges[i].vertex1 == r || edges[i].vertex2 == r) {
            good = 1;
            tmapCommand += ":";
            tmapCommand += edges[i].vertex1;
            tmapCommand += ",";
            tmapCommand += edges[i].vertex2;
            tmapCommand += ",";
            tmapCommand += edges[i].weight;
          }
        } 
      if(good == true){
        delay(50);
        sendMessage(0, tmapCommand, sender);
        logD("gmap:tx " + tmapCommand);
      }
      r++;     
      }
    }
    if (cmd == "tmap") {
      if(startstat == 4){        
        startstat = 5;
        rtmapdel = millis()/1000;
      }
      int pcount = 1;
      while (getValue(msg, ':', pcount) != "") {
        addOrUpdateEdge(
          (getValue(getValue(msg, ':', pcount), ',', 0)).toInt(),
          (getValue(getValue(msg, ':', pcount), ',', 1)).toInt(),
          (getValue(getValue(msg, ':', pcount), ',', 2)).toInt()
        );
        pcount++;
      }
    }
    if(cmd == "time"){
      logN(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
    }
    if(cmd == "seth"){
      unsigned long delaygeth = ((((millis() - timegeth) - 50) / 2));
      logD("seth:delay=" + String(delaygeth));
      delay(1000 - delaygeth);
      rtc.setTime(((getValue(msg, ':', 1)).toInt()) + 1);
      logV("seth:set=" + getValue(msg, ':', 1));
      if(startstat == 7){
        startstat = 8;
        logN("[OK] Noeud #" + String(localAddress) + " opérationnel et connecté au réseau");
      }
    }
    if(cmd == "geth"){
      delay(50);
      String tosendtimestamp = "seth:";
      tosendtimestamp += String(rtc.getLocalEpoch());
      logD("geth:rep " + tosendtimestamp + " ->" + String(sender));
      sendMessage(0, tosendtimestamp, sender);
    }
    if(cmd == "acth"){
      acth();
    }
    if(cmd == "prgh"){
      rtc.setTime((getValue(msg, ':', 1)).toInt());
    }
    if(cmd == "star"){
      startstat = 1;
    }
    if(cmd == "sho"){
      logN("[MODE] Maintenance actuellement " + String(maintmode ? "activée (veille désactivée)" : "désactivée (mode nominal)"));
    }
    if(cmd == "maint"){
      maintmode = true;
      logN("[MODE] Passage en mode maintenance — la veille automatique est désactivée");
    }
    if(cmd == "norm"){
      maintmode = false;
      logN("[MODE] Passage en mode nominal — la veille automatique est activée");
    }
    if(cmd == "stam"){
      rtc.setTime((getValue(msg, ':', 1)).toInt());
      startstat = 8;
      logN("[OK] Noeud #" + String(localAddress) + " synchronisé et opérationnel");
    }
    if(cmd == "fdih"){
      delay(10);
      timegeth = millis();
      sendMessage(1, "geth", sender);
    }    
    if(cmd == "difh"){
      if (!findValue(getValue(msg, ':', 1))) {
        delay(40);
        rtc.setTime(((getValue(msg, ':', 2)).toInt())+1);
        logV("difh:set=" + getValue(msg, ':', 2));
        addValue(getValue(msg, ':', 1));
        delay(20*localAddress);
        String tosendtimestamp = "difh:";
        tosendtimestamp += getValue(msg, ':', 1);
        tosendtimestamp += ":";
        tosendtimestamp += String(rtc.getLocalEpoch());
        logD("difh:fwd " + tosendtimestamp);
        sendMessage(1, tosendtimestamp, 0);
      }
   }
   if(cmd == "gate"){
      logN("[GATEWAY] Adresse de la passerelle réseau : noeud #" + String(stationgateway));
   }
   if(cmd == "gdfh"){
    String tempid = generateid();
    String tosenddifh = "difh:";
    tosenddifh += tempid;
    tosenddifh += ":";
    tosenddifh += String(rtc.getLocalEpoch());
    sendMessage(1, tosenddifh, 0);
    logD("gdfh:" + tosenddifh);
    }
    if(cmd == "dexp"){
      String togate = exportdata(getValue(msg, ':', 1));
      logD("dexp:" + togate);
      String tempid = generateid();
      String load = "send:";
      load += stationgateway;
      load += ":load:";
      load += localAddress;
      load += ":";
      load += togate.length();
      load += ":";
      load += tempid;
      load += ":";
      load += togate;
      logD("dexp:load " + load);
      togateAddCommand(tempid.toInt(), load);
      interpreter(load);
    }
    if(cmd == "gmea"){
      measuretodump(getValue(msg, ':', 1).toInt());
    }
    if(cmd == "parm"){
      changepval(getValue(msg, ':', 1), getValue(msg, ':', 2));
    }
    if(cmd == "slvl"){
      String arg = getValue(msg, ':', 1);
      if(arg == "none")         serialLevel = LOG_NONE;
      else if(arg == "normal")  serialLevel = LOG_NORMAL;
      else if(arg == "verbose") serialLevel = LOG_VERBOSE;
      else if(arg == "debug")   serialLevel = LOG_DEBUG;
      else { int sl = arg.toInt(); if(sl >= LOG_NONE && sl <= LOG_DEBUG) serialLevel = sl; }
      // En mode esclave netio, le niveau est contraint à [1, 2] :
      // 0 (silencieux) priverait le maître de toute sortie,
      // 3 (debug) inonderait le tunnel de messages internes.
      if(netioSlave) serialLevel = constrain(serialLevel, LOG_NORMAL, LOG_VERBOSE);
      ioOutput("[CONFIG] Niveau de log réglé sur '" + arg + "' (" + String(serialLevel) + ")");
      writetosd();
    }
    if(cmd == "iomd"){
      String arg = getValue(msg, ':', 1);
      if(arg == "usb"){
        if(ioMode == IO_BLUETOOTH) stopBLE();
        ioMode = IO_USB;
        ioOutput("[IO] Mode USB activé");
        writetosd();
      } else if(arg == "bt"){
        if(ioMode != IO_BLUETOOTH){
          ioMode = IO_BLUETOOTH;
          startBLE();
          writetosd();
        }
        ioOutput("[IO] Mode Bluetooth activé — appareil: Mycromesh-" + String(localAddress));
      } else {
        ioOutput("[IO] Mode inconnu. Commandes: iomd:usb  iomd:bt  (netio:<addr> pour le tunnel réseau)");
      }
    }
    if(cmd == "cout"){
      logN("[GATEWAY] Envoi de la prochaine entrée du cache vers la passerelle...");
      exportcache();
    }
    if(cmd == "clear"){
      int prevEdges = numEdges;
      clearEdges();
      logN("[OK] Table de routage effacée (" + String(prevEdges) + " lien(s) supprimé(s))");
    }
    if(cmd == "cachestate"){
      logN("[GATEWAY] Cache actif : " + String(prefs.getBool("incache",0) ? "oui" : "non") + "  |  passerelle en ligne : " + String(prefs.getBool("isgateonline",0) ? "oui" : "non") + "  |  commandes en attente : " + String(togateCount));
    }
    if(cmd == "adrc"){
    int del = getValue(msg, ':', 1).toInt();
    String cmd = getValue(msg, ':', 2);
    logD("adrc:" + String(del) + "ms cmd=" + cmd);
    scheduleCommand(del, cmd);
    }
    if(cmd == "startfile"){
      filereceivientstation = getValue(msg, ':', 1).toInt();
      infilecache = true;
      isfdeson = true;
      prefs.putUInt("offsetfile", 0);
      nbtogatefailfile = 0;
      logN("[FICHIER TX] Initialisation du transfert vers le noeud #" + String(filereceivientstation) + "...");
    }
    if(cmd == "filestate"){
      logN("[FICHIER TX] État — cache actif : " + String(infilecache ? "oui" : "non") + "  |  envoi actif : " + String(isfdeson ? "oui" : "non") + "  |  lignes en attente : " + String(togateCountFile));
    }
    if(cmd == "stopfile"){
      isfdeson = false;
      infilecache = false;
      logN("[FICHIER TX] Transfert de fichier arrêté manuellement");
    }
    if(cmd == "expfile"){
      logN("[FICHIER TX] Envoi de la prochaine ligne du fichier en cache...");
      exportfile();
    }
    if(cmd == "file"){
      filedelai = (millis()/1000);
      logD("file:rx " + msg.substring(5, msg.length()));
      importfile("/rx.txt", msg.substring(5, msg.length()));
    }
    if(cmd == "stft"){
      if(filereceivientstation == -1 && filesender == -1 && !broadcastMode){
        remof = getValue(msg, ':', 3).toInt();
        ftfpath = getValue(msg, ':', 2);
        if (parseFile(ftfpath)){
          infilecache = true;
          filereceivientstation = getValue(msg, ':', 1).toInt();
          filetxdelai = (millis()/1000);
          logN("[FICHIER TX] Fichier " + ftfpath + " prêt, connexion avec le noeud #" + String(filereceivientstation) + " en cours...");
          String tempisrf = "send:";
          tempisrf += filereceivientstation;
          tempisrf += ":isrf:";
          tempisrf += localAddress;
          scheduleCommand(800, tempisrf);
        }
      }
    }
    if(cmd == "isrf"){
      if(filereceivientstation == -1 && filesender == -1 && !broadcastMode){
        filedelai = (millis()/1000);
        filesender = getValue(msg, ':', 1).toInt();
        logN("[FICHIER RX] Début de réception d'un fichier depuis le noeud #" + String(filesender));
        remfromsd("/rx.txt");
        String temprfok = "send:";
        temprfok += filesender;
        temprfok += ":rfok:";
        temprfok += localAddress;
        scheduleCommand(800, temprfok);
      }      
    }
    if(cmd == "rfok"){
      if(getValue(msg, ':', 1).toInt() == filereceivientstation && isfdeson == false){
        logN("[FICHIER TX] Noeud #" + String(filereceivientstation) + " prêt, envoi de " + ftfpath + " en cours...");
        isfdeson = true;
        prefs.putUInt("offsetfile", 0);
        nbtogatefailfile = 0;
        ltgdelfile = (millis()/1000);
      }
    }
    
    if(cmd == "fend"){
      if(getValue(msg, ':', 1).toInt() == filesender){
        logN("[FICHIER RX] Fin de réception depuis le noeud #" + String(filesender) + ", compilation vers " + getValue(msg, ':', 2) + "...");
        String tempfntc = "compileFile:";
        tempfntc += getValue(msg, ':', 2);
        tempfntc += ":";
        tempfntc += filesender;
        tempfntc += ":";
        tempfntc += getValue(msg, ':', 3);
        filesender = -1;
        scheduleCommand(2000, tempfntc);
      }
    }
    if(cmd == "compileFile"){
      compileFile(getValue(msg, ':', 1), getValue(msg, ':', 2).toInt(), getValue(msg, ':', 3).toInt());      
    }
    if(cmd == "feok"){
      logN("[FICHIER TX] Confirmation reçue — " + getValue(msg, ':', 2) + " bien reçu par le noeud #" + getValue(msg, ':', 1));
      if(getValue(msg, ':', 3) == "1"){
        logD("feok:rm " + getValue(msg, ':', 2));
        remfromsd(getValue(msg, ':', 2));
      }
    }
    if(cmd == "reboot"){
      logN("[SYSTÈME] Redémarrage du noeud en cours...");
      writetosd();
      delay(500);
      ESP.restart();
    }
    if(cmd == "upgrade"){
      logN("[MISE À JOUR] Lancement de la mise à jour du firmware...");
      writetosd();
      delay(500);
      doFirmwareUpdate();
    }
    if(cmd == "version"){
      logN("[SYSTÈME] Version du firmware : " + FIRMWARE_VERSION);
    }
    if(cmd == "mkdir"){
      String path = getValue(msg, ':', 1);
      logN("[SD] Dossier " + path + (mkdirsd(path) ? " créé avec succès" : " — erreur de création"));
    }
    if(cmd == "rmdir"){
      String path = getValue(msg, ':', 1);
      logN("[SD] Dossier " + path + (rmdirsd(path) ? " supprimé" : " — erreur de suppression"));
    }
    if(cmd == "rm"){
      String path = getValue(msg, ':', 1);
      logN("[SD] Fichier " + path + (remfromsd(path) ? " supprimé" : " — erreur de suppression"));
    }
    if(cmd == "cp"){
      String src = getValue(msg, ':', 1);
      String dstDir = getValue(msg, ':', 2);
      String fname = src.substring(src.lastIndexOf('/'));
      String dst = dstDir.endsWith("/") ? dstDir + fname.substring(1) : dstDir + fname;
      logN("[SD] Copie " + src + " -> " + dst + (cpsd(src, dst) ? " : succès" : " : erreur"));
    }
    if(cmd == "mv"){
      String src = getValue(msg, ':', 1);
      String dstDir = getValue(msg, ':', 2);
      String fname = src.substring(src.lastIndexOf('/'));
      String dst = dstDir.endsWith("/") ? dstDir + fname.substring(1) : dstDir + fname;
      logN("[SD] Déplacement " + src + " -> " + dst + (mvsd(src, dst) ? " : succès" : " : erreur"));
    }
    if(cmd == "ls"){
      String path = getValue(msg, ':', 1);
      if(path == "") path = "/";
      lssd(path);
    }

    // Diffusion de la réussite de compilation (schedulé 5 min après compileFile OK)
    // Format diffusé : bcok:ID:STATION:CHEMIN
    if (cmd == "bcok") {
      String tempid = generateid();
      String bcok = "bcok:";
      bcok += tempid;
      bcok += ":";
      bcok += localAddress;
      bcok += ":";
      bcok += getValue(msg, ':', 1);  // chemin du fichier compilé
      addValue(tempid);  // Éviter de retraiter notre propre bcok si reçu en écho
      sendMessage(0, bcok, 0);  // Diffusion sans réveil
      logV("bcok:" + getValue(msg, ':', 1));
    }

    // ================================================================
    // DIFFUSION DE MISE À JOUR FIRMWARE : bupd
    // Déclenche sur l'ensemble du réseau une mise à jour 60 s après réception
    // ================================================================
    if (cmd == "bupd") {
      String tempid = generateid();
      String bupd = "bupd:";
      bupd += tempid;
      addValue(tempid);
      sendMessage(0, bupd, 0);  // Diffusion sans réveil
      logN("[MÀJU RÉSEAU] Diffusion de mise à jour envoyée — chaque station upgradée dans 60 s");
      scheduleCommand(30000, "upgrade");  // La station émettrice upgradera également
    }

    // ================================================================
    // DIFFUSION DE VERSION FIRMWARE : fwver
    // Diffuse la version du firmware actuel à l'ensemble du réseau
    // (planifié automatiquement 5 min après une mise à jour réussie)
    // ================================================================
    if (cmd == "fwver") {
      String tempid = generateid();
      String fwver = "fwver:";
      fwver += tempid;
      fwver += ":";
      fwver += FIRMWARE_VERSION;
      fwver += ":";
      fwver += localAddress;
      addValue(tempid);
      sendMessage(0, fwver, 0);  // Diffusion sans réveil
      logN("[MÀJU RÉSEAU] Diffusion de version firmware : " + FIRMWARE_VERSION);
    }

    // ================================================================
    // DIFFUSION RÉSEAU : diff:chemin/fichier
    // Diffuse un fichier vers l'ensemble du réseau (broadcast)
    // ================================================================
    if (cmd == "diff") {
      if (!broadcastMode && filereceivientstation == -1 && filesender == -1) {
        broadcastMode = true;
        broadcastEmitter = true;
        broadcastPath = getValue(msg, ':', 1);
        lastBroadcastRecv = millis() / 1000;
        lastBrdfSeq = -1;
        brdfSeqCounter = 0;
        logV("diff:start " + broadcastPath);
        if (parseFile(broadcastPath)) {
          prefs.putUInt("brdoffset", 0);
          String tempid = generateid();
          String brdl = "brdl:";
          brdl += tempid;
          brdl += ":";
          brdl += broadcastPath;
          addValue(tempid);
          sendMessage(1, brdl, 0);
          logD("diff:brdl envoyé");
          scheduleCommand(5000, "brdfstart");
        } else {
          logN("[ERREUR] Diffusion réseau : impossible de parser le fichier source");
          broadcastMode = false;
          broadcastEmitter = false;
        }
      } else {
        logN("[ERREUR] Diffusion impossible : un transfert ou une diffusion est déjà en cours");
      }
    }

    // Déclenchement de l'envoi des lignes brdf (après délai propagation brdl)
    if (cmd == "brdfstart") {
      if (broadcastEmitter) {
        broadcastFileSending = true;
        lastBrdfSendMs = 0;
        logV("diff:envoi lignes");
      }
    }

    if (cmd == "brdinit") {
      remfromsd("/rx.txt");
      logD("diff:rx init");
    }

    // Réception d'une ligne de données diffusée : brdf:SEQ:LINEDATA
    if (cmd == "brdf") {
      if (broadcastMode && !broadcastEmitter) {
        // Extraction de LINEDATA en évitant les problèmes de split sur ':' dans les données
        String seqStr = getValue(msg, ':', 1);
        int lineStart = 5 + seqStr.length() + 1;  // "brdf:" + seqStr + ":"
        String lineData = msg.substring(lineStart);
        if (lineData.length() > 0) {
          importfile("/rx.txt", lineData);
          // Rétransmission sans réveil avec délai proportionnel au rang
          String rbrdf = "trsms:0:";
          rbrdf += msg;
          scheduleCommand(20 * localAddress, rbrdf);
        }
      }
    }

    // Réception du signal de fin de diffusion : brde:ID:chemin
    if (cmd == "brde") {
      if (broadcastMode && !broadcastEmitter) {
        String filePath = getValue(msg, ':', 2);
        logV("diff:rx fin -> " + filePath);
        broadcastMode = false;
        lastBrdfSeq = -1;
        compileFile(filePath, -1, 0);  // origin=-1 : pas de feok à envoyer
        logV("diff:rx done");
      }
    }

    // ================================================================
    // MODE RÉSEAU E/S (netio) — tunnel maître/esclave via le maillage
    // ================================================================

    // Commande utilisateur : ouvre un tunnel vers l'esclave <addr>
    // Format : netio:<addr>
    if (cmd == "netio") {
      int slaveAddr = getValue(msg, ':', 1).toInt();
      if (slaveAddr <= 0) {
        ioOutput("[NETIO] Erreur : adresse esclave invalide");
      } else if (netioMaster || netioSlave) {
        ioOutput("[NETIO] Un tunnel réseau est déjà actif");
      } else {
        ioOutput("[NETIO] Ouverture du tunnel vers le noeud #" + String(slaveAddr) + "...");
        ntioPrevIoMode    = ioMode;
        ntioPrevMaintMode = maintmode;
        netioRemote       = slaveAddr;
        maintmode         = true;
        netioLastActivity = millis();
        // Paquet d'initialisation envoyé via trsp (fiable, avec accusé de réception)
        interpreter("trsp:" + String(slaveAddr) + ":ntiopen:" + String(localAddress));
        // Passer en mode maître après l'envoi pour ne pas supprimer les logs de l'init
        netioMaster = true;
        ioMode      = IO_NETIO;
      }
    }

    // Reçu par l'esclave : le maître demande l'ouverture du tunnel
    // Format : ntiopen:<master_addr>
    if (cmd == "ntiopen") {
      int masterAddr = getValue(msg, ':', 1).toInt();
      if (!netioMaster && !netioSlave && masterAddr > 0) {
        ntioPrevIoMode    = ioMode;
        ntioPrevMaintMode = maintmode;
        netioRemote       = masterAddr;
        maintmode         = true;
        netioLastActivity = millis();
        logN("[NETIO] Mode esclave activé — maître : noeud #" + String(masterAddr));
        // Confirmer au maître via trsp (fiable) avant de basculer le mode
        interpreter("trsp:" + String(masterAddr) + ":ntiok");
        // Basculer en mode esclave
        netioSlave = true;
        ioMode     = IO_NETIO;
      }
    }

    // Reçu par le maître : l'esclave confirme que le tunnel est établi
    // Format : ntiok
    if (cmd == "ntiok") {
      if (netioMaster && netioRemote >= 0) {
        netioLastActivity = millis();
        netioDisplay("[NETIO] Tunnel ouvert avec le noeud #" + String(netioRemote) + " — tapez 'exit' pour fermer");
      }
    }

    // Reçu par l'esclave : commande à exécuter localement, envoyée par le maître
    // Format : ntidata:<commande>
    if (cmd == "ntidata") {
      if (netioSlave) {
        netioLastActivity = millis();
        String subcmd = msg.substring(8); // retire le préfixe "ntidata:"
        interpreter(subcmd);
      }
    }

    // Reçu par le maître : réponse de l'esclave à afficher localement
    // Format : ntirsp:<message>
    if (cmd == "ntirsp") {
      if (netioMaster) {
        netioLastActivity = millis();
        netioDisplay(msg.substring(7)); // retire le préfixe "ntirsp:"
      }
    }

    // Reçu par l'un ou l'autre : fermeture du tunnel (depuis timeout ou commande distante)
    // Format : nticlose
    if (cmd == "nticlose") {
      if (netioMaster || netioSlave) {
        closeNetioTunnel();
        ioOutput("[NETIO] Tunnel réseau fermé");
      }
    }
}
