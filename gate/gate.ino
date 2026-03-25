#include "LiteLora.h"
#include <limits.h>
#include <ESP32Time.h>

#include "driver/gpio.h"
#include <SD.h>
#include <time.h>
#include <Preferences.h>
#include <Update.h>
#include <Wire.h>
#include <WiFi.h>
#include <ETH.h>
Preferences prefs;

ESP32Time rtc;

LiteLora lora;

const String FIRMWARE_VERSION = "1.4.0";



#define PACKET_SIZE 180
#define GROUP_K 32
#define PARITY_M 6
#define CHUNK_SIZE 90  // 180/90=2 chunks exacts par paquet, laisse place au CRC16

#define UPDATE_FILE "/update/firmware.bin"
#define BUF_SIZE 4096


static uint8_t gf_exp[512];
static uint8_t gf_log_tbl[256];

// Variable Cron

String crontabString;

// Variable de parametres

long MAX_PING_AGE = 20000;      // Duree maximale (en millisecondes) avant traitement d'une entree - stocke en ms, p.cfg en secondes
int localAddress = 61;          // address of this device
long DELAY = 3600000;           // Delai en millisecondes - stocke en ms, p.cfg en secondes
long MAX_ENTRY_AGE = 20000;     // Duree maximale (en millisecondes) avant traitement d'une entree - stocke en ms, p.cfg en secondes

int stationgateway = 1;
long TOGATE_COMMAND_TIMEOUT = 60000;  // stocke en ms, p.cfg en secondes
long NETIO_TIMEOUT     = 300000;      // ms sans activite avant fermeture automatique du tunnel - stocke en ms, p.cfg en secondes
int filetimeout = 300;
int filetxtimeout = 60;

// Niveaux de sortie serie : 0=rien, 1=normal, 2=verbose, 3=debug
#define LOG_NONE    0
#define LOG_NORMAL  1
#define LOG_VERBOSE 2
#define LOG_DEBUG   3
int serialLevel = 1;

// Mode E/S : selectionne le canal actif pour ioOutput / handleSerialInput
#define IO_USB       0
// IO_BLUETOOTH = 1 (reserve pour ajout futur)
#define IO_NETIO     2   // tunnel reseau : entree/sortie via une autre station du maillage
int ioMode = IO_USB;

// Variables du tunnel reseau E/S (mode IO_NETIO)
bool          netioMaster       = false;   // cette station est le maitre du tunnel
int           netioRemote       = -1;      // adresse de la station distante
unsigned long netioLastActivity = 0;       // horodatage de la derniere activite (millis)
int           ntioPrevIoMode    = IO_USB;  // mode E/S sauvegarde avant activation du tunnel


// variables d'etat


byte msgCount = 0;            // count of outgoing messages

// variables d'environnement

const int TAILLE_TAMPON = 256;
const int MAX_EDGES = 300;      // nombre max de liaisons
const int MAX_VERTICES = 60;    // nombre max de stations
const int MAX_ENTRIES = 10;     // Nombre maximum d'entrees
const int MAX_PINGS = 10;       // Nombre maximum d'entrees
const int MAX_SIZE = 10;        // Taille maximale du tableau de stockage
const int MAX_TOGATE_COMMANDS = 10;
const int MAX_TOGATE_COMMANDS_FILE = 20;

const int csPin = 15;          // LoRa radio chip select (io15)
const int irqPin = 4;          // LoRa DIO0 interrupt pin (io4)
const int busMuxPin = 17;      // LoRa/SD bus mux control (io17)

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
int lastMinuteChecked = -1;
unsigned long timegeth = 0;
int pingCount = 0; // Nombre actuel d'entrees dans la liste
int dataCount = 0;        // Compteur de valeurs stockees
unsigned long lastAddTime = 0; // Temps de la derniere addition
int sender;
int pingphase = 0;
unsigned long tmps = 0;
int numEdges = 0;
int numVertices = 0;

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

// ============================================================
// Phase 1 — Reseau (Wi-Fi + Ethernet LAN8720)
// ============================================================

// --- Broches Ethernet LAN8720 ---
// GPIO12 (SPI MISO LoRa/SD) vs ETH_PHY_POWER :
//   Si GPIO12 est utilise comme MISO, le driver ETH qui le pilote en sortie
//   provoque un conflit. ETH_PHY_POWER = -1 desactive le controle de puissance
//   (le LAN8720 reste alimente en permanence depuis son circuit d'alimentation).
#define ETH_PHY_ADDR     1
#define ETH_PHY_POWER    -1
#define ETH_PHY_MDC      23
#define ETH_PHY_MDIO     18
#define ETH_PHY_TYPE     ETH_PHY_LAN8720
#define NET_ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
                                              // (evite le conflit avec GPIO17 / busMuxPin)

// --- Credentials Wi-Fi (charges depuis /wifi.cfg sur SD) ---
String wifiSSID     = "";
String wifiPassword = "";

// --- Etat des interfaces ---
bool     ethConnected  = false;
bool     wifiConnected = false;
enum NetIface { IFACE_NONE = 0, IFACE_ETH = 1, IFACE_WIFI = 2 };
NetIface activeIface   = IFACE_NONE;

static unsigned long lastWifiReconnectMs = 0;
static unsigned long lastNetLogMs        = 0;
static unsigned long lastNetCheckMs      = 0;
static bool          wifiStarted         = false;  // WiFi.begin() appele une seule fois

// ============================================================

// Variables diffusion reseau (broadcast fichier)
bool broadcastMode = false;         // Station en mode reception diffusion
bool broadcastEmitter = false;      // Cette station est l'emettrice
String broadcastPath = "";          // Chemin du fichier en cours de diffusion
unsigned long lastBroadcastRecv = 0;// Horodatage derniere activite (secondes)
int lastBrdfSeq = -1;              // Dernier numero de sequence brdf traite
bool broadcastFileSending = false;  // Emettrice : envoi de lignes en cours
unsigned long lastBrdfSendMs = 0;  // Dernier envoi brdf (millis)
int brdfSeqCounter = 0;            // Compteur de sequence brdf

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
  String value;        // La valeur de type String stockee
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
int entryCount = 0; // Nombre actuel d'entrees dans la liste


struct PingEntry {
  int nbtoping;
  unsigned long pingTime;
  int nbping;
};
PingEntry pingList[MAX_PINGS];



// --- Tunnel reseau E/S (netio) ---

// Affiche un message sur le canal physique precedent du maitre,
// utilise pour afficher les reponses qui arrivent de l'esclave.
void netioDisplay(const String& msg) {
  switch (ntioPrevIoMode) {
    case IO_USB:
      Serial.println(msg);
      break;
    // Pour ajouter un canal, ajouter un case ici.
  }
}

// Ferme le tunnel reseau E/S et restaure l'etat precedent.
void closeNetioTunnel() {
  netioMaster       = false;
  ioMode            = ntioPrevIoMode;

  netioRemote       = -1;
  netioLastActivity = 0;
}
// ---------------------------------

// --- Couche d'abstraction E/S ---
// Centralise toutes les sorties texte du firmware.
// Pour ajouter un canal, incrementer IO_* et ajouter un case ici.
void ioOutput(const String& msg) {
  switch (ioMode) {
    case IO_USB:
      Serial.println(msg);
      break;
    // Pour ajouter un canal, ajouter un case ici.
    case IO_NETIO:
      // Maitre : supprime sa propre sortie - seules les reponses de l'esclave sont affichees
      break;
  }
}

// Lit l'entree du canal actif et envoie la commande a l'interpreteur.
// Pour ajouter un canal, ajouter un case ici.
void handleSerialInput() {
  switch (ioMode) {
    case IO_USB:
      if (Serial.available() > 0) { String _s = Serial.readString(); _s.trim(); interpreter(_s); delay(100); }
      break;
    // Pour ajouter un canal, ajouter un case ici.
    case IO_NETIO:
      if (netioMaster) {
        // Le maitre lit depuis son canal physique precedent et transmet a l'esclave
        String inputCmd = "";
        bool hasInput = false;
        if (ntioPrevIoMode == IO_USB) {
          if (Serial.available() > 0) { inputCmd = Serial.readString(); hasInput = true; }
        }
        // Pour ajouter un canal, ajouter un else if ici.
        if (hasInput) {
          inputCmd.trim();
          if (inputCmd.length() > 0) {
            if (inputCmd == "exit") {
              // Fermeture propre du tunnel : signaler l'esclave puis restaurer l'etat
              int savedRemote = netioRemote;
              closeNetioTunnel();
              interpreter("trsp:" + String(savedRemote) + ":nticlose");
              ioOutput("[NETIO] Tunnel ferme proprement");
            } else {
              // Transmettre la commande a l'esclave via le reseau maille (send)
              netioLastActivity = millis();
              interpreter("send:" + String(netioRemote) + ":ntidata:" + inputCmd);
            }
          }
        }
      }
      break;
  }
}
// --------------------------------

void logN(const String& msg) { if(serialLevel >= LOG_NORMAL)  ioOutput(msg); }
void logV(const String& msg) { if(serialLevel >= LOG_VERBOSE) ioOutput(msg); }
void logD(const String& msg) { if(serialLevel >= LOG_DEBUG)   ioOutput(msg); }

void loraToSD() {
    lora.releaseBus();
    digitalWrite(busMuxPin, 0);
    delay(100);
    if (!SD.begin(16)) { logN("[ERREUR] Carte SD introuvable ou non initialisee"); }
}

void sdToLora() {
    digitalWrite(busMuxPin, 1);
    lora.acquireBus();
}

bool togateAddCommand(int id, String command) {
  if (togateCount >= MAX_TOGATE_COMMANDS) {
    logN("[ERREUR] File de commandes gateway pleine, commande ignoree");
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
        logN("[ERREUR SD] Impossible d'ecrire dans /togate.cache");
      }

      sdToLora();

      // Supprimer cette entree en decalant les suivantes
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
      // Decaler les elements suivants
      for (int j = i; j < togateCount - 1; j++) {
        togateQueue[j] = togateQueue[j + 1];
      }
      togateCount--;
      return true;
    }
  }

  logD("togate:id " + String(id) + " non trouve");
  return false;
}

// Ajouter une commande a la file d'export de fichier
void togateAddCommandFile(int id, String command) {
  // Verifier si l'ID existe deja
  for (int i = 0; i < togateCountFile; i++) {
    if (togateQueueFile[i].id == id) {
      return;  // deja present
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
      
      // Decaler les elements suivants
      for (int j = i; j < togateCountFile - 1; j++) {
        togateQueueFile[j] = togateQueueFile[j + 1];
      }
      togateCountFile--;
      return true;
    }
  }

  logD("togatef:id " + String(id) + " non trouve");
  return false;
}

// Remettre les commandes expirees dans tx.txt
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
      
      // Decaler les elements suivants
      for (int j = i; j < togateCountFile - 1; j++) {
        togateQueueFile[j] = togateQueueFile[j + 1];
      }
      togateCountFile--;
      i--; // Reverifier cet index
    }
  }
  
  if (hasExpired && linesToRestore.length() > 0) {
    // Ecrire les lignes expirees dans tx.txt
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
  
  // Si trop d'echecs, marquer la gate comme hors ligne
  if (nbtogatefailfile >= 3 && isfdeson == 1) {
    isfdeson = false;
    logV("ftx:gate hors ligne");
  }
}


// Fonction pour ajouter une nouvelle entree
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

// Fonction pour surveiller, mettre a jour ou supprimer les entrees trop anciennes
void checkAndRemoveOldPingEntries() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < pingCount; ) {
    if (currentMillis - pingList[i].pingTime > MAX_PING_AGE) {
      if (pingList[i].nbping < 3) {
        // Augmenter nbping et mettre a jour pingTime
        pingList[i].nbping++;
        pingList[i].pingTime = currentMillis;
        logD("ping:retry " + String(pingList[i].nbtoping) + " n=" + String(pingList[i].nbping));

        String tempping = "trsm:";
        tempping += pingList[i].nbtoping;
        tempping += ":ping";
        scheduleCommand(300, tempping);

        i++; // Passer a l'entree suivante
      } else {
        logD("ping:rm " + String(pingList[i].nbtoping));
        String outgoingumap = exportEdgesContainingVertex(localAddress);
        addValue(getValue(outgoingumap, ':', 1));
        logD("umap:" + outgoingumap);
        sendMessage(1, outgoingumap, 0);
        for (int j = i; j < pingCount - 1; j++) {
          pingList[j] = pingList[j + 1];
        }
        pingCount--; // Reduire le nombre d'entrees
      }
      return;
    } else {
      i++; // Passer a l'entree suivante
    }
  }
}

// Fonction pour supprimer une entree par nbtoping
void removePingEntryByNbtoping(int nbtoping) {
  for (int i = 0; i < pingCount; i++) {
    if (pingList[i].nbtoping == nbtoping) {
      logD("ping:rm " + String(nbtoping));
      for (int j = i; j < pingCount - 1; j++) {
        pingList[j] = pingList[j + 1];
      }
      pingCount--; // Reduire le nombre d'entrees
      return;
    }
  }
}

// Fonction pour ajouter une nouvelle entree
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

// Fonction pour surveiller, mettre a jour ou supprimer les entrees trop anciennes
void checkAndRemoveOldEntries() {
  unsigned long currentMillis = millis();
  for (int i = 0; i < entryCount; ) {
    if (currentMillis - entryList[i].timesend > MAX_ENTRY_AGE) {
      if (entryList[i].nbtrysend < 3) {
        // Augmenter nbtrysend et mettre a jour timesend
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
        i++; // Passer a l'entree suivante
      } else {
        logD("entry:rm id=" + entryList[i].id);
        for (int j = i; j < entryCount - 1; j++) {
          entryList[j] = entryList[j + 1];
        }
        entryCount--; // Reduire le nombre d'entrees
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
      i++; // Passer a l'entree suivante
    }
  }
}

bool deststilinl(int dest){
    for (int i = 0; i < entryCount; ) {
    if (getValue(entryList[i].msgtosend, ':', 1).toInt() == dest) {
      return false;
    }
    i++; // Passer a l'entree suivante
  }
  return true;
}

// Fonction pour supprimer une entree par ID
void removeEntryByID(const String& id) {
  for (int i = 0; i < entryCount; i++) {
    if (entryList[i].id == id) {
      logD("entry:rm id=" + id);
      for (int j = i; j < entryCount - 1; j++) {
        entryList[j] = entryList[j + 1];
      }
      entryCount--; // Reduire le nombre d'entrees
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

// CRC16/CCITT-FALSE - polynome 0x1021, init 0xFFFF
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

  // SECURITE : /wifi.cfg contient les credentials Wi-Fi — ne jamais transmettre
  // ce fichier sur le reseau LoRa ni vers/depuis le serveur.
  if (path == "/wifi.cfg") {
    logN("[SECURITE] Transfert de /wifi.cfg refuse — fichier sensible (credentials Wi-Fi)");
    return false;
  }

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
  if (!tx) { inF.close(); logN("[ERREUR SD] Impossible de creer /tx.txt pour l'encodage"); return false; }

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

  logD("compile:start " + fnameced);
  delay(50);
  loraToSD();
  if (!SD.exists("/rx.txt")) { logN("[ERREUR] Compilation impossible : /rx.txt introuvable"); return; }
  File rx = SD.open("/rx.txt", FILE_READ);
  if (!rx) { logN("[ERREUR SD] Impossible d'ouvrir /rx.txt pour la compilation"); return; }

  String meta = rx.readStringUntil('\n'); meta.trim();
  if (!meta.startsWith("META:")) { rx.close(); logN("[ERREUR] Compilation : en-tete META manquant ou corrompu"); return; }

  unsigned int totalDataPackets=0, fileCRC=0, expectedSize=0, parityGroups=0;
  int scanned = sscanf(meta.c_str(), "META:%u:%X:%u:%u", &totalDataPackets, &fileCRC, &expectedSize, &parityGroups);
  if (scanned<4) { logN("[ERREUR] Compilation : impossible de lire les parametres META"); rx.close(); return; }

  if(serialLevel >= LOG_DEBUG){ char _b[64]; snprintf(_b, sizeof(_b), "compile:%u pkt %u oct CRC=%08X", totalDataPackets, expectedSize, fileCRC); ioOutput(String(_b)); }

  if (SD.exists(fnameced)) SD.remove(fnameced);
  File out = SD.open(fnameced, FILE_WRITE);
  if (!out) { rx.close(); logN("[ERREUR SD] Impossible de creer le fichier de sortie de compilation"); return; }

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
  if (!outf) { logN("[ERREUR] Fichier compile introuvable apres ecriture"); sdToLora(); return; }
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
    logN("[FICHIER RX] Fichier " + fnameced + " recu et compile avec succes" + (origin >= 0 ? " (depuis noeud #" + String(origin) + ")" : " (diffusion reseau)"));
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
    // Diffusion de la reussite de compilation uniquement pour les fichiers recus via diff
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
    logN("[ERREUR] Table de routage pleine, nouveau lien ignore");
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
      // Decaler les aretes suivantes pour ecraser celle a supprimer
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
      
      // Deplace toutes les valeurs suivantes d'une place vers l'avant
      for (int j = i; j < dataCount - 1; j++) {
        dataArray[j] = dataArray[j + 1];
      }
      
      dataCount--; // Reduit le nombre de valeurs stockees
      i--; // Pour verifier la nouvelle valeur a cet indice
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
      return true; // Valeur trouvee
    }
  }
  return false; // Valeur non trouvee
}

String generateid(){
  String msgid = "";
  msgid += formatNumber(msgCount % (int)pow(10, 2), 3);
  msgid += formatNumber(localAddress % (int)pow(10, 2), 3);
  msgid += formatNumber(random(99) % (int)pow(10, 2), 2);
  return msgid;
}

String formatNumber(int number, int digits) {
  // Cree un buffer de la taille requise pour contenir les digits + le caractere de fin de chaine '\0'
  char buffer[digits + 1];

  // Cree le format de chaine avec des zeros devant, ex: "%04d"
  String format = "%0" + String(digits) + "d";

  // Utilise sprintf pour formater le nombre avec des zeros devant
  sprintf(buffer, format.c_str(), number);

  return String(buffer);
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
      
        msgCount = getValue(sdtoenv, ':', 0).toInt();
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
      localAddress = getValue(sdtopar, ':', 1).toInt();
      DELAY = getValue(sdtopar, ':', 2).toInt() * 1000L;
      MAX_ENTRY_AGE = getValue(sdtopar, ':', 3).toInt() * 1000L;
      stationgateway = getValue(sdtopar, ':', 4).toInt();
      TOGATE_COMMAND_TIMEOUT = getValue(sdtopar, ':', 5).toInt() * 1000L;
      { int sl = getValue(sdtopar, ':', 6).toInt(); if(sl >= LOG_NONE && sl <= LOG_DEBUG) serialLevel = sl; }
      { int im = getValue(sdtopar, ':', 7).toInt(); if(im == IO_USB) ioMode = im; }
      { long nt = getValue(sdtopar, ':', 10).toInt(); if(nt > 0) NETIO_TIMEOUT = nt * 1000L; }
      { int ft = getValue(sdtopar, ':', 11).toInt(); if(ft > 0) filetimeout = ft; }
      { int fxt = getValue(sdtopar, ':', 12).toInt(); if(fxt > 0) filetxtimeout = fxt; }

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

      // Lecture credentials Wi-Fi depuis /wifi.cfg (format : SSID:PASSWORD)
      myFile = SD.open("/wifi.cfg", FILE_READ);
      if (myFile) {
        String wificfg = "";
        while (myFile.available()) wificfg += (char)myFile.read();
        myFile.close();
        wificfg = getValue(wificfg, '\n', 0);
        String tmpSSID = getValue(wificfg, ':', 0);
        String tmpPass = getValue(wificfg, ':', 1);
        if (tmpSSID.length() > 0) {
          wifiSSID     = tmpSSID;
          wifiPassword = tmpPass;
        }
      }

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
      logN("[ERREUR SD] Impossible d'ecrire /map.cfg (carte reseau)");
    }

    String varptosd;
    varptosd += MAX_PING_AGE / 1000;
    varptosd += ":";
    varptosd += localAddress;
    varptosd += ":";
    varptosd += DELAY / 1000;
    varptosd += ":";
    varptosd += MAX_ENTRY_AGE / 1000;
    varptosd += ":";
    varptosd += stationgateway;
    varptosd += ":";
    varptosd += TOGATE_COMMAND_TIMEOUT / 1000;
    varptosd += ":";
    varptosd += serialLevel;
    varptosd += ":";
    // En mode tunnel reseau (IO_NETIO), on sauvegarde le canal physique precedent
    // pour qu'au redemarrage la station revienne dans son mode normal (USB ou BT).
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
      logN("[ERREUR SD] Impossible d'ecrire /p.cfg (parametres)");
    }

        testFile = SD.open("/crontab.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(crontabString);
      testFile.close();
    }
    else{
      logN("[ERREUR SD] Impossible d'ecrire /crontab.cfg");
    }

    // Sauvegarde credentials Wi-Fi dans /wifi.cfg (format : SSID:PASSWORD)
    if (wifiSSID.length() > 0) {
      testFile = SD.open("/wifi.cfg", FILE_WRITE);
      if (testFile) {
        testFile.println(wifiSSID + ":" + wifiPassword);
        testFile.close();
      } else {
        logN("[ERREUR SD] Impossible d'ecrire /wifi.cfg");
      }
    }

    String varetosd;
    varetosd += msgCount;
    varetosd += ":";

    testFile = SD.open("/e.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(varetosd);
      testFile.close();
    }
    else{
      logN("[ERREUR SD] Impossible d'ecrire /e.cfg (etat)");
    }

    sdToLora();
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
// Lit une ligne de tx.txt et la diffuse en brdf:SEQ:LINE vers le reseau (dest=0)
// Appelee periodiquement depuis loop() tant que broadcastEmitter && broadcastFileSending
void broadcastExportLine() {
  delay(50);
  loraToSD();

  File fichier = SD.open("/tx.txt", FILE_READ);
  if (!fichier) {
    logN("[ERREUR] Diffusion reseau interrompue : /tx.txt introuvable");
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
    addValue(tempid);  // Eviter de retraiter notre propre brde
    sendMessage(0, brde, 0);  // Pas de reveil
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
      lastBrdfSeq = brdfSeqCounter;  // Marquer comme deja traite (ignore retransmissions)
      brdfSeqCounter++;
      sendMessage(0, brdf, 0);  // Pas de reveil
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
    logN("[FICHIER TX] Transfert de " + ftfpath + " termine, attente de confirmation du noeud #" + String(filereceivientstation) + "...");
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
      
      // Extraire l'ID de la commande (format identique a exportcache)
      String tempid = generateid();
      togateAddCommandFile(tempid.toInt(), ligneStr);

      String outglignestr = "file:";
      outglignestr += ligneStr;
      // Passer la ligne a l'interpreteur      
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
    logN("[ERREUR SD] Ecriture impossible dans " + file);
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
    logN("[SD] Repertoire " + path + " introuvable ou invalide");
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
    logN("[ERREUR MAJU] Fichier firmware introuvable sur la carte SD");
    return false;
  }

  size_t updateSize = updateFile.size();
  logN("[MISE A JOUR] Demarrage de la mise a jour, fichier de " + String(updateSize) + " octets...");

  if (updateSize == 0) {
    logN("[ERREUR MAJU] Le fichier firmware est vide");
    updateFile.close();
    return false;
  }

  if (!Update.begin(updateSize)) {  // verifie la partition
    logN("[ERREUR MAJU] Impossible de demarrer la mise a jour (partition insuffisante ?)");
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
      logN("[ERREUR MAJU] Ecriture partielle du firmware, mise a jour annulee");
      Update.printError(Serial);
      updateFile.close();
      Update.abort();
      return false;
    }
  }

  updateFile.close();

  if (!Update.end()) {
    logN("[ERREUR MAJU] Erreur lors de la finalisation de la mise a jour");
    Update.printError(Serial);
    return false;
  }

  if (!Update.isFinished()) {
    logN("[ERREUR MAJU] Mise a jour incomplete, redemarrage annule");
    return false;
  }

  logN("[MISE A JOUR] Succes - " + String(written) + " octets ecrits, redemarrage dans 1s...");
  prefs.putBool("justrestarted", true);  // Signale qu'une diffusion de version est requise au boot
  delay(1000);
  ESP.restart();
  return true;
}

// ============================================================
// Phase 1 — Fonctions reseau
// Approche par polling : compatible ESP32 core v2.x et v3.x,
// et contourne le conflit avec la bibliotheque Arduino WiFi standard.
// ============================================================

// Retourne l'adresse IP de l'interface active, ou "" si aucune.
String getLocalIP() {
  if (activeIface == IFACE_ETH)  return ETH.localIP().toString();
  if (activeIface == IFACE_WIFI) return WiFi.localIP().toString();
  return "";
}

// Initialise Ethernet et Wi-Fi simultanement.
// A appeler depuis setup() apres readsd() pour disposer des credentials.
void initNetwork() {
  // WiFi AVANT ETH : en ESP32 Arduino v3.x, initialiser ETH en premier
  // modifie l'etat interne du stack WiFi et empeche WiFi.begin() de connecter.
  // L'ordre correct pour l'operation simultanee ETH + WiFi est WiFi d'abord.
  if (wifiSSID.length() > 0) {
    wifiStarted = true;
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    char ssidBuf[64] = {};
    wifiSSID.toCharArray(ssidBuf, sizeof(ssidBuf));
    WiFi.begin(ssidBuf, wifiPassword.c_str());
    logN("[WIFI] Connexion a '" + wifiSSID + "'...");
  } else {
    logN("[WIFI] Pas de SSID configure (voir commande setwifi)");
  }

  // Ethernet LAN8720 (apres WiFi)
  // Signature ESP32 core v3.x.x : begin(type, phy_addr, mdc, mdio, power, clk_mode)
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO,
            ETH_PHY_POWER, NET_ETH_CLK_MODE);
  ETH.setHostname(("MycroMesh-Gate-" + String(localAddress)).c_str());
  logN("[ETH] Initialisation LAN8720...");
}

// Surveillance de l'etat reseau et reconnexion automatique.
// Throttlee a 1 appel/seconde.
void checkNetworkReconnect() {
  unsigned long now = millis();

  // Throttle : une seule execution par seconde
  if (now - lastNetCheckMs < 1000UL && now >= lastNetCheckMs) return;
  lastNetCheckMs = now;

  // --- Etat Ethernet (polling via adresse IP) ---
  bool newEthConn = (ETH.localIP() != IPAddress(0, 0, 0, 0));
  if (newEthConn != ethConnected) {
    ethConnected = newEthConn;
    if (ethConnected) {
      activeIface = IFACE_ETH;
      logN("[ETH] IP=" + ETH.localIP().toString());
    } else {
      logN("[ETH] Liaison perdue");
      if (activeIface == IFACE_ETH) {
        activeIface = wifiConnected ? IFACE_WIFI : IFACE_NONE;
        if (activeIface == IFACE_WIFI)
          logN("[RESEAU] Bascule Wi-Fi — IP=" + WiFi.localIP().toString());
        else
          logN("[RESEAU] Aucune interface disponible");
      }
    }
  }
  // Retour sur ETH si elle est revenue
  if (ethConnected && activeIface != IFACE_ETH) {
    activeIface = IFACE_ETH;
    logN("[RESEAU] Retour Ethernet — IP=" + ETH.localIP().toString());
  }

  // --- Etat Wi-Fi (polling via WiFi.status()) ---
  if (wifiSSID.length() > 0) {
    int wifiStat = (int)WiFi.status();
    bool newWifiConn = (wifiStat == WL_CONNECTED);
    if (newWifiConn != wifiConnected) {
      wifiConnected = newWifiConn;
      if (wifiConnected) {
        if (activeIface == IFACE_NONE) {
          activeIface = IFACE_WIFI;
          logN("[WIFI] IP=" + WiFi.localIP().toString());
        } else {
          logV("[WIFI] IP=" + WiFi.localIP().toString() + " (standby — ETH prioritaire)");
        }
      } else {
        logN("[WIFI] Deconnexion");
        if (activeIface == IFACE_WIFI) {
          activeIface = IFACE_NONE;
          logN("[RESEAU] Aucune interface disponible");
        }
      }
    }

    // Premier demarrage Wi-Fi : differe de 30 s pour laisser ETH se stabiliser.
    // WiFi.begin() est appele UNE SEULE FOIS. La reconnexion automatique est
    // geree par WiFi.setAutoReconnect(true) — on n'appelle jamais begin() a
    // nouveau dans cette boucle pour ne pas interrompre une connexion en cours.
    if (!wifiStarted &&
        (now - lastWifiReconnectMs > 30000UL || now < lastWifiReconnectMs)) {
      wifiStarted = true;
      logN("[WIFI] Demarrage connexion...");
      WiFi.mode(WIFI_STA);
      WiFi.setAutoReconnect(true);
      char ssidBuf[64] = {};
      wifiSSID.toCharArray(ssidBuf, sizeof(ssidBuf));
      WiFi.begin(ssidBuf, wifiPassword.c_str());
    }
  }

  // Log periodique d'etat (toutes les 5 min, niveau verbose)
  if (serialLevel >= LOG_VERBOSE &&
      (now - lastNetLogMs > 300000UL || now < lastNetLogMs)) {
    lastNetLogMs = now;
    if      (activeIface == IFACE_ETH)  logV("[RESEAU] ETH  ip=" + getLocalIP());
    else if (activeIface == IFACE_WIFI) logV("[RESEAU] WIFI ip=" + getLocalIP());
    else                                logV("[RESEAU] Aucune interface active");
  }
}

// ============================================================

// --- CPU Frequency Scaling ---

void setup() {
  Serial.begin(115200);

  prefs.begin("mycromesh", false);
  
  clearEdges();

  SPI.begin(14, 12, 13, 15); // SCK=io14, MISO=io12, MOSI=io13, SS=io15

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
  }

  if (localAddress == 0){
    MAX_PING_AGE = 20000;
    localAddress = 61;          // address of this device
    DELAY = 3600000;           // Delai en millisecondes (ici 5 secondes)
    MAX_ENTRY_AGE = 20000;     // Duree maximale (en millisecondes) avant traitement d'une entree
  }


  logN("[OK] Noeud #" + String(localAddress) + " demarre - firmware " + FIRMWARE_VERSION);

  // Initialisation reseau (Wi-Fi + Ethernet) — Phase 1
  initNetwork();

  // Si ce demarrage fait suite a une mise a jour OTA, planifie la diffusion de la version
  // firmware a l'ensemble du reseau 30 secondes apres le boot
  if (prefs.getBool("justrestarted", false)) {
    prefs.putBool("justrestarted", false);
    scheduleCommand(30000 + (80 * localAddress), "fwver");  // 5 min + delai proportionnel au rang
    logN("[MAJU RESEAU] Post-mise a jour - diffusion de version planifiee dans 30 sec");
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
    logN("[PING] Decouverte terminee — " + String(numEdges) + " lien(s) dans la table de routage");
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

  // --- Diffusion reseau : envoi periodique d'une ligne brdf (2s entre chaque ligne) ---
  if (broadcastEmitter && broadcastFileSending) {
    if (millis() >= lastBrdfSendMs + 2000 || millis() < lastBrdfSendMs) {
      lastBrdfSendMs = millis();
      broadcastExportLine();
    }
  }

  // --- Timeout 5 minutes en mode reception diffusion (stations receptrices) ---
  if (broadcastMode && !broadcastEmitter) {
    unsigned long nowSec = millis() / 1000;
    if ((nowSec - lastBroadcastRecv) > 300 || (nowSec < lastBroadcastRecv)) {
      broadcastMode = false;
      lastBrdfSeq = -1;
      logV("diff:timeout rx");
    }
  }

  // --- Timeout du tunnel reseau E/S ---
  // Si aucun echange ne transite pendant NETIO_TIMEOUT ms, on ferme le tunnel proprement.
  if (netioMaster && netioLastActivity > 0 &&
      (millis() - netioLastActivity > (unsigned long)NETIO_TIMEOUT)) {
    int  savedRemote = netioRemote;
    bool wasMaster   = netioMaster;
    closeNetioTunnel();
    ioOutput("[NETIO] Timeout - tunnel ferme (inactivite)");
    // Notifier la station distante via trsp (paquet de fin fiable)
    interpreter("trsp:" + String(savedRemote) + ":nticlose");
  }


  if (filereceivientstation > -1 && (isfdeson == 0 || infilecache == 0) && (((millis()/1000) - filetxdelai) > filetxtimeout || filetxdelai > (millis()/1000))){
    filereceivientstation = -1;
    logN("[ERREUR] Transfert fichier echoue : le noeud destinataire ne repond plus");
  }
  if (filesender > -1 && (((millis()/1000) - filedelai) > filetimeout || filedelai > (millis()/1000))){
    filesender = -1;
    logN("[ERREUR] Reception fichier interrompue : timeout depasse, transfert annule");
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

  // Surveillance et reconnexion reseau (Phase 1)
  checkNetworkReconnect();

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

  logN("[ERREUR] File de commandes interne pleine, commande ignoree");
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

  lastair = millis();
}

void onReceive() {
  uint8_t rxBuf[255];
  uint8_t packetSize = lora.readPacket(rxBuf, sizeof(rxBuf));
  if (packetSize == 0) return;
  lora.receive();



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

    // --- Diffusion reussite de compilation : bcok:ID:STATION:CHEMIN ---
    if (getValue(incoming, ':', 0) == "bcok") {
      String bcokid = getValue(incoming, ':', 1);
      if (!findValue(bcokid)) {
        addValue(bcokid);
        logV("bcok:station " + getValue(incoming, ':', 2) + " compile ok " + getValue(incoming, ':', 3));
        // Retransmission sans reveil avec delai proportionnel au rang
        String rbcok = "trsms:0:";
        rbcok += incoming;
        scheduleCommand(20 * localAddress, rbcok);
      }
    }

    // --- Diffusion de mise a jour firmware : bupd:ID ---
    // Recu par toutes les stations : planifie l'upgrade 60 s apres reception et retransmet
    if (getValue(incoming, ':', 0) == "bupd") {
      String bupdid = getValue(incoming, ':', 1);
      if (!findValue(bupdid)) {
        addValue(bupdid);
        logN("[MAJU RESEAU] Declenchement de mise a jour recu - upgrade dans 30 s");
        scheduleCommand(30000, "upgrade");
        String rbupd = "trsms:0:";
        rbupd += incoming;
        scheduleCommand(20 * localAddress, rbupd);
      }
    }

    // --- Diffusion de version firmware apres mise a jour : fwver:ID:VERSION:STATION ---
    // Recu par toutes les stations apres le redemarrage post-mise a jour d'une station
    if (getValue(incoming, ':', 0) == "fwver") {
      String fwverid = getValue(incoming, ':', 1);
      if (!findValue(fwverid)) {
        addValue(fwverid);
        logN("[MAJU RESEAU] Station #" + getValue(incoming, ':', 3) + " tourne maintenant sur le firmware " + getValue(incoming, ':', 2));
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
          // SECURITE : refuser la reception de /wifi.cfg via le reseau
          if (broadcastPath == "/wifi.cfg") {
            logN("[SECURITE] Reception de /wifi.cfg via diffusion refusee — fichier sensible");
            broadcastMode = false;
            broadcastPath = "";
            return;
          }
          lastBroadcastRecv = millis() / 1000;
          lastBrdfSeq = -1;
          // Nettoyage rx.txt et preparation reception (via interpreter, hors ISR)
          scheduleCommand(10, "brdinit");
          logV("diff:rx on " + broadcastPath);
        }
        // Retransmission avec delai proportionnel au rang
        String rbrdl = "trsm:0:";
        rbrdl += incoming;
        scheduleCommand(20 * localAddress, rbrdl);
      }
    }

    // --- Ligne de donnees diffusee : brdf:SEQ:LINEDATA ---
    if (getValue(incoming, ':', 0) == "brdf") {
      if (broadcastMode && !broadcastEmitter) {
        int seq = getValue(incoming, ':', 1).toInt();
        if (seq != lastBrdfSeq) {
          lastBrdfSeq = seq;
          lastBroadcastRecv = millis() / 1000;
          // Traitement complet (ecriture SD + retransmission) via interpreter
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
          // Retransmission sans reveil
          String rbrde = "trsms:0:";
          rbrde += incoming;
          scheduleCommand(20 * localAddress, rbrde);
          // Compilation differee (laisser les retransmissions se propager)
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
  if (broadcastMode) return;  // Bloquer les cronjobs pendant la diffusion reseau
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


void changepval(String parn, String parv){
  if(parn == "stationgateway"){
    stationgateway = parv.toInt();
    logN("[CONFIG] Parametre 'stationgateway' mis a jour : " + String(stationgateway));
    return;
  }
  if(parn == "MAX_PING_AGE"){
    MAX_PING_AGE = parv.toInt();
    logN("[CONFIG] Parametre 'MAX_PING_AGE' mis a jour : " + String(MAX_PING_AGE) + " ms");
  }

  if(parn == "localAddress"){
    localAddress = parv.toInt();
    logN("[CONFIG] Parametre 'localAddress' mis a jour : " + String(localAddress));
  }
  if(parn == "DELAY"){
    DELAY = parv.toInt();
    logN("[CONFIG] Parametre 'DELAY' (deduplication) mis a jour : " + String(DELAY) + " ms");
  }
  if(parn == "MAX_ENTRY_AGE"){
    MAX_ENTRY_AGE = parv.toInt();
    logN("[CONFIG] Parametre 'MAX_ENTRY_AGE' mis a jour : " + String(MAX_ENTRY_AGE) + " ms");
  }

  if(parn == "TOGATE_COMMAND_TIMEOUT"){
    TOGATE_COMMAND_TIMEOUT = parv.toInt();
    logN("[CONFIG] Timeout gateway mis a jour : " + String(TOGATE_COMMAND_TIMEOUT) + " ms");
  }
  if(parn == "serialLevel"){
    int sl = parv.toInt();
    if(sl >= LOG_NONE && sl <= LOG_DEBUG) serialLevel = sl;
    ioOutput("[CONFIG] Niveau de log via parm regle sur " + String(serialLevel));
  }
  if(parn == "NETIO_TIMEOUT"){
    NETIO_TIMEOUT = parv.toInt();
    logN("[CONFIG] Timeout tunnel reseau E/S mis a jour : " + String(NETIO_TIMEOUT) + " ms");
  }
}

void interpreter(String msg){  
  String cmd = getValue(msg, ':', 0);


  if(cmd == "read"){
      readsd(1);
      logN("[OK] Carte reseau et parametres recharges depuis la carte SD");
  }
  if(cmd == "write"){
     writetosd();
     logN("[OK] Configuration sauvegardee sur la carte SD");
  }
  // -- getcfg : lit p.cfg et l'envoie sur le port actif (pour le configurateur web) --
  if(cmd == "getcfg"){
    String cfg;
    cfg += MAX_PING_AGE / 1000;           cfg += ":";
    cfg += localAddress;                  cfg += ":";
    cfg += DELAY / 1000;                  cfg += ":";
    cfg += MAX_ENTRY_AGE / 1000;          cfg += ":";
    cfg += stationgateway;                cfg += ":";
    cfg += TOGATE_COMMAND_TIMEOUT / 1000; cfg += ":";
    cfg += serialLevel;                   cfg += ":";
    cfg += (ioMode == IO_NETIO ? ntioPrevIoMode : ioMode); cfg += ":";
    cfg += NETIO_TIMEOUT / 1000;          cfg += ":";
    cfg += filetimeout;                   cfg += ":";
    cfg += filetxtimeout;                 cfg += ":";
    ioOutput("CFG:" + cfg);
  }
  // -- setcfg : recoit p.cfg depuis le configurateur, applique et sauvegarde --
  // Format attendu : setcfg:MAX_PING_AGE:localAddress:...:filetxtimeout:
  if(cmd == "setcfg"){
    int fieldCount = 0;
    for(int i = 0; i < (int)msg.length(); i++) if(msg[i] == ':') fieldCount++;
    if(fieldCount < 13){
      ioOutput("CFG:ERR:format invalide (" + String(fieldCount) + "/13 champs)");
    } else {
      MAX_PING_AGE           = getValue(msg,':',1).toInt() * 1000L;
      localAddress           = getValue(msg,':',2).toInt();
      DELAY                  = getValue(msg,':',3).toInt() * 1000L;
      MAX_ENTRY_AGE          = getValue(msg,':',4).toInt() * 1000L;
      stationgateway         = getValue(msg,':',5).toInt();
      TOGATE_COMMAND_TIMEOUT = getValue(msg,':',6).toInt() * 1000L;
      { int sl = getValue(msg,':',7).toInt(); if(sl >= LOG_NONE && sl <= LOG_DEBUG) serialLevel = sl; }
      { int im = getValue(msg,':',10).toInt(); if(im == IO_USB) ioMode = im; }
      { long nt = getValue(msg,':',11).toInt(); if(nt > 0) NETIO_TIMEOUT = nt * 1000L; }
      { int ft  = getValue(msg,':',12).toInt(); if(ft  > 0) filetimeout  = ft; }
      { int fxt = getValue(msg,':',13).toInt(); if(fxt > 0) filetxtimeout = fxt; }
      writetosd();
      ioOutput("CFG:OK");
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
      logN("[PING] Envoi des sondes de decouverte reseau...");
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
      logN("[MESSAGE] Accuse de reception - message #" + getValue(msg, ':', 1) + " bien livre au noeud #" + getValue(msg, ':', 2));
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
      logN("[GATEWAY] Adresse de la passerelle reseau : noeud #" + String(stationgateway));
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
    if(cmd == "gmea"){
      String s;
      s += "trxglob:" + String(rxglob) + ";";
      s += "trxbrd:"  + String(rxbrd)  + ";";
      s += "trxloc:"  + String(rxloc)  + ";";
      s += "tdijktx:" + String(dijktx) + ";";
      s += "tdijkrx:" + String(dijkrx) + ";";
      s += "tdijkhop:"+ String(dijkhop)+ ";";
      s += "tresend1:"+ String(resend1)+ ";";
      s += "tresend2:"+ String(resend2)+ ";";
      s += "ttxcnt:"  + String(txcnt)  + ";";
      ioOutput(s);
      rxglob = 0; rxbrd = 0; rxloc = 0;
      dijktx = 0; dijkrx = 0; dijkhop = 0;
      resend1 = 0; resend2 = 0; txcnt = 0;
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
      // En mode esclave netio, le niveau est contraint a [1, 2] :
      // 0 (silencieux) priverait le maitre de toute sortie,
      // 3 (debug) inonderait le tunnel de messages internes.
      ioOutput("[CONFIG] Niveau de log regle sur '" + arg + "' (" + String(serialLevel) + ")");
      writetosd();
    }
    if(cmd == "iomd"){
      String arg = getValue(msg, ':', 1);
      if(arg == "usb"){
        ioMode = IO_USB;
        ioOutput("[IO] Mode USB active");
        writetosd();
      } else {
        ioOutput("[IO] Mode inconnu. Commandes: iomd:usb  (netio:<addr> pour le tunnel reseau)");
      }
    }
    if(cmd == "cout"){
      logN("[GATEWAY] Envoi de la prochaine entree du cache vers la passerelle...");
      exportcache();
    }
    if(cmd == "clear"){
      int prevEdges = numEdges;
      clearEdges();
      logN("[OK] Table de routage effacee (" + String(prevEdges) + " lien(s) supprime(s))");
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
      logN("[FICHIER TX] Etat - cache actif : " + String(infilecache ? "oui" : "non") + "  |  envoi actif : " + String(isfdeson ? "oui" : "non") + "  |  lignes en attente : " + String(togateCountFile));
    }
    if(cmd == "stopfile"){
      isfdeson = false;
      infilecache = false;
      logN("[FICHIER TX] Transfert de fichier arrete manuellement");
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
          logN("[FICHIER TX] Fichier " + ftfpath + " pret, connexion avec le noeud #" + String(filereceivientstation) + " en cours...");
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
        logN("[FICHIER RX] Debut de reception d'un fichier depuis le noeud #" + String(filesender));
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
        logN("[FICHIER TX] Noeud #" + String(filereceivientstation) + " pret, envoi de " + ftfpath + " en cours...");
        isfdeson = true;
        prefs.putUInt("offsetfile", 0);
        nbtogatefailfile = 0;
        ltgdelfile = (millis()/1000);
      }
    }
    
    if(cmd == "fend"){
      if(getValue(msg, ':', 1).toInt() == filesender){
        logN("[FICHIER RX] Fin de reception depuis le noeud #" + String(filesender) + ", compilation vers " + getValue(msg, ':', 2) + "...");
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
      logN("[FICHIER TX] Confirmation recue - " + getValue(msg, ':', 2) + " bien recu par le noeud #" + getValue(msg, ':', 1));
      if(getValue(msg, ':', 3) == "1"){
        logD("feok:rm " + getValue(msg, ':', 2));
        remfromsd(getValue(msg, ':', 2));
      }
    }
    if(cmd == "reboot"){
      logN("[SYSTEME] Redemarrage du noeud en cours...");
      writetosd();
      delay(500);
      ESP.restart();
    }
    if(cmd == "upgrade"){
      logN("[MISE A JOUR] Lancement de la mise a jour du firmware...");
      writetosd();
      delay(500);
      doFirmwareUpdate();
    }
    if(cmd == "version"){
      logN("[SYSTEME] Version du firmware : " + FIRMWARE_VERSION);
    }
    if(cmd == "mkdir"){
      String path = getValue(msg, ':', 1);
      logN("[SD] Dossier " + path + (mkdirsd(path) ? " cree avec succes" : " - erreur de creation"));
    }
    if(cmd == "rmdir"){
      String path = getValue(msg, ':', 1);
      logN("[SD] Dossier " + path + (rmdirsd(path) ? " supprime" : " - erreur de suppression"));
    }
    if(cmd == "rm"){
      String path = getValue(msg, ':', 1);
      logN("[SD] Fichier " + path + (remfromsd(path) ? " supprime" : " - erreur de suppression"));
    }
    if(cmd == "cp"){
      String src = getValue(msg, ':', 1);
      String dstDir = getValue(msg, ':', 2);
      String fname = src.substring(src.lastIndexOf('/'));
      String dst = dstDir.endsWith("/") ? dstDir + fname.substring(1) : dstDir + fname;
      logN("[SD] Copie " + src + " -> " + dst + (cpsd(src, dst) ? " : succes" : " : erreur"));
    }
    if(cmd == "mv"){
      String src = getValue(msg, ':', 1);
      String dstDir = getValue(msg, ':', 2);
      String fname = src.substring(src.lastIndexOf('/'));
      String dst = dstDir.endsWith("/") ? dstDir + fname.substring(1) : dstDir + fname;
      logN("[SD] Deplacement " + src + " -> " + dst + (mvsd(src, dst) ? " : succes" : " : erreur"));
    }
    if(cmd == "ls"){
      String path = getValue(msg, ':', 1);
      if(path == "") path = "/";
      lssd(path);
    }

    // Diffusion de la reussite de compilation (schedule 5 min apres compileFile OK)
    // Format diffuse : bcok:ID:STATION:CHEMIN
    if (cmd == "bcok") {
      String tempid = generateid();
      String bcok = "bcok:";
      bcok += tempid;
      bcok += ":";
      bcok += localAddress;
      bcok += ":";
      bcok += getValue(msg, ':', 1);  // chemin du fichier compile
      addValue(tempid);  // Eviter de retraiter notre propre bcok si recu en echo
      sendMessage(0, bcok, 0);  // Diffusion sans reveil
      logV("bcok:" + getValue(msg, ':', 1));
    }

    // ================================================================
    // DIFFUSION DE MISE A JOUR FIRMWARE : bupd
    // Declenche sur l'ensemble du reseau une mise a jour 60 s apres reception
    // ================================================================
    if (cmd == "bupd") {
      String tempid = generateid();
      String bupd = "bupd:";
      bupd += tempid;
      addValue(tempid);
      sendMessage(0, bupd, 0);  // Diffusion sans reveil
      logN("[MAJU RESEAU] Diffusion de mise a jour envoyee - chaque station upgradee dans 60 s");
      scheduleCommand(30000, "upgrade");  // La station emettrice upgradera egalement
    }

    // ================================================================
    // DIFFUSION DE VERSION FIRMWARE : fwver
    // Diffuse la version du firmware actuel a l'ensemble du reseau
    // (planifie automatiquement 5 min apres une mise a jour reussie)
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
      sendMessage(0, fwver, 0);  // Diffusion sans reveil
      logN("[MAJU RESEAU] Diffusion de version firmware : " + FIRMWARE_VERSION);
    }

    // ================================================================
    // DIFFUSION RESEAU : diff:chemin/fichier
    // Diffuse un fichier vers l'ensemble du reseau (broadcast)
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
          logD("diff:brdl envoye");
          scheduleCommand(5000, "brdfstart");
        } else {
          logN("[ERREUR] Diffusion reseau : impossible de parser le fichier source");
          broadcastMode = false;
          broadcastEmitter = false;
        }
      } else {
        logN("[ERREUR] Diffusion impossible : un transfert ou une diffusion est deja en cours");
      }
    }

    // Declenchement de l'envoi des lignes brdf (apres delai propagation brdl)
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

    // Reception d'une ligne de donnees diffusee : brdf:SEQ:LINEDATA
    if (cmd == "brdf") {
      if (broadcastMode && !broadcastEmitter) {
        // Extraction de LINEDATA en evitant les problemes de split sur ':' dans les donnees
        String seqStr = getValue(msg, ':', 1);
        int lineStart = 5 + seqStr.length() + 1;  // "brdf:" + seqStr + ":"
        String lineData = msg.substring(lineStart);
        if (lineData.length() > 0) {
          importfile("/rx.txt", lineData);
          // Retransmission sans reveil avec delai proportionnel au rang
          String rbrdf = "trsms:0:";
          rbrdf += msg;
          scheduleCommand(20 * localAddress, rbrdf);
        }
      }
    }

    // Reception du signal de fin de diffusion : brde:ID:chemin
    if (cmd == "brde") {
      if (broadcastMode && !broadcastEmitter) {
        String filePath = getValue(msg, ':', 2);
        logV("diff:rx fin -> " + filePath);
        broadcastMode = false;
        lastBrdfSeq = -1;
        compileFile(filePath, -1, 0);  // origin=-1 : pas de feok a envoyer
        logV("diff:rx done");
      }
    }

    // ================================================================
    // MODE RESEAU E/S (netio) - tunnel maitre/esclave via le maillage
    // ================================================================

    // Commande utilisateur : ouvre un tunnel vers l'esclave <addr>
    // Format : netio:<addr>
    if (cmd == "netio") {
      int slaveAddr = getValue(msg, ':', 1).toInt();
      if (slaveAddr <= 0) {
        ioOutput("[NETIO] Erreur : adresse esclave invalide");
      } else if (netioMaster) {
        ioOutput("[NETIO] Un tunnel reseau est deja actif");
      } else {
        ioOutput("[NETIO] Ouverture du tunnel vers le noeud #" + String(slaveAddr) + "...");
        ntioPrevIoMode    = ioMode;
        netioRemote       = slaveAddr;
        netioLastActivity = millis();
        // Paquet d'initialisation envoye via trsp (fiable, avec accuse de reception)
        interpreter("trsp:" + String(slaveAddr) + ":ntiopen:" + String(localAddress));
        // Passer en mode maitre apres l'envoi pour ne pas supprimer les logs de l'init
        netioMaster = true;
        ioMode      = IO_NETIO;
      }
    }

    // Recu par le maitre : l'esclave confirme que le tunnel est etabli
    // Format : ntiok
    if (cmd == "ntiok") {
      if (netioMaster && netioRemote >= 0) {
        netioLastActivity = millis();
        netioDisplay("[NETIO] Tunnel ouvert avec le noeud #" + String(netioRemote) + " - tapez 'exit' pour fermer");
      }
    }

    // Recu par le maitre : reponse de l'esclave a afficher localement
    // Format : ntirsp:<message>
    if (cmd == "ntirsp") {
      if (netioMaster) {
        netioLastActivity = millis();
        netioDisplay(msg.substring(7)); // retire le prefixe "ntirsp:"
      }
    }

    // Recu par l'un ou l'autre : fermeture du tunnel (depuis timeout ou commande distante)
    // Format : nticlose
    if (cmd == "nticlose") {
      if (netioMaster) {
        closeNetioTunnel();
        ioOutput("[NETIO] Tunnel reseau ferme");
      }
    }

  // ----------------------------------------------------------------
  // Phase 1 — Commandes reseau
  // ----------------------------------------------------------------

  // netstat : affiche l'etat des interfaces ETH et Wi-Fi
  if (cmd == "netstat") {
    String iface;
    if      (activeIface == IFACE_ETH)  iface = "ETH";
    else if (activeIface == IFACE_WIFI) iface = "WIFI";
    else                                iface = "NONE";
    logN("[RESEAU] Interface active : " + iface + "  IP=" + getLocalIP());
    logN("[ETH]   connecte=" + String(ethConnected ? "oui" : "non") +
         "  IP=" + (ethConnected ? ETH.localIP().toString() : "-"));
    logN("[WIFI]  connecte=" + String(wifiConnected ? "oui" : "non") +
         "  SSID=" + (wifiSSID.length() > 0 ? wifiSSID : "(non configure)") +
         "  IP=" + (wifiConnected ? WiFi.localIP().toString() : "-"));
  }

  // setwifi : configure les credentials Wi-Fi et reconnecte
  // Format : setwifi:SSID:PASSWORD
  if (cmd == "setwifi") {
    String newSSID = getValue(msg, ':', 1);
    String newPass = getValue(msg, ':', 2);
    if (newSSID.length() > 0) {
      wifiSSID     = newSSID;
      wifiPassword = newPass;
      writetosd();
      logN("[WIFI] Credentials mis a jour — SSID=" + wifiSSID);
      wifiStarted = true;  // marque comme demarre pour ne pas rappeler begin() dans checkNetworkReconnect
      WiFi.mode(WIFI_STA);
      WiFi.setAutoReconnect(true);
      WiFi.disconnect();
      char ssidBuf[64] = {};
      wifiSSID.toCharArray(ssidBuf, sizeof(ssidBuf));
      WiFi.begin(ssidBuf, wifiPassword.c_str());
    } else {
      logN("[WIFI] Erreur setwifi : SSID manquant (format : setwifi:SSID:PASSWORD)");
    }
  }
}
