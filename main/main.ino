#include <SPI.h>
#include <LoRa.h>
#include <limits.h>
#include <ESP32Time.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include <SD.h>
#include <time.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Preferences.h>

Preferences prefs;

ESP32Time rtc;

#define uS_TO_S_FACTOR 1000000

Adafruit_BME680 bme;

// Variable Cron

String crontabString;

// Variable de parametres

long MAX_PING_AGE = 20000;      // Durée maximale (en millisecondes) avant traitement d'une entrée
long starttimeout = 15000;      // Durée timout procudure start
int localAddress = 61;          // address of this device
long DELAY = 3600000;           // Délai en millisecondes (ici 5 secondes)
long MAX_ENTRY_AGE = 20000;     // Durée maximale (en millisecondes) avant traitement d'une entrée
long actiontimerdel = 30;
bool maintmode = true;
int stationgateway = 1;
long TOGATE_COMMAND_TIMEOUT = 60000;

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

const int csPin = 21;          // LoRa radio chip select
const int resetPin = -1;       // LoRa radio reset
const int irqPin = 8;          // change for your board; must be a hardware interrupt pin

// variables systeme

unsigned long ltgdel = 0;
bool sensorstart = false;
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


bool togateAddCommand(int id, String command) {
  if (togateCount >= MAX_TOGATE_COMMANDS) {
    Serial.println("[Togate] Erreur : file pleine, commande non ajoutée.");
    return false;
  }

  togateQueue[togateCount].id = id;
  togateQueue[togateCount].command = command;
  togateQueue[togateCount].timestamp = millis();
  Serial.print("[Togate] Commande ajoutée : ID=");
  Serial.print(id);
  Serial.print(", Commande=\"");
  Serial.print(command);
  Serial.println("\"");
  togateCount++;
  return true;
}

// Supprimer les commandes plus vieilles qu'une minute
void togatePurgeOld() {
  bool purged = false;

  for (int i = 0; i < togateCount; ) {
    if ((millis() - togateQueue[i].timestamp) > TOGATE_COMMAND_TIMEOUT) {
      Serial.print("[Togate] Suppression automatique (ancienne) : ID=");
      Serial.print(togateQueue[i].id);
      Serial.print(", Commande=\"");
      Serial.print(togateQueue[i].command);
      Serial.println("\"");

      Serial.print("ctg to sd : ");      
      Serial.println(togateQueue[i].command);
      prefs.putBool("incache", true);
      prefs.putBool("isgateonline", false);

      delay(50);
      LoRa.setFrequency(433.1);
      SPI.transfer(0);
      digitalWrite(csPin,1);
      digitalWrite(20,0);
      delay(100);

      if (!SD.begin(7)) {Serial.println("PBsd");}
      File myFile;
      myFile = SD.open("/togate.cache", FILE_APPEND);   
      if (myFile) {
        myFile.println(togateQueue[i].command);
        myFile.close();
      }
      else{
        Serial.println("PBfile");
      }

      digitalWrite(csPin,0);
      digitalWrite(20,1);
      LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
      LoRa.begin(433E6); 

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
      Serial.print("[Togate] Suppression par ID : ID=");
      Serial.print(togateQueue[i].id);
      Serial.print(", Commande=\"");
      Serial.print(togateQueue[i].command);
      Serial.println("\"");      
      prefs.putBool("isgateonline", true);

      // Décaler les éléments suivants
      for (int j = i; j < togateCount - 1; j++) {
        togateQueue[j] = togateQueue[j + 1];
      }
      togateCount--;
      return true;
    }
  }

  Serial.print("[Togate] Aucun ID correspondant à ");
  Serial.print(id);
  Serial.println(" trouvé.");
  return false;
}

// Fonction pour ajouter une nouvelle entrée
void addPingEntry(int nbtoping, int nbping) {
  if (pingCount < MAX_PINGS) {
    pingList[pingCount].nbtoping = nbtoping;
    pingList[pingCount].pingTime = millis();
    pingList[pingCount].nbping = nbping;
    pingCount++;
    Serial.println("Nouvelle entrée de ping ajoutée.");
  } else {
    Serial.println("Liste pleine, impossible d'ajouter une nouvelle entrée de ping.");
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
        Serial.print("Mise à jour de l'entrée nbtoping: ");
        Serial.print(pingList[i].nbtoping);
        Serial.print(" -> nbping: ");
        Serial.println(pingList[i].nbping);
        sendMessage(1, "ping", pingList[i].nbtoping);
        i++; // Passer à l'entrée suivante
      } else {
        // Supprimer l'entrée si nbping > 3
        Serial.print("Suppression de l'entrée nbtoping: ");
        Serial.println(pingList[i].nbtoping);
        String outgoingumap = exportEdgesContainingVertex(localAddress);
        addValue(getValue(outgoingumap, ':', 1));
        Serial.println(outgoingumap);
        sendMessage(1, outgoingumap, 0);
        for (int j = i; j < pingCount - 1; j++) {
          pingList[j] = pingList[j + 1];
        }
        pingCount--; // Réduire le nombre d'entrées
      }
    } else {
      i++; // Passer à l'entrée suivante
    }
  }
}

// Fonction pour supprimer une entrée par nbtoping
void removePingEntryByNbtoping(int nbtoping) {
  for (int i = 0; i < pingCount; i++) {
    if (pingList[i].nbtoping == nbtoping) {
      Serial.print("Suppression de l'entrée nbtoping: ");
      Serial.println(pingList[i].nbtoping);
      for (int j = i; j < pingCount - 1; j++) {
        pingList[j] = pingList[j + 1];
      }
      pingCount--; // Réduire le nombre d'entrées
      return;
    }
  }
  Serial.print("Aucune entrée trouvée avec nbtoping: ");
  Serial.println(nbtoping);
}

// Fonction pour ajouter une nouvelle entrée
void addEntry(const String& id, int nbtrysend, const String& msgtosend) {
  if (entryCount < MAX_ENTRIES) {
    entryList[entryCount].id = id;
    entryList[entryCount].timesend = millis();
    entryList[entryCount].nbtrysend = nbtrysend;
    entryList[entryCount].msgtosend = msgtosend;
    entryCount++;
    Serial.println("Nouvelle entrée ajoutée.");
  } else {
    Serial.println("Liste pleine, impossible d'ajouter une nouvelle entrée.");
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
        Serial.print("Mise à jour de l'entrée ID: ");
        Serial.print(entryList[i].id);
        Serial.print(" -> nbtrysend: ");
        Serial.println(entryList[i].nbtrysend);
        dijkstra(localAddress, getValue(entryList[i].msgtosend, ':', 1).toInt(), entryList[i].msgtosend);
        Serial.println(entryList[i].msgtosend);
        Serial.println(getValue(entryList[i].msgtosend, ':', 1).toInt());
        
        i++; // Passer à l'entrée suivante
      } else {
        // Supprimer l'entrée si nbtrysend > 3
        int despingverif = nextstep(localAddress, getValue(entryList[i].msgtosend, ':', 1).toInt());
        addPingEntry(despingverif, 1);
        removeEdge(localAddress, despingverif);
        sendMessage(1, "ping", despingverif);
        Serial.print("Suppression de l'entrée ID: ");
        Serial.println(entryList[i].id);
        for (int j = i; j < entryCount - 1; j++) {
          entryList[j] = entryList[j + 1];
        }
        entryCount--; // Réduire le nombre d'entrées
      }
    } else {
      i++; // Passer à l'entrée suivante
    }
  }
}

// Fonction pour supprimer une entrée par ID
void removeEntryByID(const String& id) {
  for (int i = 0; i < entryCount; i++) {
    if (entryList[i].id == id) {
      Serial.print("Suppression de l'entrée ID: ");
      Serial.println(entryList[i].id);
      for (int j = i; j < entryCount - 1; j++) {
        entryList[j] = entryList[j + 1];
      }
      entryCount--; // Réduire le nombre d'entrées
      return;
    }
  }
  Serial.print("Aucune entrée trouvée avec l'ID: ");
  Serial.println(id);
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
    Serial.print("Aucune arête trouvée contenant le sommet ");
    Serial.println(vertex);
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
      Serial.print("Poids de l'arête mise à jour avec la moyenne: ");
      Serial.print(v1);
      Serial.print(" - ");
      Serial.print(v2);
      Serial.print(" avec un poids de ");
      Serial.println(edges[i].weight);
      break;
    }
  }

  if (!edgeExists && numEdges < MAX_EDGES) {
    edges[numEdges].vertex1 = v1;
    edges[numEdges].vertex2 = v2;
    edges[numEdges].weight = weight;
    numEdges++;
    Serial.print("Nouvelle arête ajoutée : ");
    Serial.print(v1);
    Serial.print(" - ");
    Serial.print(v2);
    Serial.print(" avec un poids de ");
    Serial.println(weight);
    updateVertices();
  } else if (numEdges >= MAX_EDGES) {
    Serial.println("Le tableau d'arêtes est plein. Impossible d'ajouter une nouvelle arête.");
  }
}

void clearEdges() {
  numEdges = 0;
  Serial.println("Tableau d'arêtes vidé.");
  updateVertices();
}

void printEdges() {
  Serial.println("Tableau d'arêtes :");
  for (int i = 0; i < numEdges; i++) {
    Serial.print(edges[i].vertex1);
    Serial.print(" - ");
    Serial.print(edges[i].vertex2);
    Serial.print(" | Poids : ");
    Serial.println(edges[i].weight);
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
    Serial.println("Source ou destination invalide.");
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
    Serial.println("Aucun chemin trouvé.");
    return false;
  }

  Serial.print("Distance de ");
  Serial.print(src);
  Serial.print(" à ");
  Serial.print(dest);
  Serial.print(" est ");
  Serial.println(dist[destIndex]);

  String path = "";
  printPath(parent, destIndex, path);
  Serial.print("Chemin: ");
  Serial.println(path);

  int nextStep = getNextStep(parent, destIndex, srcIndex);
  Serial.print("Prochaine étape de ");
  Serial.print(src);
  Serial.print(" à ");
  Serial.print(dest);
  Serial.print(" est ");
  Serial.println(nextStep);
  sendMessage(1, outgoing, nextStep);  
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
  Serial.print("Toutes les liaisons impliquant ");
  Serial.print(v);
  Serial.println(" ont été supprimées.");
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
      numEdges--; // Réduire le nombre total d'arêtes
      edgeFound = true;
      Serial.print("Arête supprimée : ");
      Serial.print(v1);
      Serial.print(" - ");
      Serial.println(v2);
      break; // On sort de la boucle après avoir trouvé et supprimé l'arête
    }
  }

  if (!edgeFound) {
    Serial.print("Arête introuvable entre ");
    Serial.print(v1);
    Serial.print(" et ");
    Serial.println(v2);
  }
}

int findNearestVertex(int src) {
  updateVertices(); // Update vertices to ensure they are up-to-date

  int srcIndex = getVertexIndex(src);

  if (srcIndex == -1) {
    Serial.println("Sommet source invalide.");
    return -1;
  }

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

  if (nearestVertex == -1) {
    Serial.println("Aucun sommet voisin trouvé.");
  } else {
    Serial.print("Le sommet le plus proche de ");
    Serial.print(src);
    Serial.print(" est ");
    Serial.print(nearestVertex);
    Serial.print(" avec un poids de ");
    Serial.println(minWeight);
  }

  return nearestVertex;
}

void addValue(String newValue) {
  if (dataCount < MAX_SIZE) {
    dataArray[dataCount].value = newValue;
    dataArray[dataCount].time = millis();
    dataCount++;
    Serial.print("Valeur ajoutée : ");
    Serial.println(newValue);
  } else {
    Serial.println("Erreur : tableau plein !");
  }
}

void removeExpiredValues() {
  unsigned long currentTime = millis();

  for (int i = 0; i < dataCount; i++) {
    if (currentTime - dataArray[i].time >= DELAY) {
      Serial.print("Valeur supprimée : ");
      Serial.println(dataArray[i].value);
      
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
  Serial.println("Tableau actuel :");
  for (int i = 0; i < dataCount; i++) {
    Serial.print("Valeur : ");
    Serial.print(dataArray[i].value);
    Serial.print(", Temps restant : ");
    Serial.println(DELAY - (millis() - dataArray[i].time));
  }
  Serial.println();
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
    Serial.println("startstat : 2");
  }
  if(startstat == 2 && pingphase == 0 && ((millis()/1000) - fpingdel > 30)){
    startstat = 3;
  }
  if(startstat == 3){
   Serial.println("startstat : 3");
   nearestforstart = findNearestVertex(localAddress);
   Serial.println(nearestforstart);
   if(nearestforstart == -1){    
    startstat = 9;
    Serial.println("startstat : 9");
    Serial.println("Echec procedure de demarrage");
    return;
   }
   else{         
    sendMessage(1, "gmap", nearestforstart);  
    starttime = millis() + starttimeout;  
    startstat = 4;
    starttry = 1;
    Serial.println("startstat : 4");
   }
  }
  if(startstat == 4 && millis() >= starttime && starttry < 3){         
    sendMessage(1, "gmap", nearestforstart);
    starttry ++; 
    starttime = millis() + starttimeout;  
    Serial.println("startstat : 4");
    Serial.println("pas de reception map");
  }
  if(startstat == 4 && millis() >= starttime && starttry >= 3){      
    startstat = 3;
    Serial.println("pas de reception map");
    Serial.println("startstat : 3");
    Serial.println(nearestforstart);
    removeEdge(localAddress, nearestforstart);
  }
  if(startstat == 5 && ((millis()/1000) - rtmapdel > 40)){
    startstat = 6;
    Serial.println("startstat : 6");
    }
  if(startstat == 6){
   Serial.println("startstat : 6");
   delay(3000);
   nearestforstart = findNearestVertex(localAddress);
   Serial.println(nearestforstart);
   if(nearestforstart == -1){    
    startstat = 9;
    Serial.println("startstat : 9");
    Serial.println("Echec procedure de demarrage");
    return;
   }
   else{
    timegeth = millis();
    sendMessage(1, "geth", nearestforstart);  
    starttime = millis() + starttimeout;  
    startstat = 7;
    starttry = 1;
    Serial.println("startstat : 7");
   }
  }
  if(startstat == 7 && millis() >= starttime && starttry < 3){
    timegeth = millis();
    sendMessage(1, "geth", nearestforstart);
    starttry ++; 
    starttime = millis() + starttimeout;  
    Serial.println("startstat : 6");
    Serial.println("pas de reception time");
  }
  if(startstat == 7 && millis() >= starttime && starttry >= 3){ 
    Serial.println("pas de reception time");     
    startstat = 6;
    Serial.println("startstat : 6");
    removeEdge(localAddress, nearestforstart);
  }
}

String exportdata(String ver){
  delay(50);
  LoRa.setFrequency(433.1);
  delay(50);
  if (!SD.begin(7)) {Serial.println("PB");}  
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
      Serial.println("PBfile");
    }

    digitalWrite(csPin,0);
    digitalWrite(20,1);
    LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
    LoRa.begin(433E6); 
  
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
  Serial.println(rshval.substring(0, 1));

  if(rshval.substring(0, 1) == "n"){
    float temprecoval = 0;
    int temprecocon = 0;
    int i = 0;
    while(getValue(indump, ';', i) != "") {
      String toandump = getValue(indump, ';', i);
      if(getValue(toandump, ':', 0) == rshval){
        Serial.println(getValue(toandump, ':', 1));
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
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void measuretodump(int ver){
  if(sensorstart == false){
    startsensor();
    sensorstart = true;
  }
  String tosdarg = "";
  if(ver == 1){
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
      Serial.println(F("Failed to begin reading :("));
      return;
    }
    
    if (!bme.endReading()) {
      Serial.println(F("Failed to complete reading :("));
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

    Serial.println(tosdarg);
  }

  if(ver == 2){
    tosdarg += "test:";
    tosdarg += "ok";
    tosdarg += ";";
    tosdarg += "tsp:";
    tosdarg += "okb";
    tosdarg += ";";

    Serial.println(tosdarg);
  }
    delay(50);
    LoRa.setFrequency(433.1);
    SPI.transfer(0);
    digitalWrite(csPin,1);
    digitalWrite(20,0);
    delay(100);

    if (!SD.begin(7)) {Serial.println("PBsd");}
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
      Serial.println("PBfile");
    }

    digitalWrite(csPin,0);
    digitalWrite(20,1);
    LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
    LoRa.begin(433E6); 
  
}

void readsd(bool allrecover){
      LoRa.setFrequency(433.1);
      
      if (!SD.begin(7)) {Serial.println("PB");}  
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
      MAX_PING_AGE = getValue(sdtopar, ':', 0).toInt();
      starttimeout = getValue(sdtopar, ':', 1).toInt();
      localAddress = getValue(sdtopar, ':', 2).toInt();
      DELAY = getValue(sdtopar, ':', 3).toInt();
      MAX_ENTRY_AGE = getValue(sdtopar, ':', 4).toInt();
      actiontimerdel = getValue(sdtopar, ':', 5).toInt();
      maintmode = getValue(sdtopar, ':', 6).toInt();
      stationgateway = getValue(sdtopar, ':', 7).toInt();
      TOGATE_COMMAND_TIMEOUT = getValue(sdtopar, ':', 8).toInt();

      myFile = SD.open("/crontab.cfg", FILE_READ);
      String sdtocron = "";
      if (myFile) {
        while (myFile.available()) {
          sdtocron += (char)myFile.read();
        }
        myFile.close();
      }
      sdtocron = getValue(sdtocron, '\n', 0);
      Serial.print(sdtocron);
      crontabString = sdtocron;
      
      LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
      LoRa.begin(433E6);

}

void writetosd(){
    delay(50);
    LoRa.setFrequency(433.1);
    SPI.transfer(0);
    digitalWrite(csPin,1);
    digitalWrite(20,0);
    delay(100);
    String vartosd = exportEdgesAstmapCommand();
    
    if (!SD.begin(7)) {Serial.println("PB");}
    File testFile = SD.open("/map.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(vartosd);
      testFile.close();
    }
    else{
      Serial.println("PB");
    }

    String varptosd;
    varptosd += MAX_PING_AGE;
    varptosd += ":";
    varptosd += starttimeout;
    varptosd += ":";
    varptosd += localAddress;
    varptosd += ":";
    varptosd += DELAY;
    varptosd += ":";
    varptosd += MAX_ENTRY_AGE;
    varptosd += ":";
    varptosd += actiontimerdel;
    varptosd += ":";
    varptosd += maintmode;
    varptosd += ":";
    varptosd += stationgateway;
    varptosd += ":";
    varptosd += TOGATE_COMMAND_TIMEOUT;
    varptosd += ":";
    
    testFile = SD.open("/p.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(varptosd);
      testFile.close();
    }  
    else{
      Serial.println("PB");
    }

        testFile = SD.open("/crontab.cfg", FILE_WRITE);
    if (testFile) {
      testFile.println(crontabString);
      testFile.close();
    }  
    else{
      Serial.println("PB");
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
      Serial.println("PB");
    }
    digitalWrite(csPin,0);
    digitalWrite(20,1);
    LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
    LoRa.begin(433E6); 
}

bool exportcache() {
  delay(50);
  LoRa.setFrequency(433.1);
  SPI.transfer(0);
  digitalWrite(csPin,1);
  digitalWrite(20,0);
  delay(100);
  if (!SD.begin(7)) {Serial.println("PB");}
  File fichier = SD.open("/togate.cache", FILE_READ);
  if (!fichier) {
    Serial.println("Fichier introuvable.");
    prefs.putBool("incache", false);
    return false;
  }

  uint32_t offset = prefs.getUInt("offset", 0);
  if (offset >= fichier.size()) {
    fichier.close();
    Serial.println("Fin fichier cache");
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
    Serial.print("Ligne lue : ");
    Serial.println(ligne);
    prefs.putUInt("offset", offset);
    fichier.close();
    digitalWrite(csPin,0);
    digitalWrite(20,1);
    LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
    LoRa.begin(433E6);
    String tempstr = ligne;
    int tempid = getValue(tempstr, ':', 5).toInt();
    togateAddCommand(tempid, tempstr);
    interpreter(ligne);
    return true;
  } else {
    fichier.close();
    digitalWrite(csPin,0);
    digitalWrite(20,1);
    LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
    LoRa.begin(433E6);
    return false;
  }

 
}

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  prefs.begin("mycromesh", false);
  
  clearEdges();

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
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

  Serial.println("LoRa init succeeded.");
  Serial.println(localAddress);
  Serial.println(millis());
  actiontimer = (millis()/1000);
}

void loop() {
  checkDelayedCommands();
   removeExpiredValues();
   if(Serial.available()>0)
   {
    interpreter(Serial.readString());
    delay(100);
   }
  onReceive(LoRa.parsePacket());

      
  if(pingphase == 1 && ((millis()/1000) - tmps >= 18 || (millis()/1000) < tmps)){         
    sendMessage(1, "ping", 0);
    pingphase = 2;
    Serial.println("ping phase 2");
    tmps = (millis()/1000);
  }
  if(pingphase == 2 && ((millis()/1000) - tmps >= 18 || (millis()/1000) < tmps)){         
    sendMessage(1, "ping", 0);
    pingphase = 3;
    Serial.println("ping phase 3");
    tmps = (millis()/1000);
  }
  if(pingphase == 3 && ((millis()/1000) - tmps >= 18 || (millis()/1000) < tmps)){         
    pingphase = 0;
    String outgoingumap = exportEdgesContainingVertex(localAddress);
    Serial.println(outgoingumap);
    addValue(getValue(outgoingumap, ':', 1));
    sendMessage(1, outgoingumap, 0);
    Serial.println("fin ping");
    fpingdel = millis() / 1000;
  }

  checkAndRemoveOldEntries();
  checkAndRemoveOldPingEntries();

  if(startstat < 7 && startstat > 0 && maintmode == false){
    startprocedure();
  }

  if(prefs.getBool("incache", 0) == true && prefs.getBool("isgateonline", 0) == true && togateCount == 0 && ((millis()/1000) > (ltgdel + 2) || ltgdel > (millis()/1000))){
    ltgdel = (millis()/1000);
    exportcache();
  }

  if(pingphase == 0 && (startstat == 0 || startstat == 7 || startstat == 8) && entryCount == 0 && pingCount == 0 && togateCount == 0 && ((millis()/1000) - actiontimer >= actiontimerdel || (millis()/1000) < actiontimer && togateCount == 0)){
    if(stationstat == 0 && maintmode == false){      
      stationstat = 1;
      Serial.println("idle");
      writetosd();
      LoRa.receive();
      delay(100);
      Serial.println("Going to sleep now");
      esp_sleep_enable_timer_wakeup((nextWakeup() - 5) * uS_TO_S_FACTOR);
      esp_deep_sleep_enable_gpio_wakeup(1 << 1, ESP_GPIO_WAKEUP_GPIO_HIGH);
      gpio_set_direction((gpio_num_t)1, GPIO_MODE_INPUT);  // <<<=== Add this line
      esp_deep_sleep_start();      
      
      Serial.println("not print");
    }
  }
  else{
    if(stationstat == 1){
      Serial.println("action");
    }
    stationstat = 0;
  }
  executeCronTasks();
  togatePurgeOld();
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

  Serial.println("Erreur : file de commandes pleine.");
}

void checkDelayedCommands() {
  unsigned long currentMillis = millis();
  String tmpcmdd;
  for (int i = 0; i < MAX_COMMANDS; i++) {
    if (commandQueue[i].active && currentMillis >= commandQueue[i].triggerTime) {
      tmpcmdd = commandQueue[i].command;
      commandQueue[i].active = false;
      interpreter(tmpcmdd);
    }
  }
}

void sendMessage(bool wake, String outgoing, int destination) {
  if(wake == true){
    Serial.print("send:");
    Serial.println(destination);
    LoRa.beginPacket();                   // start packet
    LoRa.print("wake");                 // add payload
    LoRa.endPacket();                     // finish packet and send it
    delay(500);
  }
    
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
  actiontimer = (millis()/1000);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  actiontimer = (millis()/1000);

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    //Serial.println("error: message length does not match length");
    ;
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0) {
    return;                             // skip rest of function
  }

  if (recipient == 0) {

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
        delay(300*localAddress);
        sendMessage(1, incoming, 0);
    }
  }
    
    if(getValue(incoming, ':', 0) == "ping"){    
      Serial.println("Snr: " + String(LoRa.packetRssi()));
      delay(300*localAddress); 
      sendMessage(0, "rpin:" + String(LoRa.packetRssi()), sender);
      delay(100);                            // skip rest of function
    }        
    if(getValue(incoming, ':', 0) == "difh"){
      if (!findValue(getValue(incoming, ':', 1))) {
        delay(1000-660);
        rtc.setTime(((getValue(incoming, ':', 2)).toInt())+1);
        Serial.println(getValue(incoming, ':', 2));
        addValue(getValue(incoming, ':', 1));
        delay(300*localAddress);
        String tosendtimestamp = "difh:";
        tosendtimestamp += getValue(incoming, ':', 1);
        tosendtimestamp += ":";
        tosendtimestamp += String(rtc.getLocalEpoch());
        Serial.println(tosendtimestamp);
        sendMessage(1, tosendtimestamp, 0);        
      }
   }
    return;   
  }
  interpreter(incoming);
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
  delay(500);
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
      Serial.print("[Cron Parse Error] Invalid format near: "); Serial.println(cronEntry.substring(0, 20) + "...");
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
      Serial.print("[Cron Parse Error] Expected 5 time fields, found ");
      Serial.print(fieldCount);
      Serial.print(" in: "); Serial.println(timeFields);
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
    Serial.print("stationgateway:");
    Serial.println(stationgateway);
    return;
  }
  if(parn == "MAX_PING_AGE"){
    MAX_PING_AGE = parv.toInt();
    Serial.print("MAX_PING_AGE:");
    Serial.println(MAX_PING_AGE);
  }
  if(parn == "starttimeout"){
    starttimeout = parv.toInt();
    Serial.print("starttimeout:");
    Serial.println(starttimeout);
  }
  if(parn == "localAddress"){
    localAddress = parv.toInt();
    Serial.print("localAddress:");
    Serial.println(localAddress);
  }
  if(parn == "DELAY"){
    DELAY = parv.toInt();
    Serial.print("DELAY:");
    Serial.println(DELAY);
  }
  if(parn == "MAX_ENTRY_AGE"){
    MAX_ENTRY_AGE = parv.toInt();
    Serial.print("MAX_ENTRY_AGE:");
    Serial.println(MAX_ENTRY_AGE);
  }
  if(parn == "actiontimerdel"){
    actiontimerdel = parv.toInt();
    Serial.print("actiontimerdel:");
    Serial.println(actiontimerdel);
  }
}

void interpreter(String msg){
  String cmd = getValue(msg, ':', 0);
  actiontimer = (millis()/1000);

  if(cmd == "read"){  
      readsd(1);
  }  
  if(cmd == "write"){  
     writetosd();
  }  
  if(cmd == "data"){
    Serial.println(msg.substring(5, msg.length()));
  }
    if(cmd == "trsm"){
     delay(500);
     sendMessage(1, msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length()), (getValue(msg, ':', 1)).toInt());
     Serial.println(msg.substring(5+((getValue(msg, ':', 1)).length()+1), msg.length()));
     Serial.println((getValue(msg, ':', 1)).toInt());
  }
    if(cmd == "ping"){    
      Serial.println("Snr: " + String(LoRa.packetRssi()));
      delay(300*localAddress); 
      sendMessage(0, "rpin:" + String(LoRa.packetRssi()), sender);
      delay(100);  
    }    
    if(cmd == "rpin"){
      Serial.println("-----------------------------------------");
      Serial.print("ping : ");
      Serial.println(sender);
      Serial.print("TX : ");
      Serial.println(msg.substring(5, msg.length()));
      Serial.print("RX : ");
      Serial.println(LoRa.packetRssi());
      Serial.print("GLOB : ");
      int glob = ((LoRa.packetRssi())+((msg.substring(5, msg.length())).toInt()))/2;
      glob = abs(glob);
      Serial.println(glob);
      addOrUpdateEdge(localAddress, sender, glob);
      Serial.println("-----------------------------------------");
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
      Serial.println("ping phase 1");
      tmps = (millis()/1000);
    }  
    if(cmd == "dijk"){
      if(getValue(msg, ':', 2).toInt() != localAddress){
        String tempinload = (msg.substring((5+9+(getValue(msg, ':', 1).length())+(getValue(msg, ':', 2).length())+(getValue(msg, ':', 3).length())+3), msg.length()));
        Serial.println(getValue(msg, ':', 3));
        Serial.println(tempinload.length());
        Serial.println(getValue(msg, ':', 4));
        Serial.println(tempinload);
        String rxok = "rxok:";
        rxok += getValue(msg, ':', 4);
        rxok += ":";
        rxok += localAddress;
        if(tempinload.length() == getValue(msg, ':', 3).toInt()){          
          delay(200);
          sendMessage(0, rxok, (getValue(msg, ':', 2)).toInt());
          delay(50);
          if(getValue(msg, ':', 1).toInt() == localAddress){            
            interpreter(tempinload); // execution commande
          }
        }
      }
      if(getValue(msg, ':', 1).toInt() != localAddress){
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
        Serial.println(msgrt);
        if(dijkstra(localAddress, getValue(msg, ':', 1).toInt(), msgrt)){
          Serial.println("trouvé");
          addEntry(getValue(msg, ':', 4), 1, msg);
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
        Serial.println(tmapCommand);
      }
      r++;     
      }
      
    }
    if(cmd == "load"){
      delay(100);
      String tempinload = msg.substring((5+9+(getValue(msg, ':', 1).length())+(getValue(msg, ':', 2).length())+2), msg.length());
      Serial.println(getValue(msg, ':', 2));
      Serial.println(tempinload.length());
      Serial.println(getValue(msg, ':', 3));
      Serial.println(tempinload);
      String rxok = "send:";
      rxok += getValue(msg, ':', 1);
      rxok += ":arok:";
      rxok += getValue(msg, ':', 3);
      rxok += ":";
      rxok += localAddress;
      if(tempinload.length() == getValue(msg, ':', 2).toInt()){
        scheduleCommand(1500, rxok);
        delay(100);
        interpreter(tempinload); // execution commande
      }
    }
    if(cmd == "arok"){
      Serial.print("Message ");
      Serial.print(getValue(msg, ':', 1));
      Serial.print(" bien reçu par ");
      Serial.println(getValue(msg, ':', 2));
      togateRemoveById(getValue(msg, ':', 1).toInt());
    }
    if(cmd == "rxok"){
      Serial.print("Message ");
      Serial.print(getValue(msg, ':', 1));
      Serial.print(" bien reçu par ");
      Serial.println(getValue(msg, ':', 2));
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
      Serial.println(load);
      interpreter(load);
    }
    if(cmd == "send"){
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
      Serial.println(dijk);
      interpreter(dijk);
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
        Serial.println(tmapCommand);
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
      Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
    }
    if(cmd == "seth"){
      unsigned long delaygeth = ((((millis() - timegeth) - 500) / 2));
      Serial.print("delay : ");
      Serial.println(delaygeth);
      delay(1000 - delaygeth);
      rtc.setTime(((getValue(msg, ':', 1)).toInt()) + 1);
      Serial.println(getValue(msg, ':', 1));
      if(startstat == 7){
        startstat = 8;
        Serial.println("Fin procedure de demarrage");        
      }
    }
    if(cmd == "geth"){
      delay(500);
      String tosendtimestamp = "seth:";
      tosendtimestamp += String(rtc.getLocalEpoch());
      Serial.println(tosendtimestamp);
      Serial.println(sender);
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
      Serial.println(maintmode);
    }
    if(cmd == "maint"){
      maintmode = true;
      Serial.println("maintenace");
    }
    if(cmd == "norm"){
      maintmode = false;
      Serial.println("nominal");
    }
    if(cmd == "stam"){
      rtc.setTime((getValue(msg, ':', 1)).toInt());
      startstat = 8;
      Serial.println("Fin procedure de demarrage");
    }
    if(cmd == "fdih"){
      delay(100);
      timegeth = millis();
      sendMessage(1, "geth", sender);
    }    
    if(cmd == "difh"){
      if (!findValue(getValue(msg, ':', 1))) {
        delay(340);
        rtc.setTime(((getValue(msg, ':', 2)).toInt())+1);
        Serial.println(getValue(msg, ':', 2));
        addValue(getValue(msg, ':', 1));
        delay(300*localAddress);
        String tosendtimestamp = "difh:";
        tosendtimestamp += getValue(msg, ':', 1);
        tosendtimestamp += ":";
        tosendtimestamp += String(rtc.getLocalEpoch());
        Serial.println(tosendtimestamp);
        sendMessage(1, tosendtimestamp, 0);        
      }
   }
   if(cmd == "gate"){
      Serial.println(stationgateway);
   }
   if(cmd == "gdfh"){
    String tempid = generateid();
    String tosenddifh = "difh:";
    tosenddifh += tempid;
    tosenddifh += ":";
    tosenddifh += String(rtc.getLocalEpoch());
    sendMessage(1, tosenddifh, 0);
    Serial.println(tosenddifh);
    }
    if(cmd == "dexp"){
      String togate = exportdata(getValue(msg, ':', 1));
      Serial.println(togate);
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
      Serial.println(load);
      togateAddCommand(tempid.toInt(), load);
      interpreter(load);
    }
    if(cmd == "gmea"){
      measuretodump(getValue(msg, ':', 1).toInt());
    }
    if(cmd == "parm"){
      changepval(getValue(msg, ':', 1), getValue(msg, ':', 2));
    }
    if(cmd == "cout"){
      exportcache();
    }
    if(cmd == "clear"){
      clearEdges();
    }
    if(cmd == "cachestate"){
      Serial.print("incache:");
      Serial.println(prefs.getBool("incache", 0));
      Serial.print("isgateonline:");
      Serial.println(prefs.getBool("isgateonline", 0));
      Serial.print("togateCount:");
      Serial.println(togateCount);
    }
    if(cmd == "adrc"){
    int del = getValue(msg, ':', 1).toInt();
    String cmd = getValue(msg, ':', 2);
    Serial.println(del);
    scheduleCommand(del, cmd);
    }
}
