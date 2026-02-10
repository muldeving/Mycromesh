# Mycromesh - Schema Fonctionnel du Code

> Firmware v1.3.0 | ESP32-C3 | LoRa 433 MHz | Reseau Mesh IoT

---

## 1. Vue d'ensemble

Mycromesh est un systeme de reseau mesh base sur la communication radio LoRa, concu pour des noeuds IoT distribues. Il permet la collecte de donnees environnementales (temperature, humidite, pression, gaz), le routage de messages a travers un reseau maille, le transfert de fichiers avec correction d'erreurs Reed-Solomon, et la gestion d'energie via deep sleep.

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MYCROMESH - NOEUD                            │
│                                                                     │
│  ┌───────────┐  ┌──────────┐  ┌───────────┐  ┌──────────────────┐ │
│  │  LoRa     │  │ Capteur  │  │ Carte SD  │  │  ESP32-C3 MCU    │ │
│  │  433 MHz  │  │ BME680   │  │ Stockage  │  │  (Controleur)    │ │
│  │  (SPI)    │  │ (I2C)    │  │ (SPI)     │  │                  │ │
│  └─────┬─────┘  └────┬─────┘  └─────┬─────┘  └────────┬─────────┘ │
│        │              │              │                  │           │
│        └──────────────┴──────────────┴──────────────────┘           │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Architecture Modulaire

```
┌─────────────────────────────────────────────────────────────────────────┐
│                                                                         │
│                          BOUCLE PRINCIPALE                              │
│                            loop()                                       │
│                                                                         │
│   ┌─────────┐  ┌──────────┐  ┌──────────┐  ┌────────┐  ┌───────────┐ │
│   │ Serial  │  │ LoRa RX  │  │ Timers & │  │ Cron   │  │ Deep      │ │
│   │ Input   │  │ Receive  │  │ Queues   │  │ Tasks  │  │ Sleep     │ │
│   └────┬────┘  └────┬─────┘  └────┬─────┘  └───┬────┘  └─────┬─────┘ │
│        │            │             │             │              │        │
│        └────────────┴─────┬───────┴─────────────┘              │        │
│                           │                                    │        │
│                    ┌──────▼──────┐                              │        │
│                    │ interpreter │◄─────────────────────────────┘        │
│                    │  (dispatch) │                                       │
│                    └──────┬──────┘                                       │
│                           │                                             │
│         ┌─────────────────┼─────────────────────┐                       │
│         │                 │                     │                       │
│   ┌─────▼─────┐   ┌──────▼──────┐   ┌─────────▼──────────┐            │
│   │  Reseau   │   │  Transfert  │   │  Donnees &         │            │
│   │  Mesh     │   │  Fichiers   │   │  Capteurs          │            │
│   └───────────┘   └─────────────┘   └────────────────────┘            │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Modules Fonctionnels Detailles

### 3.1. Initialisation (setup)

```
setup()
  │
  ├── Serial.begin(115200)           // Communication serie
  ├── prefs.begin("mycromesh")       // Preferences persistantes
  ├── initGaloisField()              // Tables GF(256) pour Reed-Solomon
  ├── LoRa.begin(433E6)             // Radio LoRa @ 433.1 MHz
  │     ├── CS Pin: 21
  │     ├── IRQ Pin: 8
  │     └── Reset Pin: -1
  ├── rtc (ESP32Time)               // Horloge temps reel
  ├── readsd(recover)                // Chargement config depuis SD
  │     ├── /p.cfg                   //   Parametres station
  │     ├── /map.cfg                 //   Topologie reseau
  │     ├── /e.cfg                   //   Etat environnement
  │     └── /crontab.cfg            //   Taches planifiees
  └── startsensor()                  // Capteur BME680 (optionnel)
```

### 3.2. Boucle Principale (loop)

```
loop()
  │
  ├──[1] Lecture Serial ──────────► interpreter(input)
  │
  ├──[2] Reception LoRa
  │       onReceive(packetSize)
  │       ├── Lecture entete (dest, source, msgID, longueur)
  │       ├── Filtrage: message pour cette station ou broadcast?
  │       └── scheduleCommand(delai, message)
  │
  ├──[3] Execution commandes differees
  │       checkDelayedCommands()
  │       └── Pour chaque commande active dont triggerTime est passe
  │           └── interpreter(command)
  │
  ├──[4] Phases de ping (decouverte reseau)
  │       pingphase 1 → Envoi pings aux voisins
  │       pingphase 2 → Attente reponses
  │       pingphase 3 → Traitement resultats
  │
  ├──[5] Procedure de demarrage (startstat 0-8)
  │       (voir section 3.5)
  │
  ├──[6] Export cache gateway
  │       exportcache() → Reenvoi des commandes en echec
  │
  ├──[7] Export fichier en cours
  │       exportfile() → Transmission ligne par ligne
  │
  ├──[8] Taches Cron
  │       executeCronTasks()
  │
  └──[9] Gestion Deep Sleep
          Si inactif et pas de taches en cours
          └── Calcul prochaine echeance cron → deep sleep
```

### 3.3. Interpreteur de Commandes

```
interpreter(msg)
  │
  ├── COMMANDES RESEAU
  │   ├── "ping"  → Diffusion broadcast pour decouvrir les voisins
  │   ├── "rpin"  → Reponse au ping avec RSSI (force du signal)
  │   ├── "umap"  → Mise a jour de la topologie (ajout/suppression aretes)
  │   ├── "dijk"  → Routage de message via Dijkstra
  │   ├── "gmap"  → Demande de la carte reseau complete
  │   ├── "tmap"  → Envoi de la carte reseau
  │   └── "prnt"  → Affichage des aretes du graphe
  │
  ├── COMMANDES TEMPS
  │   ├── "geth"  → Demande de l'heure a un noeud
  │   ├── "seth"  → Reglage de l'heure
  │   ├── "difh"  → Synchronisation horaire distribuee
  │   ├── "fdih"  → Reponse synchronisation
  │   ├── "acth"  → Demande asynchrone de l'heure
  │   └── "stam"  → Reglage heure avec etat
  │
  ├── COMMANDES TRANSFERT FICHIER
  │   ├── "stft"  → Demarrer un transfert de fichier
  │   ├── "isrf"  → Confirmation reception prete
  │   ├── "rfok"  → Acquittement fichier recu
  │   ├── "fend"  → Fin de transfert
  │   ├── "load"  → Envoi de donnees a un destinataire
  │   └── "large" → Gestion messages > 225 octets
  │
  ├── COMMANDES DONNEES
  │   ├── "dexp"  → Export des donnees capteur
  │   ├── "trsp"  → Envoi transparent a un autre noeud
  │   ├── "send"  → Envoi message (avec reveil)
  │   └── "trsm"  → Envoi message (sans reveil)
  │
  └── COMMANDES CONTROLE
      ├── "read"     → Lecture fichier carte SD
      ├── "write"    → Ecriture fichier carte SD
      ├── "maint"    → Mode maintenance (ON)
      ├── "norm"     → Mode normal
      ├── "reboot"   → Redemarrage
      ├── "upgrade"  → Mise a jour firmware OTA
      └── "star"     → Lancer la procedure de demarrage
```

### 3.4. Reseau Mesh & Routage

```
TOPOLOGIE RESEAU (Graphe)
═══════════════════════════

  Station A ────(poids: RSSI)──── Station B
      │                               │
      │         Station E              │
      │        /         \             │
  Station C ──            ── Station D
      │                        │
       \                      /
        ── Station F ────────


STRUCTURES DE DONNEES:
  edges[MAX_EDGES=300]    →  { vertex1, vertex2, weight }
  vertices[MAX_VERTICES=60] →  [ id1, id2, ... ]


FONCTIONS PRINCIPALES:

  addOrUpdateEdge(v1, v2, weight)
    │ Ajoute ou met a jour une connexion dans le graphe
    └── Poids = force du signal (RSSI inverse)

  removeEdgesByVertex(v)
    └── Supprime toutes les connexions d'un noeud

  dijkstra(src, dest, message)
    │ Algorithme du plus court chemin
    ├── Initialise distances a INT_MAX
    ├── Relaxation iterative des aretes
    ├── Reconstruction du chemin
    └── sendMessage() vers le prochain saut

  findNearestVertex(src)
    └── Trouve le noeud le plus proche accessible

  exportEdgesAstmapCommand()
    └── Exporte la topologie en commandes "umap"
```

### 3.5. Procedure de Demarrage

```
MACHINE A ETATS DE DEMARRAGE (startstat)
════════════════════════════════════════

  [0] IDLE
   │
   ▼
  [1] PING DISCOVERY ──────► Broadcast "ping" aux voisins
   │
   ▼
  [2] ATTENTE REPONSES ────► Collecte des "rpin" (timeout: starttimeout)
   │
   ▼
  [3] RECHERCHE NOEUD ─────► findNearestVertex() pour demande carte
   │
   ▼
  [4] DEMANDE CARTE ───────► Envoi "gmap" au noeud le plus proche
   │                         (max 4 tentatives)
   ▼
  [5] ATTENTE CARTE ───────► Reception de la topologie "tmap"
   │                         (timeout: starttimeout)
   ▼
  [6] RECHERCHE TEMPS ─────► findNearestVertex() pour synchro horloge
   │
   ▼
  [7] DEMANDE HEURE ───────► Envoi "geth" (max 4 tentatives)
   │                         (timeout: starttimeout)
   ▼
  [8] OPERATIONNEL ────────► Noeud pret, reseau synchronise
```

### 3.6. Transfert de Fichiers avec Correction d'Erreurs

```
TRANSFERT DE FICHIER (Reed-Solomon)
════════════════════════════════════

  EMETTEUR                                    RECEPTEUR
  ════════                                    ═════════

  parseFile(path)
    ├── Lecture fichier
    ├── Decoupage en paquets
    │   (PACKET_SIZE = 150 octets)
    ├── Groupes de GROUP_K=8 paquets
    ├── encodeGroupParity()
    │   Genere PARITY_M=8 paquets parite
    ├── CRC32 par paquet
    └── Ecriture /tx.txt
         │
         ▼
  "stft" ──────────────────────────────► Reception demande
                                              │
                                              ▼
  exportfile() ◄──────────────────────── "isrf" (pret)
    │
    ├── Envoi ligne par ligne
    │   META: totalPkts:fileCRC:size:groups
    │   DATA: id:crc:hexdata
    │   PARITY: group:copy:crc:hexdata
    │
    └── "fend" ────────────────────────► compileFile()
                                           ├── Verification CRC32
                                           ├── Paquets manquants?
                                           │   └── invertMatrixGF()
                                           │       Reconstruction
                                           │       Reed-Solomon
                                           ├── Assemblage fichier
                                           └── "rfok" ──► Confirmation


GALOIS FIELD GF(256):
  ┌──────────────────────────────────────────────┐
  │  initGaloisField()  → Tables exp/log        │
  │  gfMul(a,b)         → Multiplication        │
  │  gfPow(x,power)     → Exponentiation        │
  │  gfInv(a)           → Inversion             │
  │  invertMatrixGF()   → Inversion matrice     │
  │  encodeGroupParity()→ Encodage parite       │
  └──────────────────────────────────────────────┘
```

### 3.7. Communication LoRa

```
ENVOI DE MESSAGE
════════════════

  sendMessage(wake, destination, message)
    │
    ├── wake == true?
    │   └── Envoi preambule de reveil (100 repetitions)
    │       pour reveiller les noeuds en deep sleep
    │
    ├── Construction paquet:
    │   ┌────────────┬────────────┬─────────┬──────────┬───────────┐
    │   │ Dest (1B)  │ Source(1B) │ MsgID   │ Longueur │ Payload   │
    │   │            │            │  (1B)   │  (1B)    │ (N bytes) │
    │   └────────────┴────────────┴─────────┴──────────┴───────────┘
    │
    └── LoRa.endPacket() → Transmission radio


RECEPTION DE MESSAGE
═══════════════════

  onReceive(packetSize)
    │
    ├── Lecture entete (dest, source, msgID, longueur)
    │
    ├── Filtrage:
    │   ├── destination == localAddress  → Message pour nous
    │   ├── destination == 0xFF         → Broadcast
    │   └── autre                       → Ignorer
    │
    ├── Extraction RSSI (force signal)
    │
    └── scheduleCommand(delai_aleatoire, message)
        └── Mise en file d'attente pour traitement
```

### 3.8. Gestion de la Passerelle (Gateway)

```
SYSTEME DE FILE GATEWAY (togateQueue)
══════════════════════════════════════

  Message a envoyer vers la passerelle
    │
    ▼
  togateAddCommand(id, command)
    │ Ajout a la file en memoire
    │ (max MAX_TOGATE_COMMANDS = 10)
    │
    ├── Envoi via dijkstra() vers stationgateway
    │
    ├── Succes? ──► togateRemoveById(id)
    │               prefs.putBool("isgateonline", true)
    │
    └── Timeout? ──► togatePurgeOld()
                     ├── Sauvegarde dans /togate.cache (SD)
                     ├── nbtogatefail++
                     └── Si >= 5 echecs → Gateway hors ligne


  FILE BASEE FICHIER (togateQueueFile)
  ─────────────────────────────────────
  Pour les transferts de fichier volumineux
    ├── togateAddCommandFile(id, command)
    ├── togateRemoveByIdFile(id)
    └── purgeToOldFile() → Remet les commandes expirees dans /tx.txt


  REPRISE APRES ECHEC:
  ─────────────────────
  exportcache()
    └── Relit /togate.cache depuis SD
        └── Reenvoi des commandes en attente vers la passerelle
```

### 3.9. Capteurs & Donnees

```
CAPTEUR BME680
══════════════

  startsensor()
    └── Configuration oversampling:
        ├── Temperature:  x8
        ├── Pression:     x4
        ├── Humidite:     x2
        └── Filtre IIR:   taille 3

  measuretodump(version)
    ├── bme.performReading()
    ├── Lecture:
    │   ├── Temperature (C)
    │   ├── Humidite (%)
    │   ├── Pression (hPa)
    │   └── Resistance gaz (Ohms)
    ├── Formatage: "localAddress,T,H,P,G,timestamp"
    └── Ecriture → /version.datadump (SD)

  dexp (export donnees)
    └── Lecture du fichier .datadump
        └── Envoi vers la passerelle via le reseau mesh
```

### 3.10. Planificateur Cron

```
SYSTEME CRON
════════════

  Format: "minute heure jour mois jourSemaine tache;..."

  Exemples:
    "*/5 * * * * measuretodump"    → Mesure toutes les 5 min
    "0 8 * * 1 dexp"               → Export chaque lundi a 8h
    "30 * * * * ping"              → Ping toutes les heures a :30


  executeCronTasks()
    │
    ├── Lecture crontabString (depuis /crontab.cfg)
    ├── Decoupe par ";" en taches individuelles
    ├── Pour chaque tache:
    │   ├── Parse: minute heure jour mois jourSemaine
    │   ├── checkCronField(valeur, champActuel)
    │   │   ├── "*"     → Toujours vrai
    │   │   ├── "*/N"   → Vrai si (actuel % N == 0)
    │   │   ├── "a"     → Remplace par localAddress
    │   │   └── valeur  → Comparaison directe
    │   └── Si tous les champs correspondent → interpreter(tache)
    │
    └── lastMinuteChecked = minute actuelle
        (empeche la double execution)


  nextWakeup()
    │ Calcule la prochaine echeance cron
    └── Retourne le delai en secondes pour le deep sleep
```

### 3.11. Gestion d'Energie

```
DEEP SLEEP
══════════

  Conditions d'entree:
    ├── maintmode == false (pas en maintenance)
    ├── Pas de transfert fichier en cours
    ├── Pas de commandes en attente
    ├── Procedure de demarrage terminee (startstat == 8 ou 0)
    └── Delai d'inactivite depasse

  Entree en deep sleep:
    ├── writetosd()  → Sauvegarde etat sur SD
    ├── Calcul duree = nextWakeup() - 5 secondes
    ├── esp_deep_sleep_enable_timer_wakeup(duree)
    ├── esp_sleep_enable_ext0_wakeup(GPIO1, LOW)  → Reveil materiel
    └── esp_deep_sleep_start()

  Sources de reveil:
    ├── Timer → Echeance cron atteinte
    └── GPIO1 → Interruption materielle externe
```

---

## 4. Structures de Donnees

```
┌──────────────────────────────────────────────────────────────────┐
│                     STRUCTURES PRINCIPALES                       │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Edge { vertex1, vertex2, weight }                               │
│  └── edges[300] : Connexions du graphe mesh                     │
│                                                                  │
│  Entry { id, timesend, nbtrysend, msgtosend }                   │
│  └── entryList[10] : Messages en attente d'acquittement         │
│                                                                  │
│  DelayedCommand { triggerTime, command, active }                 │
│  └── commandQueue[10] : Commandes a executer en differe         │
│                                                                  │
│  TogateCommand { id, command, timestamp }                        │
│  └── togateQueue[10] : File passerelle en memoire               │
│                                                                  │
│  TogateCommandFile { id, command, timestamp }                    │
│  └── togateQueueFile[20] : File passerelle pour fichiers        │
│                                                                  │
│  PingEntry { nbtoping, pingTime, nbping }                        │
│  └── pingList[10] : Decouverte de voisinage en cours            │
│                                                                  │
│  Data { value, time }                                            │
│  └── dataArray[10] : Tampon temporaire de donnees               │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 5. Fichiers de Configuration (Carte SD)

```
/
├── p.cfg              Parametres station
│                      (adresse, timeouts, delais, gateway)
├── map.cfg            Topologie reseau
│                      Format: "v1,v2,poids" par ligne
├── e.cfg              Etat environnement
│                      (startstat, compteur messages)
├── crontab.cfg        Taches planifiees (format cron)
├── data.ver           Versions de structure de donnees
├── togate.cache       Commandes passerelle en echec
├── tx.txt             Fichier prepare pour emission (avec RS)
├── rx.txt             Tampon d'assemblage reception
├── large.cmd          Staging pour messages volumineux
├── *.datadump         Accumulation des mesures capteur
└── update/
    └── firmware.bin   Firmware pour mise a jour OTA
```

---

## 6. Diagramme de Flux Global

```
                    ┌─────────┐
                    │ POWER   │
                    │  ON     │
                    └────┬────┘
                         │
                    ┌────▼────┐
                    │ setup() │
                    └────┬────┘
                         │
              ┌──────────▼──────────┐
              │      loop()         │◄─────────────────────────────┐
              └──────────┬──────────┘                              │
                         │                                         │
          ┌──────────────┼──────────────┐                          │
          │              │              │                          │
     ┌────▼────┐   ┌─────▼─────┐  ┌────▼─────┐                   │
     │ Serial  │   │ LoRa RX   │  │ Timers   │                   │
     │ Input   │   │ onReceive │  │ Cron     │                   │
     └────┬────┘   └─────┬─────┘  └────┬─────┘                   │
          │              │              │                          │
          └──────────────┼──────────────┘                          │
                         │                                         │
                  ┌──────▼──────┐                                  │
                  │ interpreter │                                  │
                  └──────┬──────┘                                  │
                         │                                         │
       ┌─────────────────┼─────────────────┐                      │
       │                 │                 │                      │
  ┌────▼─────┐    ┌──────▼──────┐   ┌──────▼──────┐              │
  │ Routage  │    │ Fichiers &  │   │ Capteurs &  │              │
  │ Mesh     │    │ Correction  │   │ Donnees     │              │
  │ Dijkstra │    │ Reed-Solomon│   │ BME680      │              │
  └────┬─────┘    └──────┬──────┘   └──────┬──────┘              │
       │                 │                 │                      │
       └─────────────────┼─────────────────┘                      │
                         │                                         │
                  ┌──────▼──────┐                                  │
                  │ sendMessage │                                  │
                  │ (LoRa TX)   │                                  │
                  └──────┬──────┘                                  │
                         │                                         │
                  ┌──────▼──────┐     ┌────────────┐              │
                  │ Inactivite? │─NO──►│ Retour     │──────────────┘
                  └──────┬──────┘     │ loop()     │
                        YES           └────────────┘
                         │
                  ┌──────▼──────┐
                  │ Deep Sleep  │
                  │ (Timer/GPIO)│
                  └──────┬──────┘
                         │
                    ┌────▼────┐
                    │ REVEIL  │──────► setup() (reinitialisation)
                    └─────────┘
```

---

## 7. Brochage Materiel

```
ESP32-C3
════════

  Pin 21 ──── CS LoRa (SPI Chip Select)
  Pin  8 ──── IRQ LoRa (Interruption)
  Pin  7 ──── CS SD Card (SPI Chip Select)
  Pin 20 ──── Controle module LoRa (GPIO)
  Pin  1 ──── Entree reveil (Deep Sleep Wakeup)

  SPI Bus  ── Partage entre LoRa et SD Card
               (commutation GPIO pour multiplexage)

  I2C Bus  ── Capteur BME680
               (Temperature, Humidite, Pression, Gaz)
```

---

## 8. Protocole de Communication

```
FORMAT PAQUET LoRa:
═══════════════════

  ┌────────────┬────────────┬──────────┬──────────┬─────────────┐
  │ Destinat.  │  Source     │  MsgID   │ Longueur │  Payload    │
  │  (1 octet) │  (1 octet) │ (1 oct.) │ (1 oct.) │ (N octets)  │
  └────────────┴────────────┴──────────┴──────────┴─────────────┘

  Taille max payload: ~225 octets

FORMAT TEXTE (separateur ':'):
═════════════════════════════

  Message simple:   "commande:param1:param2:..."
  Message route:    "dijk:source:dest:longueur:id:payload"
  Transfert fichier:"stft:noeudRX:chemin:noeudOrigine"
  Mise a jour topo: "umap:id:vertex:aretes..."

FORMAT FICHIER TRANSFERT (avec Reed-Solomon):
═════════════════════════════════════════════

  META:totalPaquets:CRCfichier:tailleAttendue:groupesParite:K:M
  DATA:idPaquet:CRCpaquet:
  DATA_CONT:donneesHex...
  PARITY:idGroupe:numCopie:CRCparite:
  PARITY_CONT:donneesHex...
```

---

## 9. Constantes et Limites

| Parametre               | Valeur | Description                          |
|--------------------------|--------|--------------------------------------|
| MAX_VERTICES             | 60     | Nombre max de stations dans le mesh  |
| MAX_EDGES                | 300    | Nombre max de connexions             |
| MAX_ENTRIES              | 10     | Messages en vol simultanes           |
| MAX_PINGS                | 10     | Pings simultanes                     |
| MAX_COMMANDS             | 10     | File de commandes differees          |
| MAX_TOGATE_COMMANDS      | 10     | File passerelle (memoire)            |
| MAX_TOGATE_COMMANDS_FILE | 20     | File passerelle (fichiers)           |
| PACKET_SIZE              | 150    | Taille paquet RS (octets)            |
| GROUP_K                  | 8      | Paquets donnees par groupe RS        |
| PARITY_M                 | 8      | Paquets parite par groupe RS         |
| TAILLE_TAMPON            | 256    | Buffer I/O fichier                   |
| filetimeout              | 300    | Timeout reception fichier (sec)      |
| filetxtimeout            | 60     | Timeout emission fichier (sec)       |
| Frequence LoRa           | 433.1  | MHz                                  |
