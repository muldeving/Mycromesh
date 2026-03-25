# FSD — MycroMesh Gateway & Serveur

> **Functional Specification Document**
> Version 1.0 | Composant : `mycromesh-gateway-server`
> Projet : [MycroMesh](https://github.com/muldeving/Mycromesh)

---

## Table des matières

1. [Objectif et périmètre](#1-objectif-et-périmètre)
2. [Vue d'ensemble du système](#2-vue-densemble-du-système)
3. [Architecture réseau](#3-architecture-réseau)
4. [Protocole de liaison Gate ↔ Serveur](#4-protocole-de-liaison-gate--serveur)
5. [Canal Terminal](#5-canal-terminal)
6. [Canal Commandes](#6-canal-commandes)
7. [Traitement des données `data:`](#7-traitement-des-données-data)
8. [Transfert de fichiers](#8-transfert-de-fichiers)
9. [Fiabilité et persistance](#9-fiabilité-et-persistance)
10. [Interfaces web](#10-interfaces-web)
11. [Sécurité](#11-sécurité)
12. [Stack technique](#12-stack-technique)
13. [Schéma de base de données](#13-schéma-de-base-de-données)
14. [Phases d'implémentation](#14-phases-dimplémentation)
15. [Contraintes et limites](#15-contraintes-et-limites)

---

## 1. Objectif et périmètre

Ce document spécifie le comportement fonctionnel attendu du sous-système **Gateway + Serveur** du projet MycroMesh. Il couvre :

- La **Gate** : passerelle matérielle ESP32 faisant l'interface entre le réseau mesh LoRa et le serveur.
- Le **Serveur** : service Python gérant la Gate, traitant les données reçues, effectuant compilations et parsing, et exposant des interfaces web.

Ce document ne couvre pas le firmware des stations MycroMesh (ESP32-C3 + LoRa), qui fait l'objet du `SCHEMA_FONCTIONNEL.md`.

---

## 2. Vue d'ensemble du système

```
  Réseau MycroMesh (LoRa 433 MHz)
         │
         │  Radio LoRa
         ▼
  ┌─────────────────────────────────────────┐
  │               GATE (ESP32)              │
  │                                         │
  │  ┌──────────────┐  ┌──────────────────┐ │
  │  │  Wi-Fi natif │  │ Ethernet LAN8720 │ │
  │  └──────┬───────┘  └────────┬─────────┘ │
  │         │                  │            │
  │         └────────┬─────────┘            │
  │                  │ (dual-stack)         │
  │  ┌───────────────▼────────────────────┐ │
  │  │       Carte SD (buffer offline)    │ │
  │  └────────────────────────────────────┘ │
  └─────────────────┬───────────────────────┘
                    │
         Canal Terminal  +  Canal Commandes
         (WebSocket ou TCP custom, port dédié)
                    │
  ┌─────────────────▼───────────────────────┐
  │            SERVEUR (Python)             │
  │                                         │
  │  ┌────────────┐  ┌─────────────────┐    │
  │  │  Parseur   │  │  MySQL          │    │
  │  │  data.ver  │  │  (tables/ver.)  │    │
  │  └────────────┘  └─────────────────┘    │
  │  ┌────────────┐  ┌─────────────────┐    │
  │  │  Fichiers  │  │  Apache2 + PHP  │    │
  │  │  compiler  │  │  (terminal web, │    │
  │  │  / parser  │  │   transferts)   │    │
  │  └────────────┘  └─────────────────┘    │
  └─────────────────────────────────────────┘
```

---

## 3. Architecture réseau

### 3.1 Gate — double interface réseau

La Gate utilise **deux interfaces réseau simultanées** sur le même ESP32 :

| Interface | Composant | Usage |
|-----------|-----------|-------|
| Wi-Fi | Intégré ESP32 | Liaison Gate ↔ Serveur (réseau local) |
| Ethernet | LAN8720 (SPI) | Liaison Gate ↔ Serveur (câblé, prioritaire) |

**Broches Ethernet LAN8720 :**

```cpp
ETH_CLK_MODE  = ETH_CLOCK_GPIO17_OUT
ETH_POWER     = GPIO 12
ETH_TYPE      = ETH_PHY_LAN8720
ETH_ADDR      = 1
ETH_MDC       = GPIO 23
ETH_MDIO      = GPIO 18
```

### 3.2 Comportement réseau attendu

- Les deux interfaces doivent être initialisées au démarrage.
- La liaison Ethernet est **prioritaire** sur le Wi-Fi lorsque les deux sont disponibles.
- En cas de perte de l'interface active, la Gate bascule automatiquement sur l'interface disponible.
- La Gate doit maintenir les deux canaux (terminal + commandes) actifs en permanence.

---

## 4. Protocole de liaison Gate ↔ Serveur

### 4.1 Contraintes

- **MQTT est exclu** comme protocole de liaison Gate ↔ Serveur.
- Le protocole retenu ne doit pas entrer en **conflit de ports** avec les services existants sur le serveur :

| Service existant | Port(s) |
|-----------------|---------|
| Apache2 + PHP | 80, 443 |
| FTP | 20, 21 |
| SSH | 22 |
| VPN (OpenVPN / WireGuard) | 1194 / 51820 |
| Grafana | 3000 |
| Home Assistant | 8123 |

### 4.2 Solution retenue

**WebSocket** ou **TCP socket custom** géré par le service Python, sur un **port dédié documenté** (ex. : 8765).

Caractéristiques requises :
- Communication **persistante et bidirectionnelle**.
- Gestion simultanée de **deux canaux logiques** sur la même connexion physique :
  - **Canal Terminal** : I/O interactif de la Gate (équivalent port série USB).
  - **Canal Commandes** : transfert des commandes exécutées par l'interpréteur de la Gate.
- Reconnexion automatique côté Gate en cas de déconnexion.

### 4.3 Format des messages sur le canal

Chaque message échangé est préfixé par son type de canal :

```
TERM:<payload>     → Canal Terminal
CMD:<payload>      → Canal Commandes
ACK:<id>           → Accusé de réception (commandes data:)
FILE:<payload>     → Canal transfert de fichiers
```

---

## 5. Canal Terminal

### 5.1 Rôle

Le canal Terminal reproduit le comportement d'un **port série USB** branché à la Gate :
- Tout caractère saisi dans l'interface web est transmis à la Gate (stdin).
- Toute sortie de la Gate est redirigée vers l'interface web (stdout).

### 5.2 Comportement Gate

- La Gate transmet toute sortie série sur le canal Terminal.
- La Gate lit toute entrée reçue sur le canal Terminal et l'injecte dans son interpréteur de commandes.

### 5.3 Comportement Serveur

- Le serveur maintient un **buffer de session** par connexion web active.
- Les données du canal Terminal sont relayées en temps réel vers le client web via WebSocket HTTP (Apache2 → navigateur).
- L'historique de la session est conservé en mémoire (pas de persistance obligatoire).

---

## 6. Canal Commandes

### 6.1 Rôle

Toutes les commandes exécutées par l'interpréteur de la Gate sont transmises au serveur via ce canal.

### 6.2 Comportement Gate

- Chaque commande exécutée est envoyée au serveur **avant** ou **pendant** son exécution locale.
- Les commandes préfixées `data:` sont **exclusivement** transmises au serveur et **ne sont pas interprétées localement** par la Gate.

### 6.3 Comportement Serveur

- Le serveur **logue toutes les commandes** reçues sans exception (horodatage, identifiant source, commande complète).
- Le serveur **traite activement** les commandes `data:` (voir section 7).
- Les autres commandes peuvent déclencher des traitements spécifiques configurables.

---

## 7. Traitement des données `data:`

### 7.1 Format général

Chaque commande `data:` émise par une station MycroMesh suit la structure :

```
data:TXstation:version:valeur1:valeur2:...:valeurN
```

| Champ | Description |
|-------|-------------|
| `TXstation` | Identifiant de la station émettrice |
| `version` | Numéro de version du schéma (référence `data.ver`) |
| `valeur1..N` | Valeurs des champs dans l'ordre défini par la version |

### 7.2 Convention de typage des champs

Le préfixe du nom de champ indique son type :

| Préfixe | Type |
|---------|------|
| `n` | Valeur numérique |
| `t` | Valeur texte |

### 7.3 Fichier `data.ver` — référence des versions de schéma

Le fichier `data.ver` est partagé entre les stations du réseau et le serveur. Il définit pour chaque version la liste ordonnée des champs attendus.

| Version | Champs | Sémantique |
|---------|--------|------------|
| `0` | `nbatt`, `tstartstat`, `tmaintmode`, `trxglob`, `trxbrd`, `trxloc`, `tdijktx`, `tdijkrx`, `tdijkhop`, `tresend1`, `tresend2`, `ttxcnt`, `ntime` | Statistiques de fonctionnement station |
| `1` | `ntemp`, `nhum`, `npres`, `nres`, `ntime` | Mesures environnementales complètes (BME680) |
| `2` | `test`, `tsp` | Données de test / débogage |
| `3` | `ntemp`, `nhum`, `npres`, `ntime` | Mesures environnementales sans résistance gaz |
| `4` | `ntemp`, `ntime` | Mesure minimale (température + horodatage) |

### 7.4 Traitement côté serveur

```
Réception commande data:
        │
        ▼
Lecture data.ver → identification version
        │
        ▼
Extraction des valeurs dans l'ordre de la version
        │
        ▼
Mappage champs → colonnes table MySQL (table_v<version>)
        │
        ▼
INSERT dans la table correspondante
        │
        ▼
Envoi ACK:<id> vers la Gate
```

- Chaque version de schéma dispose de sa **propre table MySQL dédiée**, nommée `data_v<version>` (ex. : `data_v0`, `data_v1`).
- Les schémas de tables évoluent **indépendamment** d'une version à l'autre.
- En cas d'erreur d'insertion, le serveur renvoie un NACK et logue l'erreur ; la Gate conserve la commande pour renvoi.

---

## 8. Transfert de fichiers

### 8.1 Vue d'ensemble

Les transferts de fichiers sont **bidirectionnels** entre la Gate et le serveur. Le serveur centralise toute la charge de calcul (compilation, parsing).

```
Gate                              Serveur
 │                                   │
 │── rx.txt ────────────────────────►│  (Gate → Serveur)
 │                                   │  parsing / compilation
 │◄─────────────────── tx.txt ───────│  (Serveur → Gate)
 │           ou firmware compilé     │
```

### 8.2 Rôles du serveur

| Action | Description |
|--------|-------------|
| Réception `rx.txt` | Reçoit le fichier brut depuis la Gate |
| Parsing | Analyse et valide le contenu du fichier |
| Compilation | Génère le fichier compilé ou `tx.txt` |
| Renvoi | Retransmet le résultat à la Gate |

### 8.3 Diffusion et envoi ciblé

Le serveur expose deux opérations de distribution vers le réseau MycroMesh, **via la Gate** :

| Opération | Description |
|-----------|-------------|
| **Broadcast** | Diffuse un fichier vers toutes les stations du réseau MycroMesh |
| **Send ciblé** | Envoie un fichier à une station spécifique identifiée par son adresse |

Les deux opérations transitent par la Gate, qui prend en charge le routage LoRa.

### 8.4 API serveur (appel interne)

```python
# Diffusion broadcast vers tout le réseau
server.broadcast_file(filepath: str) -> bool

# Envoi ciblé vers une station
server.send_file(filepath: str, target_station: str) -> bool
```

### 8.5 Flux de transfert Gate ↔ Serveur

```
Serveur                            Gate
   │                                │
   │── FILE:stft:<nom>:<taille> ───►│  Demande de transfert
   │◄──────────────── FILE:isrf ────│  Gate prête à recevoir
   │── FILE:data:<ligne> ──────────►│  Envoi ligne par ligne
   │── FILE:data:<ligne> ──────────►│  ...
   │── FILE:fend ──────────────────►│  Fin de transfert
   │◄──────────────── FILE:rfok ────│  Accusé de réception OK
```

---

## 9. Fiabilité et persistance

### 9.1 Garantie de livraison des commandes `data:`

La Gate garantit la réception de chaque commande `data:` par le serveur via un mécanisme **ACK/NACK** :

```
Gate                              Serveur
 │                                   │
 │── CMD:data:<payload> ────────────►│
 │                                   │  traitement...
 │◄─────────────────── ACK:<id> ─────│
 │  (suppression de la file locale)  │
```

En l'absence d'ACK dans un délai configuré, la Gate :
1. Remet la commande en file d'attente.
2. Retente l'envoi jusqu'à épuisement des tentatives.
3. En cas d'échec persistant, passe à l'étape 9.2.

### 9.2 Stockage SD en cas de liaison perdue

Si la liaison Gate ↔ Serveur est perdue :

- Les commandes `data:` en attente sont **sérialisées sur la carte SD** dans `/togate.cache`.
- À la restauration de la liaison, le service `exportcache()` relit `/togate.cache` et renvoi automatiquement les commandes stockées au serveur.
- L'ordre de renvoi respecte l'ordre d'enregistrement (FIFO).

```
/togate.cache  →  Format : une commande par ligne
                  "<id>:<timestamp>:<commande_complète>"
```

### 9.3 Détection de l'état de la liaison

| Condition | Action Gate |
|-----------|-------------|
| ACK reçu | `isgateonline = true` ; suppression de la file |
| 5 échecs consécutifs | `isgateonline = false` ; stockage SD activé |
| Reconnexion détectée | Vidage du cache SD vers le serveur |

---

## 10. Interfaces web

Toutes les interfaces web sont **hébergées sur Apache2** et servies en HTTPS.

### 10.1 Terminal web

| Propriété | Valeur |
|-----------|--------|
| URL | `/terminal` (ou sous-chemin configurable) |
| Technologies | HTML, JavaScript, WebSocket |
| Rôle | Relay I/O de la Gate (équivalent port série USB) |
| Architecture | Modulaire — intégrable dans l'outil de gestion MycroMesh |

**Fonctionnalités :**
- Connexion en temps réel à la Gate via le serveur Python (WebSocket HTTP).
- Affichage de la sortie de la Gate.
- Saisie de commandes envoyées à la Gate.
- Indicateur d'état de la liaison (connecté / déconnecté).
- Historique de session affiché dans la page.

### 10.2 Page transfert de fichiers

| Propriété | Valeur |
|-----------|--------|
| URL | `/files` (ou sous-chemin configurable) |
| Technologies | HTML, JavaScript, PHP |
| Rôle | Upload / download de fichiers entre navigateur et Gate |

**Fonctionnalités :**
- Upload d'un fichier vers la Gate (avec sélection de la station cible ou broadcast).
- Téléchargement d'un fichier reçu depuis la Gate.
- Indicateur de progression du transfert.
- Liste des fichiers en attente / transférés.

---

## 11. Sécurité

### 11.1 Exigences

- **Toutes les liaisons** Gate ↔ Serveur doivent être **chiffrées** (TLS/SSL).
- **Toutes les interfaces web** exposées au réseau doivent être servies en **HTTPS**.
- L'accès au terminal web et à la page de transfert de fichiers est **authentifié** (session, token, ou équivalent).
- La surface d'attaque exposée est **minimisée** : seuls les ports strictement nécessaires sont ouverts.

### 11.2 Port dédié

Le port utilisé par le service Python pour la liaison Gate ↔ Serveur doit être :
- Non conflictuel avec les services existants (voir section 4.1).
- Documenté dans la configuration du serveur.
- Filtré par le pare-feu pour n'accepter que les connexions depuis l'adresse IP de la Gate.

### 11.3 Authentification Gate

La Gate doit s'authentifier auprès du serveur à chaque connexion (ex. : token partagé, certificat client TLS).

---

## 12. Stack technique

| Composant | Technologie | Notes |
|-----------|-------------|-------|
| Gate | ESP32 (Arduino/IDF) | Wi-Fi natif + Ethernet LAN8720 |
| Service serveur | Python 3 | Service systemd sur Ubuntu Server |
| Web server | Apache2 + PHP | Stack existante |
| Base de données | MySQL | Tables par version de schéma |
| Terminal web | HTML + JS | WebSocket client |
| Stockage gate | Carte SD (SPI) | Buffer offline `/togate.cache` |
| Communication | WebSocket / TCP custom | Port dédié (ex. 8765) |
| Chiffrement | TLS 1.2+ | Certificat Let's Encrypt ou CA interne |

---

## 13. Schéma de base de données

### 13.1 Table de log des commandes

```sql
CREATE TABLE cmd_log (
    id           BIGINT AUTO_INCREMENT PRIMARY KEY,
    received_at  DATETIME NOT NULL,
    tx_station   VARCHAR(16) NOT NULL,
    raw_command  TEXT NOT NULL,
    processed    TINYINT(1) DEFAULT 0
);
```

### 13.2 Tables de données par version

**Version 0 — Statistiques station**

```sql
CREATE TABLE data_v0 (
    id          BIGINT AUTO_INCREMENT PRIMARY KEY,
    tx_station  VARCHAR(16) NOT NULL,
    received_at DATETIME NOT NULL,
    nbatt       VARCHAR(64),
    tstartstat  VARCHAR(64),
    tmaintmode  VARCHAR(64),
    trxglob     VARCHAR(64),
    trxbrd      VARCHAR(64),
    trxloc      VARCHAR(64),
    tdijktx     VARCHAR(64),
    tdijkrx     VARCHAR(64),
    tdijkhop    VARCHAR(64),
    tresend1    VARCHAR(64),
    tresend2    VARCHAR(64),
    ttxcnt      VARCHAR(64),
    ntime       BIGINT
);
```

**Version 1 — Mesures environnementales complètes**

```sql
CREATE TABLE data_v1 (
    id          BIGINT AUTO_INCREMENT PRIMARY KEY,
    tx_station  VARCHAR(16) NOT NULL,
    received_at DATETIME NOT NULL,
    ntemp       FLOAT,
    nhum        FLOAT,
    npres       FLOAT,
    nres        FLOAT,
    ntime       BIGINT
);
```

**Version 2 — Test / débogage**

```sql
CREATE TABLE data_v2 (
    id          BIGINT AUTO_INCREMENT PRIMARY KEY,
    tx_station  VARCHAR(16) NOT NULL,
    received_at DATETIME NOT NULL,
    test        TEXT,
    tsp         TEXT
);
```

**Version 3 — Mesures environnementales sans résistance**

```sql
CREATE TABLE data_v3 (
    id          BIGINT AUTO_INCREMENT PRIMARY KEY,
    tx_station  VARCHAR(16) NOT NULL,
    received_at DATETIME NOT NULL,
    ntemp       FLOAT,
    nhum        FLOAT,
    npres       FLOAT,
    ntime       BIGINT
);
```

**Version 4 — Mesure minimale**

```sql
CREATE TABLE data_v4 (
    id          BIGINT AUTO_INCREMENT PRIMARY KEY,
    tx_station  VARCHAR(16) NOT NULL,
    received_at DATETIME NOT NULL,
    ntemp       FLOAT,
    ntime       BIGINT
);
```

### 13.3 Table de suivi des fichiers transférés

```sql
CREATE TABLE file_transfers (
    id            BIGINT AUTO_INCREMENT PRIMARY KEY,
    started_at    DATETIME NOT NULL,
    completed_at  DATETIME,
    direction     ENUM('gate_to_server', 'server_to_gate') NOT NULL,
    filename      VARCHAR(255) NOT NULL,
    target_station VARCHAR(16),   -- NULL = broadcast
    status        ENUM('pending', 'in_progress', 'done', 'error') NOT NULL DEFAULT 'pending',
    size_bytes    INT
);
```

---

## 14. Phases d'implémentation

### Phase 1 — Réseau de la Gate

**Objectif :** Initialisation et gestion fiable des deux interfaces réseau.

- [ ] Initialisation simultanée Wi-Fi ESP32 + Ethernet LAN8720.
- [ ] Test de connectivité sur chaque interface.
- [ ] Implémentation du basculement automatique (Ethernet prioritaire).
- [ ] Test de connectivité simultanée et de stabilité.
- [ ] Reconnexion automatique en cas de perte de liaison.

### Phase 2 — Service Python et liaison Gate ↔ Serveur

**Objectif :** Établissement des deux canaux de communication.

- [ ] Création du service Python (systemd) sur Ubuntu Server.
- [ ] Implémentation du serveur WebSocket (ou TCP) sur port dédié.
- [ ] Multiplexage des deux canaux logiques (Terminal + Commandes) sur la connexion.
- [ ] Implémentation de l'ACK/NACK pour les commandes `data:`.
- [ ] Implémentation du mécanisme de stockage SD en cas de perte de liaison.
- [ ] Implémentation de `exportcache()` pour le renvoi automatique après reconnexion.
- [ ] Log de toutes les commandes reçues dans `cmd_log`.

### Phase 3 — Terminal web

**Objectif :** Interface web reproduisant un port série USB vers la Gate.

- [ ] Développement de la page HTML/JS du terminal.
- [ ] Connexion WebSocket entre navigateur et serveur Python (via proxy Apache2).
- [ ] Relay bidirectionnel I/O Gate ↔ navigateur.
- [ ] Indicateur d'état de la liaison.
- [ ] Architecture modulaire pour intégration dans l'outil de gestion MycroMesh.

### Phase 4 — Traitement des données et injection MySQL

**Objectif :** Parseur `data.ver` et injection automatique en base.

- [ ] Implémentation du parseur de commandes `data:` côté serveur.
- [ ] Lecture dynamique du fichier `data.ver` pour résolution des versions.
- [ ] Création des tables MySQL pour chaque version (V0 à V4).
- [ ] Injection automatique des données dans la table correspondante.
- [ ] Gestion des erreurs d'insertion et retransmission du NACK.

### Phase 5 — Gestion des fichiers

**Objectif :** Transferts bidirectionnels et diffusion vers le réseau MycroMesh.

- [ ] Implémentation du transfert `rx.txt` Gate → Serveur.
- [ ] Implémentation du parsing et de la compilation côté serveur.
- [ ] Renvoi du `tx.txt` / fichier compilé au Gate.
- [ ] Implémentation du broadcast fichier vers le réseau MycroMesh via la Gate.
- [ ] Implémentation de l'envoi ciblé vers une station spécifique via la Gate.
- [ ] Développement de la page web de transfert de fichiers (Apache2/PHP).

### Phase 6 — Sécurité

**Objectif :** Chiffrement et authentification de l'ensemble des liaisons.

- [ ] Mise en place du TLS sur la liaison Gate ↔ Serveur (port dédié).
- [ ] Configuration HTTPS pour toutes les interfaces web (Apache2).
- [ ] Authentification de la Gate auprès du serveur (token / certificat client).
- [ ] Authentification des utilisateurs web (terminal + transfert de fichiers).
- [ ] Configuration du pare-feu (restriction d'accès au port dédié).
- [ ] Audit de la surface d'attaque.

---

## 15. Contraintes et limites

| Contrainte | Détail |
|------------|--------|
| Pas de MQTT | Protocole explicitement exclu pour la liaison Gate ↔ Serveur |
| Pas de conflit de ports | Port dédié à choisir hors de la liste de la section 4.1 |
| Charge de calcul sur le serveur | La Gate ne compile ni ne parse : tout est délégué au serveur |
| Stack serveur existante | Apache2 + PHP + MySQL déjà en place — ne pas casser l'existant |
| Taille max payload LoRa | ~225 octets par paquet (contrainte réseau mesh amont) |
| Buffer SD gate | Capacité limitée par la carte SD embarquée |
| Débit Wi-Fi / Ethernet | Le transit de fichiers volumineux doit gérer le débit disponible |

---

## Références

- Projet MycroMesh : [https://github.com/muldeving/Mycromesh](https://github.com/muldeving/Mycromesh)
- Schéma fonctionnel firmware nœuds : `SCHEMA_FONCTIONNEL.md`
- Document source : `mycromesh-gateway-server.md`
