# Projet MycroMesh — Gateway & Serveur

> Récapitulatif technique du système de passerelle ESP32 et de son serveur de gestion associé,
> dans le cadre du projet [MycroMesh](https://github.com/muldeving/Mycromesh).

---

## Vue d'ensemble

Le système se compose de deux éléments complémentaires :

- **La Gate** — une passerelle matérielle ESP32 avec double accès réseau (Wi-Fi natif + Ethernet LAN8720), connectée au réseau MycroMesh d'un côté, et au serveur de l'autre.
- **Le Serveur** — un service Python tournant sur Ubuntu Server (stack Apache2 + PHP + MySQL existante), qui gère la gate, traite les données reçues, effectue les compilations, et expose un terminal web.

La communication Gate ↔ Serveur repose sur **deux canaux distincts** :

| Canal | Rôle |
|---|---|
| **Terminal** | Remige d'entrée/sortie de la gate (équivalent port série USB) |
| **Commandes** | Transfert des commandes exécutées par l'interpréteur de la gate |

---

## Architecture réseau de la Gate

La gate utilise deux interfaces réseau simultanées :

```cpp
// Interface Ethernet (LAN8720)
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
#define ETH_POWER       12
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        1
#define ETH_MDC         23
#define ETH_MDIO        18

// Interface Wi-Fi (intégrée ESP32)
```

---

## Fonctionnalités clés

### Terminal web
- Le serveur expose une **page web hébergée sur Apache** qui relie l'entrée/sortie de la gate, comme si elle était connectée par port série USB.
- L'interface est **modulaire** : elle sera intégrée ultérieurement dans un outil de gestion global du réseau MycroMesh.

### Canal commandes
- Toutes les commandes exécutées par l'interpréteur de la gate sont **transmises au serveur**.
- Le serveur peut **traiter certaines commandes** et les **logue toutes** sans exception.
- Les commandes préfixées `data:` ne sont **pas interprétées par la gate** : elles sont transmises au serveur pour traitement.

### Fiabilité des données (`data:`)
- La gate **garantit la réception** de chaque commande `data:` par le serveur (accusé de réception).
- En cas de liaison perdue, les commandes `data:` sont **stockées sur la carte SD** de la gate.
- Elles sont renvoyées automatiquement au serveur dès la **restauration de la liaison**.

### Traitement des données (`data:`) côté serveur

#### Format de la commande `data:`

Chaque commande `data:` émise par une station MycroMesh suit la structure :

```
TXstation:version:data
```

- **`TXstation`** — identifiant de la station émettrice.
- **`version`** — numéro de version du schéma de données (référence `data.ver`).
- **`data`** — valeurs des champs, dans l'ordre défini par la version.

**Convention de typage des champs** (préfixe du nom de champ) :

| Préfixe | Type |
|---|---|
| `n` | Valeur numérique |
| `t` | Valeur texte |

#### Fichier `data.ver` — versions de schéma définies

Le fichier `data.ver` est la référence partagée entre les stations du réseau et le serveur. Il définit, pour chaque version, la liste ordonnée des champs attendus.

| Version | Champs | Usage |
|---|---|---|
| `0` | `nbatt`, `tstartstat`, `tmaintmode`, `trxglob`, `trxbrd`, `trxloc`, `tdijktx`, `tdijkrx`, `tdijkhop`, `tresend1`, `tresend2`, `ttxcnt`, `ntime` | Statistiques de fonctionnement station (batterie, modes, compteurs réseau, horodatage) |
| `1` | `ntemp`, `nhum`, `npres`, `nres`, `ntime` | Mesures environnementales complètes (température, humidité, pression, résistance, horodatage) |
| `2` | `test`, `tsp` | Données de test / débogage |
| `3` | `ntemp`, `nhum`, `npres`, `ntime` | Mesures environnementales sans résistance |
| `4` | `ntemp`, `ntime` | Mesure minimale (température + horodatage) |

#### Traitement serveur

- Le serveur lit le fichier `data.ver` pour identifier la structure de la commande reçue selon son numéro de version.
- Il mappe les champs reçus sur les colonnes de la **table MySQL correspondante à la version**.
- Chaque version possède sa propre table dédiée, ce qui permet l'évolution indépendante des schémas.

### Transfert de fichiers
- Transferts bidirectionnels entre la gate et le serveur.
- Le serveur prend en charge les **compilations** et le **parsing** des fichiers pour la gate :
  - Il reçoit le `rx.txt` de la gate, traite le fichier, et lui renvoie le `tx.txt` ou le fichier compilé.
  - Objectif : **décharger la gate** de toute charge de calcul.
- Le serveur permet nativement, via un simple appel, de :
  - **Diffuser un fichier** vers tout le réseau MycroMesh (broadcast).
  - **Envoyer un fichier** à une station spécifique du réseau.
  - Dans les deux cas, le transit s'effectue via la gate.
- Une **page web dédiée** permet d'envoyer et de réceptionner des fichiers.

### Sécurité
- L'ensemble des liaisons Gate ↔ Serveur et des interfaces web exposées au réseau devront être **chiffrées et sécurisées**.
- Objectif : limiter la surface d'attaque et protéger contre les accès malveillants.

---

## Protocole de liaison Gate ↔ Serveur

**MQTT est exclu** comme protocole de liaison entre la gate et le serveur.

La solution retenue devra fonctionner **sans conflit de ports ni de ressources** avec les services déjà en place sur le serveur :

| Service existant | Protocole / Port typique |
|---|---|
| Apache2 + PHP | HTTP 80 / HTTPS 443 |
| FTP | 20, 21 |
| SSH | 22 |
| VPN | 1194 (OpenVPN) ou 51820 (WireGuard) |
| Grafana | 3000 |
| Home Assistant | 8123 |

La liaison Gate ↔ Serveur devra donc utiliser un **port dédié non conflictuel**, avec une communication persistante et bidirectionnelle (canal terminal + canal commandes). Une solution de type **WebSocket** (ex. sur port 8765 ou similaire) ou **TCP socket custom** géré par le service Python est à privilégier, en s'assurant que le port choisi est libre et documenté dans la configuration du serveur.

---

## Stack technique

| Composant | Technologie |
|---|---|
| Gate | ESP32 (Wi-Fi + Ethernet LAN8720) |
| Serveur | Python (Ubuntu Server) |
| Web server | Apache2 + PHP |
| Base de données | MySQL |
| Terminal web | Page HTML/JS hébergée sur Apache |
| Stockage gate | Carte SD (buffer offline) |

---

## Phases d'implémentation

### Phase 1 — Réseau de la gate
- Initialisation et gestion des deux interfaces réseau (Wi-Fi ESP32 + Ethernet LAN8720).
- Tests de basculement et de connectivité simultanée.

### Phase 2 — Serveur et liaison Gate ↔ Serveur
- Création du service Python serveur.
- Établissement des deux canaux de communication (terminal + commandes) via Ethernet et Wi-Fi.
- Mise en place de l'accusé de réception des commandes `data:` et du mécanisme de stockage SD en cas de perte de liaison.

### Phase 3 — Terminal web
- Développement de la page web de terminal sur Apache.
- Liaison au serveur pour le remige des entrées/sorties de la gate.
- Architecture modulaire pour intégration future dans l'outil de gestion MycroMesh.

### Phase 4 — Traitement des données et injection MySQL
- Implémentation du parseur `data.ver` côté serveur.
- Injection automatique des données reçues dans les tables MySQL correspondantes.

### Phase 5 — Gestion des fichiers
- Transferts bidirectionnels Gate ↔ Serveur (compilation, parsing, `rx.txt` / `tx.txt`).
- Implémentation de la diffusion et de l'envoi ciblé vers le réseau MycroMesh via la gate.
- Page web dédiée à l'envoi et à la réception de fichiers.

---

## Références

- Projet MycroMesh : [https://github.com/muldeving/Mycromesh](https://github.com/muldeving/Mycromesh)
- Projet iOS-Keyboard (inspiration architecture BLE/ESP32) : voir documentation interne
