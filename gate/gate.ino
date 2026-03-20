// =============================================================================
// Mycromesh - Variant GATE
// Extension de core avec des fonctionnalités gateway et des comportements
// différents sur certaines fonctions.
// Les fonctions communes sont dans src/mycromesh.h.
//
// — Surcharge du pinout —
// Décommenter et ajuster les #define ci-dessous pour changer le câblage GATE.
// Ces #define DOIVENT être avant le #include "src/mycromesh.h".
//
//   #define PIN_LORA_CS   21
//   #define PIN_LORA_IRQ   8
//   #define PIN_ONEWIRE    3
//   #define PIN_SD_CS      7
//   #define PIN_WAKEUP     1
//
// — Surcharge de fonctions du core —
// Pour remplacer une fonction de src/mycromesh.h dans ce variant :
//   1. Dans src/mycromesh.h, entourer la version core avec :
//        #ifndef VARIANT_GATE
//          void maFonction() { ... }  // version core
//        #endif
//   2. Définir la version GATE ici, après le #include.
//
// — Code variant-spécifique dans le core partagé —
// Utiliser #ifdef VARIANT_GATE dans src/mycromesh.h pour isoler du code GATE.
// =============================================================================

#define VARIANT_GATE

#include "src/mycromesh.h"

// =============================================================================
// setup() GATE — appelé au démarrage
// Appelle setup_base() pour le comportement commun, puis ajoute les
// initialisations spécifiques GATE.
// =============================================================================
void setup() {
  setup_base();

  // ---- Initialisations GATE (à venir) ----------------------------------------
}

// =============================================================================
// loop() GATE — boucle principale
// Appelle loop_base() pour le comportement commun, puis ajoute ou remplace
// les opérations périodiques spécifiques GATE.
// =============================================================================
void loop() {
  loop_base();

  // ---- Opérations GATE (à venir) ---------------------------------------------
}

// ---- Fonctions GATE supplémentaires / surcharges (à venir) ------------------
