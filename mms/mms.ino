// =============================================================================
// Mycromesh - Variant MMS (MMS Core)
// Extension de core avec des fonctionnalités spécifiques MMS.
// Les fonctions communes sont dans src/mycromesh.h.
//
// — Surcharge du pinout —
// Décommenter et ajuster les #define ci-dessous pour changer le câblage MMS.
// Ces #define DOIVENT être avant le #include "src/mycromesh.h".
//
//   #define PIN_LORA_CS   21
//   #define PIN_LORA_IRQ   8
//   #define PIN_ONEWIRE    3
//   #define PIN_SD_CS      7
//   #define PIN_WAKEUP     1
//
// — Code variant-spécifique dans le core partagé —
// Utiliser #ifdef VARIANT_MMS dans src/mycromesh.h pour isoler du code MMS.
// =============================================================================

#define VARIANT_MMS

#include "src/mycromesh.h"

// =============================================================================
// setup() MMS — appelé au démarrage
// Appelle setup_base() pour le comportement commun, puis ajoute les
// initialisations spécifiques MMS.
// =============================================================================
void setup() {
  setup_base();

  // ---- Initialisations MMS (à venir) ----------------------------------------
}

// =============================================================================
// loop() MMS — boucle principale
// Appelle loop_base() pour le comportement commun, puis ajoute les
// opérations périodiques spécifiques MMS.
// =============================================================================
void loop() {
  loop_base();

  // ---- Opérations MMS (à venir) ---------------------------------------------
}

// ---- Fonctions MMS supplémentaires (à venir) --------------------------------
