// =============================================================================
// Mycromesh - Variant GATE
// Extension de core avec des fonctionnalités gateway et des comportements
// différents sur certaines fonctions.
// Les fonctions communes sont dans src/mycromesh.h.
//
// Pour activer du code GATE-spécifique dans le core partagé, utiliser :
//   #ifdef VARIANT_GATE
//     // code spécifique gate
//   #endif
//
// Pour remplacer une fonction du core dans ce variant :
//   - Renommer la fonction dans src/mycromesh.h avec un suffixe _base
//     (ex: handleMessage_base) sous guard #ifndef VARIANT_GATE
//   - Définir la version GATE ici
// =============================================================================

#define VARIANT_GATE

#include "src/mycromesh.h"

// ---- Ajouts et surcharges GATE (à venir) ------------------------------------
// Les fonctions spécifiques au variant GATE seront ajoutées ici.
