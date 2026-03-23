#!/usr/bin/env python3
"""
test_gate.py — Tests fonctionnels de la gate Mycromesh via liaison série
Réseau minimal : gate (localAddress configurable) + 1 autre station.

Usage:
    python3 test_gate.py --port /dev/ttyUSB0 --baud 115200 --node 7
    python3 test_gate.py --port COM3 --node 7 --skip-network

Options:
    --port          Port série de la gate (défaut : /dev/ttyUSB0)
    --baud          Vitesse (défaut : 115200)
    --node          Adresse de l'autre station sur le réseau (défaut : 7)
    --timeout       Timeout attente réponse en secondes (défaut : 15)
    --skip-network  Passe les tests qui nécessitent une réponse de l'autre station
    --verbose       Affiche toutes les lignes reçues (même hors-attendu)

Notes:
    - Le script force slvl:debug au démarrage (nécessaire pour intercepter
      ping:2/ping:3, acth, read, write qui ne sortent qu'au niveau 3).
    - slvl est restauré à 1 (normal) à la fin des tests.
"""

import serial
import time
import argparse
import sys
import re

# ─── Couleurs terminal ────────────────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

# ─── Résultats globaux ────────────────────────────────────────────────────────
results = []


class Gate:
    def __init__(self, port, baud, timeout, verbose):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.timeout = timeout
        self.verbose = verbose
        time.sleep(2)       # attente reset ESP32 à l'ouverture du port
        self.flush()

    def flush(self):
        self.ser.reset_input_buffer()

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())
        if self.verbose:
            print(f"  {CYAN}>>> {cmd}{RESET}")

    def read_lines(self, duration):
        """Lit toutes les lignes reçues pendant `duration` secondes."""
        lines = []
        deadline = time.time() + duration
        while time.time() < deadline:
            raw = self.ser.readline()
            if raw:
                line = raw.decode(errors="replace").strip()
                if line:
                    if self.verbose:
                        print(f"  {YELLOW}<<< {line}{RESET}")
                    lines.append(line)
        return lines

    def wait_for(self, patterns, extra=1):
        """
        Attend jusqu'à ce que tous les patterns soient trouvés ou timeout.
        Puis lit `extra` secondes supplémentaires pour vider le buffer.
        Retourne (ok, toutes_les_lignes_reçues).
        """
        if isinstance(patterns, (str, re.Pattern)):
            patterns = [patterns]
        remaining = list(patterns)
        lines = []
        deadline = time.time() + self.timeout
        while time.time() < deadline and remaining:
            raw = self.ser.readline()
            if raw:
                line = raw.decode(errors="replace").strip()
                if line:
                    if self.verbose:
                        print(f"  {YELLOW}<<< {line}{RESET}")
                    lines.append(line)
                    remaining = [p for p in remaining
                                 if not re.search(p, line, re.IGNORECASE)]
        lines += self.read_lines(extra)
        return (len(remaining) == 0), lines

    def close(self):
        self.ser.close()


# ─── Infrastructure de test ───────────────────────────────────────────────────

def section(title):
    print(f"\n{BOLD}{CYAN}{'─'*60}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'─'*60}{RESET}")


def run(gate, name, cmd, patterns, extra=1, skip=False):
    """Envoie `cmd`, attend `patterns`. extra = secondes de lecture après match."""
    if skip:
        print(f"  {YELLOW}SKIP{RESET}  {name}")
        results.append(("SKIP", name))
        return None, []

    gate.flush()
    if cmd:
        gate.send(cmd)
    ok, lines = gate.wait_for(patterns, extra=extra)
    status = f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"
    print(f"  {status}  {name}")
    if not ok and not gate.verbose:
        for l in lines[-5:]:
            print(f"         {YELLOW}{l}{RESET}")
    results.append(("PASS" if ok else "FAIL", name))
    return ok, lines


def run_seq(gate, name, steps, skip=False):
    """
    Test séquentiel : steps = [(cmd_ou_None, pattern, extra), ...]
    Chaque étape attend son pattern avant de passer à la suivante.
    Un flush() est fait avant chaque envoi, PAS entre les attentes
    (pour ne pas perdre les lignes en transit).
    """
    if skip:
        print(f"  {YELLOW}SKIP{RESET}  {name}")
        results.append(("SKIP", name))
        return

    all_ok = True
    for cmd, pattern, extra in steps:
        if cmd:
            gate.flush()
            gate.send(cmd)
        ok, lines = gate.wait_for(pattern, extra=extra)
        if not ok:
            all_ok = False
            if not gate.verbose:
                for l in lines[-5:]:
                    print(f"         {YELLOW}{l}{RESET}")
            break

    status = f"{GREEN}PASS{RESET}" if all_ok else f"{RED}FAIL{RESET}"
    print(f"  {status}  {name}")
    results.append(("PASS" if all_ok else "FAIL", name))


# ─── Suites de tests ──────────────────────────────────────────────────────────

def test_systeme(g):
    section("SYSTÈME")
    run(g, "Version firmware",     "version", r"\[SYSTEME\].*1\.")
    run(g, "Heure RTC",            "time",    r"\d{4}.*\d{2}:\d{2}:\d{2}|epoch|\d{10}")
    run(g, "Statistiques trafic",  "gmea",    r"trxglob:")


def test_config(g):
    section("CONFIGURATION")

    # Lecture config — on capture la ligne pour setcfg round-trip
    ok, lines = run(g, "getcfg — lecture", "getcfg", r"^CFG:")
    cfg_line = next((l for l in lines if l.startswith("CFG:")), None)

    # Niveaux de log
    run(g, "slvl:verbose",   "slvl:verbose",  r"\[CONFIG\].*verbose")
    run(g, "slvl:normal",    "slvl:normal",   r"\[CONFIG\].*normal")
    run(g, "slvl:debug",     "slvl:debug",    r"\[CONFIG\].*debug")
    run(g, "slvl numérique", "slvl:1",        r"\[CONFIG\]")

    # Mode I/O
    run(g, "iomd:usb", "iomd:usb", r"\[IO\].*USB")

    # Paramètre individuel
    run(g, "parm:serialLevel", "parm:serialLevel:1", r"\[CONFIG\]")

    # setcfg round-trip avec la config lue
    if cfg_line:
        fields = cfg_line[4:].split(":")   # retire le "CFG:" initial
        if len(fields) >= 13:
            run(g, "setcfg — round-trip",
                "setcfg:" + ":".join(fields[:13]) + ":",
                r"CFG:OK")
        else:
            run(g, "setcfg — format ERR attendu",
                "setcfg:1:2:3",
                r"CFG:ERR")
    else:
        run(g, "setcfg — format ERR attendu",
            "setcfg:1:2:3",
            r"CFG:ERR")

    # Rétablit slvl:debug pour les sections suivantes
    # (slvl:1 ci-dessus l'a abaissé ; ping:2/3, acth etc. ne sortent qu'au niveau 3)
    g.send("slvl:debug")
    g.read_lines(1)


def test_sd(g):
    section("CARTE SD")

    # ls racine (toujours peuplée)
    run(g, "ls racine", "ls:/", r"\S")

    # Séquence : mkdir → cp → ls (contenu) → mkdir2 → mv → rm → rmdir × 2
    # cp et mv prennent un répertoire DESTINATION, le nom de fichier est conservé
    run_seq(g, "mkdir → cp → ls → mv → rm → rmdir", [
        ("mkdir:/test_mycro",              r"\[SD\]",           1),
        ("cp:/p.cfg:/test_mycro",          r"\[SD\].*succes",   1),
        ("ls:/test_mycro",                 r"p\.cfg|\S",        1),
        ("mkdir:/test_mycro2",             r"\[SD\]",           1),
        ("mv:/test_mycro/p.cfg:/test_mycro2", r"\[SD\].*succes", 1),
        ("rm:/test_mycro2/p.cfg",          r"\[SD\].*supprime", 1),
        ("rmdir:/test_mycro",              r"\[SD\]",           1),
        ("rmdir:/test_mycro2",             r"\[SD\]",           1),
    ])


def test_gateway(g, node, skip_network):
    section("PASSERELLE (GATEWAY)")
    run(g, "gate — adresse passerelle",  "gate",        r"\[GATEWAY\]")
    run(g, "cachestate — état cache",    "cachestate",  r"\[GATEWAY\]")
    run(g, "filestate — état transfert", "filestate",   r"\[FICHIER")


def test_decouverte(g, node, skip_network):
    section("DÉCOUVERTE RÉSEAU")

    # pigo : vague 1 immédiate (logN), puis vague 2 à +4s (logD), vague 3 à +8s (logD),
    # fin à +12s (logN). On intercepte chaque vague séparément sans flush entre elles
    # pour ne pas perdre les lignes en transit.
    run_seq(g, "pigo — 3 vagues + fin", [
        ("pigo",  r"\[PING\].*sondes",  1),   # t=0 : vague 1 logN
        (None,    r"ping:2",            1),   # t≈4s : vague 2 logD
        (None,    r"ping:3",            1),   # t≈8s : vague 3 logD
        (None,    r"\[PING\].*termin",  2),   # t≈12s : fin logN
    ])

    run(g, "prnt — table de routage", "prnt", r"edges:")
    run(g, "expv — export topologie", "expv", r"tmap:")

    run(g, "clear — efface la table", "clear", r"\[OK\].*efface",
        skip=skip_network)

    # Re-découverte après clear (pigo complet, ~15s)
    if not skip_network:
        run_seq(g, "pigo après clear — re-découverte", [
            ("pigo",  r"\[PING\].*sondes",  1),
            (None,    r"ping:2",            1),
            (None,    r"ping:3",            1),
            (None,    r"\[PING\].*termin",  2),
        ])


def test_reseau(g, node, skip_network):
    section("COMMUNICATION RÉSEAU (nécessite l'autre station)")

    # trsp : envoi fiable → accusé [MESSAGE] en retour
    run(g, f"trsp → nœud {node} (attend accusé)",
        f"trsp:{node}:ping",
        r"\[MESSAGE\].*livre|\[MESSAGE\].*accuse|arok:",
        extra=10,
        skip=skip_network)

    # send générique → même accusé de réception
    run(g, f"send → nœud {node}",
        f"send:{node}:data:test_serial",
        r"\[MESSAGE\].*livre|\[MESSAGE\].*accuse|arok:",
        extra=8,
        skip=skip_network)

    # acth : envoie geth au voisin le plus proche, reçoit seth → logD
    run(g, "acth — synchro horloge",
        "acth",
        r"geth|seth|\[OK\].*heure|\[OK\].*synchro|horloge|rtc",
        extra=10,
        skip=skip_network)

    # netio : tunnel maître vers l'autre station
    # Étape 1 : demande d'ouverture (logN immédiat)
    # Étape 2 : confirmation ntiok de l'esclave (aller-retour réseau, peut prendre ~10s)
    # Étape 3 : envoi de "version" dans le tunnel → réponse ntirsp de l'esclave
    # Étape 4 : "exit" → fermeture propre
    run_seq(g, f"netio → nœud {node} (ouvre tunnel, version, ferme)",
        [
            (f"netio:{node}", r"\[NETIO\].*ouverture|ouvert|tunnel",  2),
            (None,            r"\[NETIO\].*tunnel ouvert|tapez",      20),
            ("version",       r"1\.\d+|\[SYSTEME\]",                  8),
            ("exit",          r"\[NETIO\].*ferme",                    3),
        ],
        skip=skip_network)


def test_transfert_fichier(g, node, skip_network):
    section("TRANSFERT DE FICHIER (nécessite l'autre station)")

    # stft : lance le transfert de /p.cfg (petit fichier toujours présent)
    # → l'autre station répond isrf, gate répond rfok, puis envoie les chunks
    run_seq(g, f"stft — transfert /p.cfg → nœud {node}",
        [
            (f"stft:{node}:/p.cfg:0", r"\[FICHIER|isrf|rfok|transfert|\[ERREUR\]", 20),
        ],
        skip=skip_network)


def test_planification(g):
    section("PLANIFICATION (adrc)")

    # adrc planifie "version" dans 2s. Aucune sortie immédiate ;
    # on attend directement la sortie différée.
    run(g, "adrc — version différée de 2s",
        "adrc:2000:version",
        r"\[SYSTEME\].*1\.",
        extra=1)


def test_divers(g):
    section("COMMANDES DIVERSES")

    run(g, "data — message texte",  "data:hello_test", r"hello_test")
    run(g, "read — rechargement SD", "read",           r"\[OK\].*recharg|recharg|carte.*reseau")
    run(g, "write — sauvegarde SD",  "write",          r"\[OK\].*sauvegard|sauvegard|configuration.*sauvegard")


# ─── Résumé ───────────────────────────────────────────────────────────────────

def print_summary():
    print(f"\n{BOLD}{'═'*60}{RESET}")
    print(f"{BOLD}  RÉSUMÉ{RESET}")
    print(f"{BOLD}{'═'*60}{RESET}")
    passed  = sum(1 for s, _ in results if s == "PASS")
    failed  = sum(1 for s, _ in results if s == "FAIL")
    skipped = sum(1 for s, _ in results if s == "SKIP")

    for status, name in results:
        if status == "PASS":
            print(f"  {GREEN}✓{RESET}  {name}")
        elif status == "FAIL":
            print(f"  {RED}✗{RESET}  {name}")
        else:
            print(f"  {YELLOW}−{RESET}  {name}")

    print(f"\n  Total : {len(results)}  |  "
          f"{GREEN}PASS : {passed}{RESET}  |  "
          f"{RED}FAIL : {failed}{RESET}  |  "
          f"{YELLOW}SKIP : {skipped}{RESET}")
    print(f"{BOLD}{'═'*60}{RESET}\n")
    return failed == 0


# ─── Point d'entrée ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Tests série gate Mycromesh")
    parser.add_argument("--port",         default="/dev/ttyUSB0")
    parser.add_argument("--baud",         type=int, default=115200)
    parser.add_argument("--node",         type=int, default=7,
                        help="Adresse de l'autre station (défaut : 7)")
    parser.add_argument("--timeout",      type=int, default=15,
                        help="Timeout attente réponse en secondes (défaut : 15)")
    parser.add_argument("--skip-network", action="store_true",
                        help="Ignore les tests nécessitant la station distante")
    parser.add_argument("--verbose",      action="store_true",
                        help="Affiche toutes les lignes série échangées")
    args = parser.parse_args()

    print(f"\n{BOLD}Mycromesh — Tests série gate{RESET}")
    print(f"  Port    : {args.port}  @{args.baud} baud")
    print(f"  Nœud    : gate ↔ station #{args.node}")
    print(f"  Timeout : {args.timeout}s")
    if args.skip_network:
        print(f"  {YELLOW}Mode hors-réseau actif (tests réseau ignorés){RESET}")

    try:
        g = Gate(args.port, args.baud, args.timeout, args.verbose)
    except serial.SerialException as e:
        print(f"\n{RED}Impossible d'ouvrir {args.port} : {e}{RESET}")
        sys.exit(1)

    try:
        # Force slvl:debug pour toute la session de test :
        # ping:2, ping:3, acth, read, write ne sortent qu'au niveau 3.
        g.send("slvl:debug")
        g.read_lines(1)

        test_systeme(g)
        test_config(g)
        test_sd(g)
        test_gateway(g, args.node, args.skip_network)
        test_decouverte(g, args.node, args.skip_network)
        test_reseau(g, args.node, args.skip_network)
        test_transfert_fichier(g, args.node, args.skip_network)
        test_planification(g)
        test_divers(g)

    except KeyboardInterrupt:
        print(f"\n{YELLOW}Interrompu.{RESET}")
    finally:
        # Restaure le niveau de log normal
        g.send("slvl:1")
        time.sleep(0.5)
        g.close()

    ok = print_summary()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
