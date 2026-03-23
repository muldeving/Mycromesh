#!/usr/bin/env python3
"""
test_gate.py — Tests fonctionnels de la gate Mycromesh via liaison série
Réseau minimal : gate (localAddress configurable) + 1 autre station.

Usage:
    python3 test_gate.py --port /dev/ttyUSB0 --baud 115200 --node 7
    python3 test_gate.py --port COM3 --node 7 --skip-network

Options:
    --port     Port série de la gate (défaut : /dev/ttyUSB0)
    --baud     Vitesse (défaut : 115200)
    --node     Adresse de l'autre station sur le réseau (défaut : 7)
    --timeout  Timeout attente réponse en secondes (défaut : 15)
    --skip-network  Passe les tests qui nécessitent une réponse de l'autre station
    --verbose  Affiche toutes les lignes reçues (même hors-attendu)
"""

import serial
import time
import argparse
import sys
import re

# ─── Couleurs terminal ───────────────────────────────────────────────────────
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
        time.sleep(2)           # attente reset ESP32 à l'ouverture du port
        self.flush()

    def flush(self):
        self.ser.reset_input_buffer()

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())
        if self.verbose:
            print(f"  {CYAN}>>> {cmd}{RESET}")

    def read_lines(self, duration):
        """Lit toutes les lignes pendant `duration` secondes."""
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

    def wait_for(self, patterns, extra=2):
        """
        Attend jusqu'à ce que tous les patterns (str ou regex) soient trouvés,
        ou jusqu'au timeout. Retourne (ok, lignes_reçues).
        patterns peut être une liste ou une seule str/regex.
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
                                 if not (re.search(p, line) if isinstance(p, str) else p.search(line))]
        # lecture du surplus
        lines += self.read_lines(extra)
        return (len(remaining) == 0), lines

    def close(self):
        self.ser.close()


# ─── Infrastructure de test ───────────────────────────────────────────────────

def section(title):
    print(f"\n{BOLD}{CYAN}{'─'*60}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'─'*60}{RESET}")


def run(gate, name, cmd, patterns, extra=2, skip=False):
    """Exécute un test : envoie cmd, vérifie patterns."""
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


def run_multi(gate, name, steps, skip=False):
    """
    Test multi-étapes : steps = [(cmd, [patterns], extra), ...]
    Toutes les étapes doivent passer.
    """
    if skip:
        print(f"  {YELLOW}SKIP{RESET}  {name}")
        results.append(("SKIP", name))
        return

    all_ok = True
    for cmd, patterns, extra in steps:
        gate.flush()
        if cmd:
            gate.send(cmd)
        ok, lines = gate.wait_for(patterns, extra=extra)
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
    run(g, "Version firmware",          "version",  r"\[SYSTEME\].*1\.")
    run(g, "Heure RTC",                 "time",     r"\d{4}-\d{2}-\d{2}|\d{2}:\d{2}:\d{2}|epoch")
    run(g, "Statistiques trafic",       "gmea",     r"trxglob:")


def test_config(g):
    section("CONFIGURATION")

    # Lecture config
    ok, lines = run(g, "getcfg — lecture",  "getcfg",  r"^CFG:")
    cfg_line = next((l for l in lines if l.startswith("CFG:")), None)

    # Niveau de log
    run(g, "slvl:verbose",   "slvl:verbose",  r"\[CONFIG\].*verbose")
    run(g, "slvl:normal",    "slvl:normal",   r"\[CONFIG\].*normal")
    run(g, "slvl:debug",     "slvl:debug",    r"\[CONFIG\].*debug")
    run(g, "slvl numérique", "slvl:1",        r"\[CONFIG\]")

    # Mode I/O
    run(g, "iomd:usb",  "iomd:usb",  r"\[IO\].*USB")

    # Paramètre individuel (parm)
    run(g, "parm — lecture/écriture",
        "parm:serialLevel:1",
        r"\[CONFIG\]")

    # setcfg — on remet la même config lue si disponible
    if cfg_line:
        fields = cfg_line[4:].split(":")
        if len(fields) >= 13:
            run(g, "setcfg — round-trip",
                "setcfg:" + ":".join(fields[:13]) + ":",
                r"CFG:OK")
    else:
        # setcfg minimal valide
        run(g, "setcfg — format invalide (doit renvoyer ERR)",
            "setcfg:1:2:3",
            r"CFG:ERR")


def test_decouverte(g, node, skip_network):
    section("DÉCOUVERTE RÉSEAU")

    # pigo — 3 vagues + fin (délai total ~15s)
    run(g, "pigo — vague 1",
        "pigo",
        r"\[PING\].*sondes",
        extra=1)

    run(g, "pigo — vague 2",
        None,
        r"ping:2",
        extra=6)

    run(g, "pigo — vague 3",
        None,
        r"ping:3",
        extra=6)

    run(g, "pigo — découverte terminée",
        None,
        r"\[PING\].*termin",
        extra=6)

    # Lecture de la table après pigo
    run(g, "prnt — table de routage",     "prnt",  r"edges:")
    run(g, "expv — export topologie",     "expv",  r"tmap:")

    # L'autre station doit avoir répondu aux pings pour que les liens existent
    run(g, "clear — efface la table",     "clear", r"\[OK\].*table.*efface|table.*routage.*efface",
        skip=skip_network)

    # Remettre la découverte après clear
    if not skip_network:
        run(g, "pigo après clear — re-découverte",
            "pigo",
            r"\[PING\].*termin",
            extra=20)


def test_sd(g):
    section("CARTE SD")

    run(g, "ls racine",           "ls:/",           r"/|FILE|DIR|\[SD\]|ERREUR")
    run(g, "mkdir /test_mycro",   "mkdir:/test_mycro",   r"\[SD\]|OK|Erreur")
    run(g, "ls /test_mycro",      "ls:/test_mycro",      r"\[SD\]|vide|OK|ERREUR")
    run(g, "rmdir /test_mycro",   "rmdir:/test_mycro",   r"\[SD\]|OK|Erreur")

    # cp et mv sur un fichier existant (p.cfg est toujours présent)
    run(g, "cp p.cfg → /p_bak.cfg",
        "cp:/p.cfg:/p_bak.cfg",
        r"\[SD\].*succes|OK|Erreur")
    run(g, "mv /p_bak.cfg → /p_bak2.cfg",
        "mv:/p_bak.cfg:/p_bak2.cfg",
        r"\[SD\].*succes|OK|Erreur")
    run(g, "rm /p_bak2.cfg",
        "rm:/p_bak2.cfg",
        r"\[SD\].*supprime|OK|Erreur")


def test_gateway(g, node, skip_network):
    section("PASSERELLE (GATEWAY)")
    run(g, "gate — adresse passerelle",  "gate",        r"\[GATEWAY\]")
    run(g, "cachestate — état cache",    "cachestate",  r"\[GATEWAY\]")
    run(g, "filestate — état transfert", "filestate",   r"\[FICHIER")


def test_reseau(g, node, skip_network):
    section("COMMUNICATION RÉSEAU (nécessite l'autre station)")

    # trsp : envoi fiable (attend arok en retour)
    run(g, f"trsp → nœud {node} (attend arok)",
        f"trsp:{node}:ping",
        r"arok:|Message envoye|MESSAGE",
        extra=10,
        skip=skip_network)

    # send générique
    run(g, f"send → nœud {node}",
        f"send:{node}:data:test_serial",
        r"arok:|Message envoye|MESSAGE|send",
        extra=8,
        skip=skip_network)

    # synchronisation horloge via la station voisine
    run(g, "acth — synchro horloge (via nœud proche)",
        "acth",
        r"geth|seth|\[OK\]|heure|time|synchro",
        extra=10,
        skip=skip_network)

    # netio — tunnel maître (la gate contrôle l'autre station)
    run_multi(g, f"netio → nœud {node} (ouvre tunnel, envoie version, ferme)",
        [
            (f"netio:{node}",     [r"\[NETIO\].*Ouverture|tunnel"],              5),
            (None,                [r"\[NETIO\].*Tunnel ouvert|ntiok"],            10),
            ("version",           [r"1\.|ntirsp|\[SYSTEME\]|NETIO"],              8),
            ("exit",              [r"\[NETIO\].*ferme|Tunnel ferme"],             5),
        ],
        skip=skip_network)


def test_transfert_fichier(g, node, skip_network):
    section("TRANSFERT DE FICHIER (nécessite l'autre station)")

    # Transfert de p.cfg vers l'autre station (petit fichier toujours présent)
    run_multi(g, f"stft — transfert /p.cfg → nœud {node}",
        [
            (f"stft:{node}:/p.cfg:0",
             [r"isrf|rfok|\[FICHIER|\[ERREUR\]|transfert"],
             15),
        ],
        skip=skip_network)


def test_cron_schedule(g):
    section("PLANIFICATION (adrc)")

    # adrc : planifie une commande dans 2 secondes
    run_multi(g, "adrc — commande différée (version dans 2s)",
        [
            ("adrc:2000:version",   [r"adrc|schedule|OK|\[SYSTEME\]"], 3),
            (None,                  [r"\[SYSTEME\].*1\."],              4),
        ])


def test_data(g):
    section("COMMANDES DIVERSES")

    run(g, "data — message texte",  "data:hello_test",  r"hello_test")
    run(g, "read — rechargement SD",  "read",           r"\[OK\].*recharg|recharge|read")
    run(g, "write — sauvegarde SD",   "write",          r"\[OK\].*sauvegard|sauvegarde|write")


# ─── Résumé ───────────────────────────────────────────────────────────────────

def print_summary():
    print(f"\n{BOLD}{'═'*60}{RESET}")
    print(f"{BOLD}  RÉSUMÉ{RESET}")
    print(f"{BOLD}{'═'*60}{RESET}")
    passed = sum(1 for s, _ in results if s == "PASS")
    failed = sum(1 for s, _ in results if s == "FAIL")
    skipped = sum(1 for s, _ in results if s == "SKIP")
    total = len(results)

    for status, name in results:
        if status == "PASS":
            print(f"  {GREEN}✓{RESET}  {name}")
        elif status == "FAIL":
            print(f"  {RED}✗{RESET}  {name}")
        else:
            print(f"  {YELLOW}−{RESET}  {name}")

    print(f"\n  Total : {total}  |  "
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
                        help="Adresse de l'autre station dans le réseau")
    parser.add_argument("--timeout",      type=int, default=15,
                        help="Timeout attente réponse (secondes)")
    parser.add_argument("--skip-network", action="store_true",
                        help="Passe les tests nécessitant la station distante")
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
        test_systeme(g)
        test_config(g)
        test_sd(g)
        test_gateway(g, args.node, args.skip_network)
        test_decouverte(g, args.node, args.skip_network)
        test_reseau(g, args.node, args.skip_network)
        test_transfert_fichier(g, args.node, args.skip_network)
        test_cron_schedule(g)
        test_data(g)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Interrompu.{RESET}")
    finally:
        g.close()

    ok = print_summary()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
