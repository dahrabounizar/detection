"""
=============================================================
 INTERFACE PONT ROULANT 3 AXES — MANUEL + AUTOMATIQUE
 Rotation Dynamixel + Aimant + Obstacles + Vue 2D
 -----------------------------------------------------------
 VERSION CORRIGÉE :
  [FIX #1] DTR/RTS désactivés avant ouverture port (anti-reset Mega)
  [FIX #2] Mots-clés aimant alignés avec Arduino (AIMANT_ON/OFF)
  [FIX #3] Détection d'erreurs restreinte à "ERREUR:" / "STOP &"
  [FIX #4] Thread lecture série démarré AVANT le sleep boot
  [FIX #5] Logs explicites sur timeouts et états bloqués
  [FIX #6] Mots-clés aimant corrigés dans la mission auto
  [FIX #7] try/finally sur tous les threads pour garantir busy=False

 ÉVOLUTIONS MISSION AUTO :
  - Planification de chemin XY (Dijkstra sur graphe de visibilité)
  - Obstacles BAS  : survolés (Z_sec local par segment)
  - Obstacles HAUTS : contournés en XY via coins étendus + marge
  - Validation préliminaire (positions, espace de travail, obstacles)
  - Aperçu visuel du chemin sur la carte (3 chemins distincts)
  - Z différenciés pour fardeau et livraison
  - Refus si chemin impossible (obstacles infranchissables)

 NOUVEAUTÉS V2 :
  - [NEW] Mouvements XYZ SIMULTANÉS sur les trajets aller/retour/origine
          (interpolation linéaire de Z, clampée par hauteur de survol)
  - [NEW] Retour automatique à la position initiale (0,0,0) après dépose,
          avec planification d'évitement d'obstacles indépendante
  - [NEW] Pause configurable + log clair entre dépose et retour origine
  - [NEW] Phase d'attrapage/dépose avec marge d'approche (5 cm) pour
          sécurité — descente Z pure sur les derniers cm
  - [NEW] Interface refondue en LIGHT THEME moderne (palette claire,
          icônes améliorées, typographie soignée, accents colorés)
  - [NEW] 3ème couleur de chemin (orange) pour le retour origine
=============================================================
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import queue
import math
import heapq

# ================== CONSTANTES ================================
WORKSPACE_X = 0.71
WORKSPACE_Y = 0.37
WORKSPACE_Z = 0.60
L_CABLE_INIT = 0.10

SCALE = 800
CANVAS_W = int(WORKSPACE_X * SCALE)
CANVAS_H = int(WORKSPACE_Y * SCALE)

V_XY = 0.15
A_XY = 0.15
MARGE_Z = 0.03                    # Marge sécurité Z au-dessus des obstacles bas (m)
MARGE_XY = 0.03                   # Marge sécurité XY autour des obstacles (m)
MARGE_APPROCHE_Z = 0.05           # Marge d'approche Z avant attrapage/dépose (5 cm)
PAUSE_AVANT_RETOUR_ORIGINE = 1.5  # Pause (s) entre fin de dépose et retour origine

BAUDRATE = 115200
BOOT_DELAY_S = 2.5  # [FIX #4] Mega 2560 : 2.5s pour assurer fin du bootloader

# ================== PALETTE LIGHT THEME =======================
# Fonds
BG_MAIN     = '#f5f7fa'   # Fond principal de la fenêtre
BG_PANEL    = '#ffffff'   # Fond des panneaux et LabelFrames
BG_INPUT    = '#f9fafb'   # Fond des champs de saisie
BG_CANVAS   = '#fafbfc'   # Fond du canvas (vue XY)
# Bordures
BORDER       = '#e5e7eb'  # Bordures et séparateurs
BORDER_FOCUS = '#0066cc'  # Bordure au focus
# Texte
FG_PRIMARY   = '#1f2937'  # Texte principal (presque noir)
FG_SECONDARY = '#6b7280'  # Texte secondaire (gris)
FG_MUTED     = '#9ca3af'  # Texte légendes / labels grille
# Accents
ACCENT        = '#0066cc'  # Bleu principal (actions, headers)
ACCENT_HOVER  = '#0052a3'  # Bleu hover
SUCCESS       = '#10b981'  # Vert (aimant ON, succès, fardeau)
SUCCESS_HOVER = '#059669'
DANGER        = '#ef4444'  # Rouge (obstacles, déconnecter)
DANGER_HOVER  = '#dc2626'
WARNING       = '#f59e0b'  # Orange (warning, retour origine, pont)
WARNING_HOVER = '#d97706'
INFO          = '#9333ea'  # Violet (livraison, retour livraison)
# Grille canvas
GRID_MAJOR  = '#d1d5db'
GRID_MINOR  = '#e5e7eb'
# Chemins
PATH_GO     = '#0066cc'   # Chemin aller (bleu)
PATH_RETURN = '#9333ea'   # Chemin retour livraison (violet)
PATH_HOME   = '#f59e0b'   # Chemin retour origine (orange)
# Pont
PONT_COLOR  = '#f59e0b'
PONT_OUTLINE = '#fbbf24'
# Journal (sombre pour contraste console)
LOG_BG = '#1f2937'
LOG_FG = '#10b981'


# ================== SERIAL MANAGER ============================

class SerialManager:
    """
    Gestion de la liaison série avec l'Arduino Mega 2560.
    Corrige le bug critique du reset DTR sous Windows.
    """

    def __init__(self, log_callback):
        self.ser = None
        self.log = log_callback
        self.response_queue = queue.Queue()
        self.running = False

    def lister_ports(self):
        """Liste les ports COM disponibles."""
        return [p.device for p in serial.tools.list_ports.comports()]

    def connecter(self, port):
        """
        Ouvre le port en désactivant DTR/RTS pour empêcher
        le reset permanent de l'Arduino Mega sous Windows.
        Démarre le thread de lecture AVANT le sleep boot.
        """
        try:
            # [FIX #1] Pré-configuration sans ouverture immédiate
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = BAUDRATE
            self.ser.timeout = 0.1
            self.ser.dtr = False   # Empêche le reset Mega à l'ouverture
            self.ser.rts = False   # Empêche aussi un reset via RTS
            self.ser.open()

            # [FIX #4] Thread démarré AVANT le sleep boot
            # pour capturer les messages de boot Arduino
            self.running = True
            threading.Thread(target=self._lire_serie, daemon=True).start()

            # Le Mega reset quand même si DTR a été pulsé pendant l'open.
            # On laisse le temps au bootloader de finir.
            time.sleep(BOOT_DELAY_S)

            # On vide la queue (boot messages) pour partir propre
            while not self.response_queue.empty():
                try:
                    self.response_queue.get_nowait()
                except queue.Empty:
                    break

            self.log(f"✓ Connecté à {port} (DTR/RTS désactivés)")
            return True

        except Exception as e:
            self.log(f"✗ Erreur de connexion : {e}")
            self.ser = None
            return False

    def deconnecter(self):
        """Ferme proprement le port série."""
        self.running = False
        time.sleep(0.15)  # laisse le temps au thread de sortir
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.log("Déconnecté.")

    def est_connecte(self):
        """Indique si le port est ouvert et utilisable."""
        try:
            return self.ser is not None and self.ser.is_open
        except Exception:
            return False

    def _lire_serie(self):
        """
        Boucle de lecture série en tâche de fond.
        Push tous les messages dans response_queue + log.
        """
        while self.running:
            try:
                if self.ser is None or not self.ser.is_open:
                    break
                if self.ser.in_waiting:
                    ligne = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if ligne:
                        self.log(f"[ARDUINO] {ligne}")
                        self.response_queue.put(ligne)
                else:
                    time.sleep(0.01)
            except Exception:
                break

    def envoyer(self, cmd):
        """Envoie une commande terminée par \\n vers l'Arduino."""
        if self.est_connecte():
            try:
                self.ser.write(f"{cmd}\n".encode())
                self.log(f"→ {cmd}")
            except Exception as e:
                self.log(f"✗ Erreur d'envoi : {e}")
        else:
            self.log("✗ Non connecté !")

    def attendre_message(self, mot_cle, timeout=120):
        """
        Attend un message Arduino contenant `mot_cle`.

        [FIX #3] Détection d'erreur restreinte à des préfixes explicites :
          - "ERREUR:"  (réponse Arduino formelle, ex. 'ERREUR: Tapez init')
          - "STOP &"   (recalibrage déclenché par fin de course)
        Cela évite les faux positifs sur des logs informatifs.

        [FIX #5] Log explicite en cas de timeout / d'erreur détectée.
        """
        t0 = time.time()
        while (time.time() - t0) < timeout:
            try:
                msg = self.response_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if mot_cle in msg:
                return True

            # [FIX #3] Détection précise des messages d'erreur uniquement
            if msg.startswith("ERREUR:"):
                self.log(f"⚠ Erreur Arduino détectée : {msg}")
                return False
            if "STOP &" in msg:
                self.log(f"⚠ Fin de course Arduino : {msg}")
                return False

        self.log(f"⚠ Timeout en attendant '{mot_cle}' ({timeout}s)")
        return False


# ================== MISSION AUTO ==============================

class Mission:
    """
    Mission automatique avec planification de chemin évitant les obstacles.

    Stratégie hybride :
      - Obstacles BAS  (z_max + MARGE_Z ≤ WORKSPACE_Z) : survolés
      - Obstacles HAUTS (au-dessus du plafond Z atteignable) : contournés en XY
        via Dijkstra sur graphe de visibilité (coins étendus + marges)
    """

    def __init__(self, serial_mgr, log_cb):
        self.s = serial_mgr
        self.log = log_cb

    # ===================================================================
    # OUTILS GÉOMÉTRIQUES
    # ===================================================================

    @staticmethod
    def _rect_etendu(o, marge):
        """
        Renvoie le rectangle (x1, y1, x2, y2) d'un obstacle avec marge ajoutée
        sur chaque côté. (x1,y1) = coin bas-gauche, (x2,y2) = coin haut-droit.
        """
        x1 = o['x'] - marge
        y1 = o['y'] - marge
        x2 = o['x'] + o['l'] + marge
        y2 = o['y'] + o['w'] + marge
        return (x1, y1, x2, y2)

    @staticmethod
    def _point_dans_rect(px, py, rect):
        """Test : le point (px, py) est-il à l'intérieur (ou sur le bord) du rectangle ?"""
        x1, y1, x2, y2 = rect
        return (x1 <= px <= x2) and (y1 <= py <= y2)

    @staticmethod
    def _point_strict_dans_rect(px, py, rect, eps=1e-9):
        """
        Test : le point est-il STRICTEMENT à l'intérieur du rectangle
        (avec une petite tolérance pour les coins). Utilisé par le test
        d'intersection pour ne pas bloquer un segment partant d'un coin.
        """
        x1, y1, x2, y2 = rect
        return (x1 + eps < px < x2 - eps) and (y1 + eps < py < y2 - eps)

    def _point_dans_obstacle(self, x, y, obstacles, marge=MARGE_XY):
        """True si le point (x,y) est dans au moins un obstacle (avec marge)."""
        for o in obstacles:
            if self._point_dans_rect(x, y, self._rect_etendu(o, marge)):
                return True
        return False

    @staticmethod
    def _segments_intersectent(p1, p2, p3, p4):
        """
        Test d'intersection STRICTE entre deux segments [p1,p2] et [p3,p4].
        Algo classique par produits vectoriels (orientation).
        Note : retourne False si les segments sont colinéaires ou se touchent
        seulement par une extrémité (cas exactement aux coins).
        """
        def ccw(a, b, c):
            return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
        return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and \
               (ccw(p1, p2, p3) != ccw(p1, p2, p4))

    def _segment_intersecte_rect(self, p1, p2, rect):
        """
        True si le segment p1→p2 traverse le rectangle.
        Un segment partant ou arrivant exactement à un coin (ou sur le bord)
        N'EST PAS considéré comme intersectant — c'est essentiel pour le
        graphe de visibilité où les waypoints sont les coins eux-mêmes.
        """
        # Une extrémité STRICTEMENT à l'intérieur → traversée
        if self._point_strict_dans_rect(p1[0], p1[1], rect) or \
           self._point_strict_dans_rect(p2[0], p2[1], rect):
            return True

        x1, y1, x2, y2 = rect
        coins = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        eps = 1e-9

        # Détecter si p1 ou p2 EST exactement un coin du rectangle
        def _est_coin(p):
            return any(abs(p[0] - c[0]) < eps and abs(p[1] - c[1]) < eps
                       for c in coins)
        p1_coin = _est_coin(p1)
        p2_coin = _est_coin(p2)

        cotes = [(coins[0], coins[1]), (coins[1], coins[2]),
                 (coins[2], coins[3]), (coins[3], coins[0])]

        for c1, c2 in cotes:
            # Si une extrémité du segment EST un coin de ce côté,
            # le segment touche ce côté seulement par ce coin → pas une traversée
            if p1_coin and ((abs(p1[0]-c1[0]) < eps and abs(p1[1]-c1[1]) < eps) or
                            (abs(p1[0]-c2[0]) < eps and abs(p1[1]-c2[1]) < eps)):
                continue
            if p2_coin and ((abs(p2[0]-c1[0]) < eps and abs(p2[1]-c1[1]) < eps) or
                            (abs(p2[0]-c2[0]) < eps and abs(p2[1]-c2[1]) < eps)):
                continue
            if self._segments_intersectent(p1, p2, c1, c2):
                return True
        return False

    def _z_securite_segment(self, A, B, obstacles_bas):
        """
        Pour un segment XY donné, retourne la hauteur Z minimale pour
        survoler tous les obstacles bas que ce segment traverse.
        Si aucun obstacle survolé → 0.0 (déplacement à hauteur basse).
        """
        z_max_local = 0.0
        for o in obstacles_bas:
            rect = self._rect_etendu(o, MARGE_XY)
            if self._segment_intersecte_rect(A, B, rect):
                z_max_local = max(z_max_local, o['z_max'] + MARGE_Z)
        return min(z_max_local, WORKSPACE_Z)

    # ===================================================================
    # PLANIFICATION DE CHEMIN (Dijkstra sur graphe de visibilité)
    # ===================================================================

    def _planifier_chemin_xy(self, A, B, obstacles):
        """
        Calcule un chemin XY de A à B évitant les obstacles HAUTS.
        Les obstacles bas seront survolés (gérés par _z_securite_segment).

        Retourne :
          - liste de points [(x,y), ...] du chemin (incluant A et B)
          - None si aucun chemin trouvable
        """
        # Séparation bas / hauts (selon faisabilité du survol)
        obstacles_hauts = [o for o in obstacles
                           if (o['z_max'] + MARGE_Z) > WORKSPACE_Z]

        # Cas trivial : aucun obstacle haut → ligne droite
        if not obstacles_hauts:
            return [A, B]

        # Vérifier que A et B ne sont pas à l'intérieur d'un obstacle haut
        if self._point_dans_obstacle(A[0], A[1], obstacles_hauts):
            self.log(f"⚠ Point de départ ({A[0]:.2f},{A[1]:.2f}) dans un obstacle haut")
            return None
        if self._point_dans_obstacle(B[0], B[1], obstacles_hauts):
            self.log(f"⚠ Point d'arrivée ({B[0]:.2f},{B[1]:.2f}) dans un obstacle haut")
            return None

        # Construction des sommets : A, B, + 4 coins étendus de chaque obstacle haut
        sommets = [A, B]
        for o in obstacles_hauts:
            x1, y1, x2, y2 = self._rect_etendu(o, MARGE_XY)
            for c in [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]:
                # Ne garder que les coins dans l'espace de travail
                if 0 <= c[0] <= WORKSPACE_X and 0 <= c[1] <= WORKSPACE_Y:
                    # Et qui ne sont pas STRICTEMENT à l'intérieur d'un autre
                    # obstacle haut (un coin sur le bord d'un autre est OK comme waypoint)
                    inside_other = False
                    for o2 in obstacles_hauts:
                        if o2 is o:
                            continue
                        if self._point_strict_dans_rect(
                                c[0], c[1], self._rect_etendu(o2, MARGE_XY)):
                            inside_other = True
                            break
                    if not inside_other:
                        sommets.append(c)

        n = len(sommets)

        # Construction des arêtes : segment ne traversant aucun obstacle haut
        rects_hauts = [self._rect_etendu(o, MARGE_XY) for o in obstacles_hauts]
        adj = [[] for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                p1, p2 = sommets[i], sommets[j]
                bloque = False
                for rect in rects_hauts:
                    if self._segment_intersecte_rect(p1, p2, rect):
                        bloque = True
                        break
                if not bloque:
                    d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                    adj[i].append((j, d))
                    adj[j].append((i, d))

        # Dijkstra : 0 = A, 1 = B
        INF = float('inf')
        dist = [INF] * n
        prev = [-1] * n
        dist[0] = 0.0
        pq = [(0.0, 0)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue
            if u == 1:
                break
            for v, w in adj[u]:
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))

        if dist[1] == INF:
            return None

        # Reconstruction du chemin (de B vers A puis inversion)
        chemin = []
        cur = 1
        while cur != -1:
            chemin.append(sommets[cur])
            cur = prev[cur]
        chemin.reverse()
        return chemin

    # ===================================================================
    # VALIDATION PRÉLIMINAIRE
    # ===================================================================

    def _validation_preliminaire(self, fardeau, livraison, obstacles):
        """
        Vérifications avant tout mouvement.
        Retourne (ok: bool, message: str).
        """
        # Espace de travail
        for nom, p in [("Fardeau", fardeau), ("Livraison", livraison)]:
            if not (0 <= p['x'] <= WORKSPACE_X):
                return False, f"{nom} : X={p['x']} hors espace [0..{WORKSPACE_X}]"
            if not (0 <= p['y'] <= WORKSPACE_Y):
                return False, f"{nom} : Y={p['y']} hors espace [0..{WORKSPACE_Y}]"
            if not (0 <= p['z'] <= WORKSPACE_Z):
                return False, f"{nom} : Z={p['z']} hors espace [0..{WORKSPACE_Z}]"

        # Pas dans un obstacle (avec marge)
        if self._point_dans_obstacle(fardeau['x'], fardeau['y'], obstacles):
            return False, "Fardeau positionné dans (ou trop près d') un obstacle"
        if self._point_dans_obstacle(livraison['x'], livraison['y'], obstacles):
            return False, "Livraison positionnée dans (ou trop près d') un obstacle"

        return True, "OK"

    # ===================================================================
    # GÉNÉRATION DES SEGMENTS DE MOUVEMENT
    # ===================================================================

    def _segments_avec_z(self, chemin, obstacles):
        """
        Convertit une liste de waypoints XY en liste de segments avec
        Z de sécurité local pour chaque segment.

        Retourne : liste de tuples (x_dest, y_dest, z_sec_local)
        """
        obstacles_bas = [o for o in obstacles
                         if (o['z_max'] + MARGE_Z) <= WORKSPACE_Z]

        segments = []
        for i in range(len(chemin) - 1):
            A = chemin[i]
            B = chemin[i + 1]
            z = self._z_securite_segment(A, B, obstacles_bas)
            segments.append((B[0], B[1], z))
        return segments

    def _segments_avec_z_interpole(self, chemin, z_depart, z_arrivee, obstacles):
        """
        Convertit une liste de waypoints XY en liste de segments avec Z
        interpolé linéairement entre z_depart et z_arrivee selon la longueur
        cumulée parcourue. Le Z calculé est ensuite *clampé* par la hauteur
        de sécurité minimale du segment (pour ne jamais traverser un obstacle
        bas), garantissant un mouvement XYZ simultané sûr.

        Retourne : liste de tuples (x_dest, y_dest, z_dest_clampe)
        """
        # Cas dégénérés
        if not chemin or len(chemin) < 2:
            return []

        obstacles_bas = [o for o in obstacles
                         if (o['z_max'] + MARGE_Z) <= WORKSPACE_Z]

        # Longueurs des segments et longueur totale
        longueurs = []
        for i in range(len(chemin) - 1):
            dx = chemin[i + 1][0] - chemin[i][0]
            dy = chemin[i + 1][1] - chemin[i][1]
            longueurs.append(math.hypot(dx, dy))
        long_totale = sum(longueurs)

        # Cas trivial : tous les waypoints sont au même point XY
        # → un seul segment qui termine en (x,y,z_arrivee)
        if long_totale < 1e-6:
            B = chemin[-1]
            return [(B[0], B[1], z_arrivee)]

        segments = []
        long_cumulee = 0.0
        for i in range(len(chemin) - 1):
            A = chemin[i]
            B = chemin[i + 1]
            long_cumulee += longueurs[i]
            # Fraction parcourue à la fin du segment courant
            frac = long_cumulee / long_totale
            # Z interpolé linéairement à l'extrémité B du segment
            z_interp = z_depart + frac * (z_arrivee - z_depart)
            # Clamp par la hauteur de survol obligatoire
            z_secu = self._z_securite_segment(A, B, obstacles_bas)
            z_final = max(z_interp, z_secu)
            # Sécurité finale : ne jamais dépasser le plafond Z atteignable
            if z_final > WORKSPACE_Z:
                z_final = WORKSPACE_Z
            segments.append((B[0], B[1], z_final))
        return segments

    # ===================================================================
    # EXÉCUTION DE LA MISSION
    # ===================================================================

    def executer(self, fardeau, livraison, obstacles, angle_pickup, angle_livraison,
                 chemin_aller=None, chemin_retour=None, chemin_origine=None,
                 cb_pos=None):
        """
        Exécute la mission complète avec planification d'évitement et
        mouvements XYZ simultanés. Inclut un retour à l'origine (0,0,0)
        après la dépose.

        Si chemin_aller / chemin_retour / chemin_origine sont fournis (depuis
        la GUI qui a déjà appelé _planifier_chemin_xy pour l'aperçu), ils
        sont utilisés directement. Sinon, planification ici.

        Séquence :
          3.a  Trajet aller XYZ simultané (Z 0 → fardeau.z + MARGE_APPROCHE_Z)
          3.b  Rotation attrapage
          3.c  Descente Z pure sur fardeau
          3.d  Aimant ON
          3.e  Remontée Z à fardeau.z + MARGE_APPROCHE_Z
          3.f  Trajet retour XYZ simultané vers livraison.z + MARGE_APPROCHE_Z
          3.g  Rotation livraison
          3.h  Descente Z pure sur livraison
          3.i  Aimant OFF
          3.j  Remontée Z à livraison.z + MARGE_APPROCHE_Z
          3.k  Pause + log "début retour origine"
          3.l  Trajet retour-origine XYZ simultané vers (0,0,0)
          3.m  Log final
        """
        self.log("\n" + "=" * 45)
        self.log("   DÉBUT DE MISSION AUTOMATIQUE")
        self.log("=" * 45)

        # --- Phase 0 : validation préliminaire ---
        ok, msg = self._validation_preliminaire(fardeau, livraison, obstacles)
        if not ok:
            self.log(f"⚠ Validation échouée : {msg}")
            return False
        self.log(f"✓ Validation OK")

        # --- Phase 1 : planification chemins (si non fournis) ---
        A_dep = (0.0, 0.0)
        A_far = (fardeau['x'], fardeau['y'])
        A_liv = (livraison['x'], livraison['y'])

        if chemin_aller is None:
            chemin_aller = self._planifier_chemin_xy(A_dep, A_far, obstacles)
        if chemin_retour is None:
            chemin_retour = self._planifier_chemin_xy(A_far, A_liv, obstacles)
        if chemin_origine is None:
            chemin_origine = self._planifier_chemin_xy(A_liv, A_dep, obstacles)

        if chemin_aller is None:
            self.log("⚠ Aucun chemin trouvé : départ → fardeau (obstacles infranchissables)")
            return False
        if chemin_retour is None:
            self.log("⚠ Aucun chemin trouvé : fardeau → livraison (obstacles infranchissables)")
            return False
        # Fallback retour origine : si planification impossible, on inverse l'aller
        # (qui est garanti faisable puisqu'il a déjà été parcouru à l'aller).
        if chemin_origine is None:
            self.log("⚠ Aucun chemin trouvé pour retour origine — fallback : inversion du chemin aller")
            chemin_origine = list(reversed(chemin_aller))

        long_aller = sum(math.hypot(chemin_aller[i+1][0]-chemin_aller[i][0],
                                    chemin_aller[i+1][1]-chemin_aller[i][1])
                         for i in range(len(chemin_aller)-1))
        long_retour = sum(math.hypot(chemin_retour[i+1][0]-chemin_retour[i][0],
                                     chemin_retour[i+1][1]-chemin_retour[i][1])
                          for i in range(len(chemin_retour)-1))
        long_origine = sum(math.hypot(chemin_origine[i+1][0]-chemin_origine[i][0],
                                      chemin_origine[i+1][1]-chemin_origine[i][1])
                           for i in range(len(chemin_origine)-1))
        self.log(f"✓ Chemin aller   : {len(chemin_aller)-1} segment(s), longueur {long_aller:.2f} m")
        self.log(f"✓ Chemin retour  : {len(chemin_retour)-1} segment(s), longueur {long_retour:.2f} m")
        self.log(f"✓ Chemin origine : {len(chemin_origine)-1} segment(s), longueur {long_origine:.2f} m")

        # --- Phase 2 : génération des segments avec Z interpolé ---
        # Z de départ aller : 0 (position après homing)
        # Z d'arrivée aller : approche fardeau (fardeau.z + marge)
        z_approche_far = fardeau['z'] + MARGE_APPROCHE_Z
        z_approche_liv = livraison['z'] + MARGE_APPROCHE_Z

        segs_aller   = self._segments_avec_z_interpole(
            chemin_aller, 0.0, z_approche_far, obstacles)
        segs_retour  = self._segments_avec_z_interpole(
            chemin_retour, z_approche_far, z_approche_liv, obstacles)
        segs_origine = self._segments_avec_z_interpole(
            chemin_origine, z_approche_liv, 0.0, obstacles)

        # --- Phase 3 : exécution séquencée ---

        # 3.a Trajet aller XYZ simultané (départ → approche fardeau)
        self.log("\n--- Trajet aller XYZ simultané (départ → approche fardeau) ---")
        for i, (xt, yt, zt) in enumerate(segs_aller):
            self.log(f"  Waypoint {i+1}/{len(segs_aller)} : ({xt:.2f}, {yt:.2f}, Z={zt:.2f})")
            self.s.envoyer(f"{xt} {yt} {zt} {V_XY} {A_XY}")
            if not self.s.attendre_message("ARRIVE", 120):
                self.log("⚠ Mouvement aller échoué. STOP.")
                return False
            if cb_pos:
                cb_pos(xt, yt)

        # 3.b Rotation attrapage
        self.log(f"\n--- Rotation attrapage ({angle_pickup}°) ---")
        self.s.envoyer(f"rot {angle_pickup}")
        if not self.s.attendre_message("ROT_OK", 10):
            self.log("⚠ Rotation échouée.")
        time.sleep(1.5)

        # 3.c Descente Z pure sur le fardeau
        self.log(f"\n--- Descente sur fardeau (Z={fardeau['z']:.2f}) ---")
        self.s.envoyer(f"{fardeau['x']} {fardeau['y']} {fardeau['z']} {V_XY} {A_XY}")
        if not self.s.attendre_message("ARRIVE", 120):
            self.log("⚠ Descente fardeau échouée. STOP.")
            return False

        # 3.d Aimant ON
        self.log("\n--- Aimant ON ---")
        self.s.envoyer("aimant_on")
        self.s.attendre_message("AIMANT_ON", 5)
        time.sleep(0.5)

        # 3.e Remontée Z à fardeau.z + marge d'approche
        self.log(f"\n--- Remontée fardeau à Z={z_approche_far:.2f} ---")
        self.s.envoyer(f"{fardeau['x']} {fardeau['y']} {z_approche_far} {V_XY} {A_XY}")
        if not self.s.attendre_message("ARRIVE", 120):
            self.log("⚠ Remontée fardeau échouée. STOP.")
            return False

        # 3.f Trajet retour XYZ simultané (fardeau → approche livraison)
        self.log("\n--- Trajet retour XYZ simultané (fardeau → approche livraison) ---")
        for i, (xt, yt, zt) in enumerate(segs_retour):
            self.log(f"  Waypoint {i+1}/{len(segs_retour)} : ({xt:.2f}, {yt:.2f}, Z={zt:.2f})")
            self.s.envoyer(f"{xt} {yt} {zt} {V_XY} {A_XY}")
            if not self.s.attendre_message("ARRIVE", 120):
                self.log("⚠ Mouvement retour échoué. STOP.")
                return False
            if cb_pos:
                cb_pos(xt, yt)

        # 3.g Rotation livraison
        self.log(f"\n--- Rotation livraison ({angle_livraison}°) ---")
        self.s.envoyer(f"rot {angle_livraison}")
        if not self.s.attendre_message("ROT_OK", 10):
            self.log("⚠ Rotation échouée.")
        time.sleep(1.5)

        # 3.h Descente Z pure sur livraison
        self.log(f"\n--- Descente sur livraison (Z={livraison['z']:.2f}) ---")
        self.s.envoyer(f"{livraison['x']} {livraison['y']} {livraison['z']} {V_XY} {A_XY}")
        if not self.s.attendre_message("ARRIVE", 120):
            self.log("⚠ Descente livraison échouée. STOP.")
            return False

        # 3.i Aimant OFF (dépose)
        self.log("\n--- Aimant OFF (dépose) ---")
        self.s.envoyer("aimant_off")
        self.s.attendre_message("AIMANT_OFF", 5)
        time.sleep(0.5)

        Z_RETRAIT_SECURITE = 0.30  # Hauteur de sécurité après dépose

        # 3.j Remontée à Z=0.30 (hauteur de sécurité fixe après dépose)
        self.log(f"\n--- Remontée après dépose à Z={Z_RETRAIT_SECURITE:.2f} (hauteur sécurité) ---")
        self.s.envoyer(f"{livraison['x']} {livraison['y']} {Z_RETRAIT_SECURITE} {V_XY} {A_XY}")
        if not self.s.attendre_message("ARRIVE", 120):
            self.log("⚠ Remontée après dépose échouée. STOP.")
            return False

        # 3.k Pause + log "début retour origine"
        self.log(f"\n--- Dépose terminée. Pause {PAUSE_AVANT_RETOUR_ORIGINE:.1f}s avant retour origine ---")
        time.sleep(PAUSE_AVANT_RETOUR_ORIGINE)
        self.log("\n--- DÉBUT RETOUR À LA POSITION INITIALE (0, 0, 0) ---")

        # 3.l Trajet retour-origine : XY suivant le chemin planifié, Z maintenu à 0.30
        for i, (xt, yt, _zt) in enumerate(segs_origine):
            # On maintient Z=0.30 tout au long du retour XY (pas d'interpolation Z)
            zt_retour = Z_RETRAIT_SECURITE
            self.log(f"  Waypoint {i+1}/{len(segs_origine)} : ({xt:.2f}, {yt:.2f}, Z={zt_retour:.2f})")
            self.s.envoyer(f"{xt} {yt} {zt_retour} {V_XY} {A_XY}")
            if not self.s.attendre_message("ARRIVE", 120):
                self.log("⚠ Mouvement retour origine échoué. Le fardeau est livré mais le pont n'est pas à l'origine.")
                return False
            if cb_pos:
                cb_pos(xt, yt)

        # 3.m Descente finale à Z=0 pour terminer en (0, 0, 0)
        self.log("\n--- Descente finale à Z=0.00 (position origine) ---")
        self.s.envoyer(f"0.0 0.0 0.0 {V_XY} {A_XY}")
        if not self.s.attendre_message("ARRIVE", 120):
            self.log("⚠ Descente finale à l'origine échouée.")
            return False
        if cb_pos:
            cb_pos(0.0, 0.0)

        # 3.n Log final
        self.log("\n" + "=" * 45)
        self.log("   MISSION TERMINÉE ✓")
        self.log("   Fardeau livré, pont revenu à l'origine (0, 0, 0).")
        self.log("=" * 45)
        return True


# ================== GUI ======================================

class PontRoulantGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Pont Roulant 3 Axes — Contrôleur")
        self.root.configure(bg=BG_MAIN)

        self.serial_mgr = SerialManager(self.log_msg)
        self.mission = Mission(self.serial_mgr, self.log_msg)
        self.obstacles = []
        self.pont_pos = (0, 0)
        self.busy = False
        self.chemin_aller = None    # Liste de waypoints (x,y) ou None
        self.chemin_retour = None   # Liste de waypoints (x,y) ou None
        self.chemin_origine = None  # Liste de waypoints (x,y) ou None — retour origine

        self._build_styles()
        self._build_ui()
        self._draw_canvas()

    def _build_styles(self):
        """
        Configuration complète des styles ttk pour le thème light moderne.
        Toutes les couleurs sont issues de la palette définie en début de fichier.
        """
        s = ttk.Style()
        s.theme_use('clam')

        # ----- Frames -----
        s.configure('TFrame', background=BG_MAIN)
        s.configure('Panel.TFrame', background=BG_PANEL)

        # ----- Labels -----
        s.configure('TLabel',
                    background=BG_PANEL, foreground=FG_PRIMARY,
                    font=('Segoe UI', 9))
        s.configure('H.TLabel',
                    background=BG_PANEL, foreground=ACCENT,
                    font=('Segoe UI', 11, 'bold'))
        s.configure('Title.TLabel',
                    background=BG_MAIN, foreground=FG_PRIMARY,
                    font=('Segoe UI', 14, 'bold'))
        s.configure('Subtle.TLabel',
                    background=BG_MAIN, foreground=FG_SECONDARY,
                    font=('Segoe UI', 9))
        s.configure('PanelSubtle.TLabel',
                    background=BG_PANEL, foreground=FG_SECONDARY,
                    font=('Segoe UI', 8))

        # ----- LabelFrames (cadres titrés) -----
        s.configure('TLabelframe',
                    background=BG_PANEL, bordercolor=BORDER,
                    relief='solid', borderwidth=1)
        s.configure('TLabelframe.Label',
                    background=BG_PANEL, foreground=ACCENT,
                    font=('Segoe UI', 9, 'bold'))

        # ----- Boutons -----
        # Standard
        s.configure('TButton',
                    background=BG_PANEL, foreground=FG_PRIMARY,
                    font=('Segoe UI', 9),
                    borderwidth=1, relief='solid', bordercolor=BORDER,
                    padding=(8, 4), focuscolor=BORDER)
        s.map('TButton',
              background=[('active', BG_INPUT), ('pressed', BORDER)],
              bordercolor=[('focus', BORDER_FOCUS), ('active', ACCENT)])

        # Accent (bleu)
        s.configure('Accent.TButton',
                    background=ACCENT, foreground='white',
                    font=('Segoe UI', 9, 'bold'),
                    borderwidth=0, relief='flat',
                    padding=(10, 5), focuscolor=ACCENT_HOVER)
        s.map('Accent.TButton',
              background=[('active', ACCENT_HOVER), ('pressed', ACCENT_HOVER)])

        # Big (gros bouton primaire)
        s.configure('Big.TButton',
                    background=ACCENT, foreground='white',
                    font=('Segoe UI', 11, 'bold'),
                    borderwidth=0, relief='flat',
                    padding=(12, 8), focuscolor=ACCENT_HOVER)
        s.map('Big.TButton',
              background=[('active', ACCENT_HOVER), ('pressed', ACCENT_HOVER)])

        # Success (vert)
        s.configure('Success.TButton',
                    background=SUCCESS, foreground='white',
                    font=('Segoe UI', 9, 'bold'),
                    borderwidth=0, relief='flat',
                    padding=(10, 5), focuscolor=SUCCESS_HOVER)
        s.map('Success.TButton',
              background=[('active', SUCCESS_HOVER), ('pressed', SUCCESS_HOVER)])

        # Danger (rouge)
        s.configure('Danger.TButton',
                    background=DANGER, foreground='white',
                    font=('Segoe UI', 9, 'bold'),
                    borderwidth=0, relief='flat',
                    padding=(10, 5), focuscolor=DANGER_HOVER)
        s.map('Danger.TButton',
              background=[('active', DANGER_HOVER), ('pressed', DANGER_HOVER)])

        # Warning (orange)
        s.configure('Warning.TButton',
                    background=WARNING, foreground='white',
                    font=('Segoe UI', 9, 'bold'),
                    borderwidth=0, relief='flat',
                    padding=(10, 5), focuscolor=WARNING_HOVER)
        s.map('Warning.TButton',
              background=[('active', WARNING_HOVER), ('pressed', WARNING_HOVER)])

        # ----- Entries (champs de saisie) -----
        s.configure('TEntry',
                    fieldbackground=BG_INPUT, foreground=FG_PRIMARY,
                    bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER,
                    borderwidth=1, padding=4,
                    insertcolor=ACCENT)
        s.map('TEntry',
              bordercolor=[('focus', BORDER_FOCUS)],
              lightcolor=[('focus', BORDER_FOCUS)],
              darkcolor=[('focus', BORDER_FOCUS)])

        # ----- Combobox -----
        s.configure('TCombobox',
                    fieldbackground=BG_INPUT, background=BG_PANEL,
                    foreground=FG_PRIMARY,
                    bordercolor=BORDER, arrowcolor=FG_SECONDARY,
                    borderwidth=1, padding=3)
        s.map('TCombobox',
              fieldbackground=[('readonly', BG_INPUT)],
              bordercolor=[('focus', BORDER_FOCUS)])

        # ----- Notebook (onglets) -----
        s.configure('TNotebook',
                    background=BG_MAIN, borderwidth=0)
        s.configure('TNotebook.Tab',
                    background=BG_INPUT, foreground=FG_SECONDARY,
                    font=('Segoe UI', 10, 'bold'),
                    padding=(20, 8), borderwidth=0)
        s.map('TNotebook.Tab',
              background=[('selected', BG_PANEL), ('active', BG_PANEL)],
              foreground=[('selected', ACCENT), ('active', FG_PRIMARY)],
              expand=[('selected', [1, 1, 1, 0])])

        # ----- Scrollbar -----
        s.configure('TScrollbar',
                    background=BG_INPUT, troughcolor=BG_MAIN,
                    bordercolor=BORDER, arrowcolor=FG_SECONDARY,
                    borderwidth=0)
        s.map('TScrollbar',
              background=[('active', BORDER)])

    def _build_ui(self):
        # === HEADER : titre + sous-titre ===
        header = ttk.Frame(self.root, style='TFrame')
        header.pack(side=tk.TOP, fill=tk.X, padx=12, pady=(10, 4))
        ttk.Label(header, text="⚙  Pont Roulant 3 Axes",
                  style='Title.TLabel').pack(side=tk.LEFT)
        ttk.Label(header, text="Contrôleur de mission · Mode manuel & automatique",
                  style='Subtle.TLabel').pack(side=tk.LEFT, padx=(12, 0), pady=(6, 0))

        # === Séparateur sous header ===
        sep = tk.Frame(self.root, bg=BORDER, height=1)
        sep.pack(side=tk.TOP, fill=tk.X, padx=12)

        top = ttk.Frame(self.root, style='TFrame')
        top.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        # === LEFT: Canvas ===
        left = ttk.Frame(top, style='TFrame')
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        ttk.Label(left, text="VUE DE DESSUS (XY)",
                  style='Title.TLabel',
                  font=('Segoe UI', 10, 'bold')).pack(pady=(0, 6), anchor=tk.W)
        cf = tk.Frame(left, bg=BORDER, bd=0, relief=tk.FLAT,
                      highlightbackground=BORDER, highlightthickness=1)
        cf.pack(fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(cf, width=CANVAS_W, height=CANVAS_H,
                                bg=BG_CANVAS, highlightthickness=0)
        self.canvas.pack(padx=1, pady=1)

        # Légende — 5 entrées avec icônes colorées
        leg = ttk.Frame(left, style='TFrame')
        leg.pack(fill=tk.X, pady=8)
        legende_items = [
            ("■  Obstacle",        DANGER),
            ("●  Fardeau",         SUCCESS),
            ("★  Livraison",       INFO),
            ("▲  Pont",            PONT_COLOR),
            ("⤺  Retour origine",  PATH_HOME),
        ]
        for txt, col in legende_items:
            lbl = tk.Label(leg, text=txt, fg=col, bg=BG_MAIN,
                           font=('Segoe UI', 9, 'bold'))
            lbl.pack(side=tk.LEFT, padx=10)

        # === RIGHT: Controls ===
        right = ttk.Frame(top, width=360, style='TFrame')
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        self._build_connexion(right)

        # Notebook (tabs)
        self.notebook = ttk.Notebook(right)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=4, pady=6)

        tab_manuel = ttk.Frame(self.notebook, style='Panel.TFrame')
        tab_auto = ttk.Frame(self.notebook, style='Panel.TFrame')
        self.notebook.add(tab_manuel, text="  MANUEL  ")
        self.notebook.add(tab_auto, text="  AUTO  ")

        self._build_tab_manuel(tab_manuel)
        self._build_tab_auto(tab_auto)

        # === BOTTOM: Log ===
        lf = ttk.LabelFrame(self.root, text="  📋  Journal  ")
        lf.pack(fill=tk.X, padx=12, pady=(0, 10))
        self.log_text = scrolledtext.ScrolledText(
            lf, height=7,
            bg=LOG_BG, fg=LOG_FG,
            font=('Consolas', 9),
            insertbackground=LOG_FG,
            borderwidth=0, relief=tk.FLAT,
            wrap=tk.WORD)
        self.log_text.pack(fill=tk.X, padx=4, pady=4)

    # ---------- CONNEXION ----------
    def _build_connexion(self, parent):
        f = ttk.LabelFrame(parent, text="  🔌  Connexion série  ")
        f.pack(fill=tk.X, padx=4, pady=4)

        r1 = ttk.Frame(f, style='Panel.TFrame')
        r1.pack(fill=tk.X, padx=8, pady=8)
        self.port_var = tk.StringVar()
        ports = self.serial_mgr.lister_ports()
        self.port_cb = ttk.Combobox(r1, textvariable=self.port_var,
                                    values=ports, width=12, state='readonly')
        self.port_cb.pack(side=tk.LEFT, padx=(0, 4))
        if ports:
            self.port_cb.set(ports[0])
        ttk.Button(r1, text="⟳", width=3,
                   command=self._refresh_ports).pack(side=tk.LEFT, padx=2)
        self.btn_conn = ttk.Button(r1, text="Connecter",
                                   command=self._toggle_conn,
                                   style='Accent.TButton')
        self.btn_conn.pack(side=tk.LEFT, padx=4)

    # ---------- TAB MANUEL ----------
    def _build_tab_manuel(self, parent):
        # Init
        f_init = ttk.LabelFrame(parent, text="  🏠  Initialisation  ")
        f_init.pack(fill=tk.X, padx=6, pady=(6, 4))
        ttk.Button(f_init, text="🏠   HOMING (Init)",
                   command=self._do_homing,
                   style='Big.TButton').pack(fill=tk.X, padx=8, pady=8)

        # Mouvement manuel
        f_mvt = ttk.LabelFrame(parent, text="  ⊕  Mouvement  ")
        f_mvt.pack(fill=tk.X, padx=6, pady=4)

        r1 = ttk.Frame(f_mvt, style='Panel.TFrame')
        r1.pack(fill=tk.X, padx=8, pady=(8, 2))
        for lbl, default, attr in [("X:", "0.0", "man_x"),
                                   ("Y:", "0.0", "man_y"),
                                   ("Z:", "0.0", "man_z")]:
            ttk.Label(r1, text=lbl).pack(side=tk.LEFT)
            e = ttk.Entry(r1, width=7)
            e.pack(side=tk.LEFT, padx=3)
            e.insert(0, default)
            setattr(self, attr, e)

        r2 = ttk.Frame(f_mvt, style='Panel.TFrame')
        r2.pack(fill=tk.X, padx=8, pady=2)
        ttk.Label(r2, text="V:").pack(side=tk.LEFT)
        self.man_v = ttk.Entry(r2, width=6)
        self.man_v.pack(side=tk.LEFT, padx=3)
        self.man_v.insert(0, "0.15")
        ttk.Label(r2, text="A:").pack(side=tk.LEFT, padx=(8, 0))
        self.man_a = ttk.Entry(r2, width=6)
        self.man_a.pack(side=tk.LEFT, padx=3)
        self.man_a.insert(0, "0.15")

        ttk.Button(f_mvt, text="▶  Déplacer",
                   command=self._do_move,
                   style='Accent.TButton').pack(fill=tk.X, padx=8, pady=(6, 8))

        # Aimant — [FIX #2] Mots-clés alignés avec Arduino
        f_aim = ttk.LabelFrame(parent, text="  🧲  Aimant  ")
        f_aim.pack(fill=tk.X, padx=6, pady=4)
        r_aim = ttk.Frame(f_aim, style='Panel.TFrame')
        r_aim.pack(fill=tk.X, padx=8, pady=8)
        ttk.Button(r_aim, text="🧲  Aimant ON",
                   command=lambda: self._do_cmd("aimant_on", "AIMANT_ON"),
                   style='Success.TButton').pack(side=tk.LEFT,
                                                 expand=True, fill=tk.X, padx=2)
        ttk.Button(r_aim, text="🧲  Aimant OFF",
                   command=lambda: self._do_cmd("aimant_off", "AIMANT_OFF"),
                   style='Danger.TButton').pack(side=tk.LEFT,
                                                expand=True, fill=tk.X, padx=2)

        # Rotation
        f_rot = ttk.LabelFrame(parent, text="  ↻  Rotation  ")
        f_rot.pack(fill=tk.X, padx=6, pady=4)
        r_rot = ttk.Frame(f_rot, style='Panel.TFrame')
        r_rot.pack(fill=tk.X, padx=8, pady=8)
        ttk.Label(r_rot, text="Angle (°):").pack(side=tk.LEFT)
        self.man_angle = ttk.Entry(r_rot, width=7)
        self.man_angle.pack(side=tk.LEFT, padx=6)
        self.man_angle.insert(0, "0")
        ttk.Button(r_rot, text="↻  Tourner",
                   command=self._do_rotation,
                   style='Accent.TButton').pack(side=tk.LEFT, padx=4)

    # ---------- TAB AUTO ----------
    def _build_tab_auto(self, parent):
        canvas_scroll = tk.Canvas(parent, bg=BG_PANEL, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient=tk.VERTICAL,
                                  command=canvas_scroll.yview)
        scroll_frame = ttk.Frame(canvas_scroll, style='Panel.TFrame')

        scroll_frame.bind("<Configure>",
                          lambda e: canvas_scroll.configure(
                              scrollregion=canvas_scroll.bbox("all")))
        canvas_scroll.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas_scroll.configure(yscrollcommand=scrollbar.set)

        canvas_scroll.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Init
        f0 = ttk.LabelFrame(scroll_frame, text="  🏠  Initialisation  ")
        f0.pack(fill=tk.X, padx=6, pady=(6, 4))
        ttk.Button(f0, text="🏠   HOMING (Init)",
                   command=self._do_homing,
                   style='Big.TButton').pack(fill=tk.X, padx=8, pady=8)

        # Fardeau
        f1 = ttk.LabelFrame(scroll_frame, text="  📦  Fardeau (attraper)  ")
        f1.pack(fill=tk.X, padx=6, pady=4)

        r = ttk.Frame(f1, style='Panel.TFrame')
        r.pack(fill=tk.X, padx=8, pady=(8, 2))
        self.auto_entries = {}
        for lbl, default, key in [("X:", "0.10", "fx"),
                                  ("Y:", "0.10", "fy"),
                                  ("Z:", "0.50", "fz")]:
            ttk.Label(r, text=lbl).pack(side=tk.LEFT)
            e = ttk.Entry(r, width=7)
            e.pack(side=tk.LEFT, padx=3)
            e.insert(0, default)
            self.auto_entries[key] = e

        r2 = ttk.Frame(f1, style='Panel.TFrame')
        r2.pack(fill=tk.X, padx=8, pady=2)
        for lbl, default, key in [("L:", "0.05", "fl"),
                                  ("l:", "0.05", "fw"),
                                  ("h:", "0.03", "fh")]:
            ttk.Label(r2, text=lbl).pack(side=tk.LEFT)
            e = ttk.Entry(r2, width=6)
            e.pack(side=tk.LEFT, padx=3)
            e.insert(0, default)
            self.auto_entries[key] = e

        r3 = ttk.Frame(f1, style='Panel.TFrame')
        r3.pack(fill=tk.X, padx=8, pady=(2, 8))
        ttk.Label(r3, text="Angle attrapage (°):").pack(side=tk.LEFT)
        self.auto_entries['angle_pick'] = ttk.Entry(r3, width=7)
        self.auto_entries['angle_pick'].pack(side=tk.LEFT, padx=4)
        self.auto_entries['angle_pick'].insert(0, "0")

        # Livraison
        f2 = ttk.LabelFrame(scroll_frame, text="  📍  Point de livraison  ")
        f2.pack(fill=tk.X, padx=6, pady=4)

        r = ttk.Frame(f2, style='Panel.TFrame')
        r.pack(fill=tk.X, padx=8, pady=(8, 2))
        for lbl, default, key in [("X:", "0.50", "lx"),
                                  ("Y:", "0.25", "ly"),
                                  ("Z:", "0.40", "lz")]:
            ttk.Label(r, text=lbl).pack(side=tk.LEFT)
            e = ttk.Entry(r, width=7)
            e.pack(side=tk.LEFT, padx=3)
            e.insert(0, default)
            self.auto_entries[key] = e

        r2 = ttk.Frame(f2, style='Panel.TFrame')
        r2.pack(fill=tk.X, padx=8, pady=(2, 8))
        ttk.Label(r2, text="Angle livraison (°):").pack(side=tk.LEFT)
        self.auto_entries['angle_liv'] = ttk.Entry(r2, width=7)
        self.auto_entries['angle_liv'].pack(side=tk.LEFT, padx=4)
        self.auto_entries['angle_liv'].insert(0, "90")

        # Obstacles
        f3 = ttk.LabelFrame(scroll_frame, text="  🚧  Obstacles  ")
        f3.pack(fill=tk.X, padx=6, pady=4)

        r = ttk.Frame(f3, style='Panel.TFrame')
        r.pack(fill=tk.X, padx=8, pady=(8, 2))
        self.obs_fields = {}
        for lbl, default, key in [("X:", "0.30", "ox"), ("Y:", "0.10", "oy")]:
            ttk.Label(r, text=lbl).pack(side=tk.LEFT)
            e = ttk.Entry(r, width=6)
            e.pack(side=tk.LEFT, padx=2)
            e.insert(0, default)
            self.obs_fields[key] = e

        r2 = ttk.Frame(f3, style='Panel.TFrame')
        r2.pack(fill=tk.X, padx=8, pady=2)
        for lbl, default, key in [("L:", "0.10", "ol"), ("l:", "0.10", "ow")]:
            ttk.Label(r2, text=lbl).pack(side=tk.LEFT)
            e = ttk.Entry(r2, width=6)
            e.pack(side=tk.LEFT, padx=2)
            e.insert(0, default)
            self.obs_fields[key] = e

        r3 = ttk.Frame(f3, style='Panel.TFrame')
        r3.pack(fill=tk.X, padx=8, pady=2)
        for lbl, default, key in [("Zmin:", "0.0", "ozmin"),
                                  ("Zmax:", "0.30", "ozmax")]:
            ttk.Label(r3, text=lbl).pack(side=tk.LEFT)
            e = ttk.Entry(r3, width=6)
            e.pack(side=tk.LEFT, padx=2)
            e.insert(0, default)
            self.obs_fields[key] = e

        r4 = ttk.Frame(f3, style='Panel.TFrame')
        r4.pack(fill=tk.X, padx=8, pady=(4, 4))
        ttk.Button(r4, text="+  Ajouter",
                   command=self._add_obs,
                   style='Accent.TButton').pack(side=tk.LEFT, padx=2)
        ttk.Button(r4, text="−  Supprimer",
                   command=self._del_obs).pack(side=tk.LEFT, padx=2)
        ttk.Button(r4, text="✕  Vider",
                   command=self._clear_obs,
                   style='Danger.TButton').pack(side=tk.LEFT, padx=2)

        self.obs_list = tk.Listbox(f3, height=4,
                                   bg=BG_INPUT, fg=DANGER,
                                   font=('Consolas', 8),
                                   selectbackground=DANGER,
                                   selectforeground='white',
                                   borderwidth=1, relief=tk.SOLID,
                                   highlightthickness=0)
        self.obs_list.pack(fill=tk.X, padx=8, pady=(4, 8))

        # Mission
        f4 = ttk.LabelFrame(scroll_frame, text="  🚀  Mission  ")
        f4.pack(fill=tk.X, padx=6, pady=4)

        ttk.Button(f4, text="👁  Aperçu chemin",
                   command=self._calculer_chemin_apercu,
                   style='Accent.TButton').pack(
            fill=tk.X, padx=8, pady=(8, 4))

        self.btn_mission = ttk.Button(f4, text="▶   LANCER MISSION",
                                      command=self._launch_mission,
                                      style='Big.TButton')
        self.btn_mission.pack(fill=tk.X, padx=8, pady=4)

        self.status_var = tk.StringVar(value="En attente")
        ttk.Label(f4, textvariable=self.status_var,
                  style='PanelSubtle.TLabel',
                  font=('Segoe UI', 9, 'italic')).pack(padx=8, pady=(2, 8))

    # ============ CANVAS ============

    def _m2px(self, xm, ym):
        return int(xm * SCALE), CANVAS_H - int(ym * SCALE)

    def _draw_canvas(self):
        c = self.canvas
        c.delete("all")

        # Grille (light theme)
        for i in range(0, int(WORKSPACE_X * 100) + 1, 5):
            x = int(i / 100.0 * SCALE)
            col = GRID_MINOR if i % 10 else GRID_MAJOR
            c.create_line(x, 0, x, CANVAS_H, fill=col)
        for i in range(0, int(WORKSPACE_Y * 100) + 1, 5):
            y = CANVAS_H - int(i / 100.0 * SCALE)
            col = GRID_MINOR if i % 10 else GRID_MAJOR
            c.create_line(0, y, CANVAS_W, y, fill=col)

        # Labels axes (cm)
        for i in range(0, int(WORKSPACE_X * 100) + 1, 10):
            c.create_text(int(i / 100.0 * SCALE), CANVAS_H - 5, text=str(i),
                          fill=FG_MUTED, font=('Consolas', 7), anchor=tk.S)
        for i in range(0, int(WORKSPACE_Y * 100) + 1, 10):
            c.create_text(5, CANVAS_H - int(i / 100.0 * SCALE), text=str(i),
                          fill=FG_MUTED, font=('Consolas', 7), anchor=tk.W)

        # Obstacles (rouge clair sur fond clair)
        for o in self.obstacles:
            x1, y1 = self._m2px(o['x'], o['y'] + o['w'])
            x2, y2 = self._m2px(o['x'] + o['l'], o['y'])
            c.create_rectangle(x1, y1, x2, y2,
                               fill='#fee2e2', outline=DANGER, width=2)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            c.create_text(cx, cy, text=f"Z:{o['z_max']:.2f}",
                          fill=DANGER, font=('Consolas', 8, 'bold'))

        # Fardeau (vert clair)
        try:
            fx = float(self.auto_entries['fx'].get())
            fy = float(self.auto_entries['fy'].get())
            fl = float(self.auto_entries['fl'].get())
            fw = float(self.auto_entries['fw'].get())
            p1x, p1y = self._m2px(fx - fl / 2, fy + fw / 2)
            p2x, p2y = self._m2px(fx + fl / 2, fy - fw / 2)
            c.create_rectangle(p1x, p1y, p2x, p2y,
                               fill='#d1fae5', outline=SUCCESS, width=2)
            c.create_text((p1x + p2x) // 2, (p1y + p2y) // 2 - 12,
                          text="FARDEAU",
                          fill=SUCCESS, font=('Segoe UI', 8, 'bold'))
        except (ValueError, KeyError):
            pass

        # Livraison (violet clair)
        try:
            lx = float(self.auto_entries['lx'].get())
            ly = float(self.auto_entries['ly'].get())
            px, py = self._m2px(lx, ly)
            c.create_polygon(px, py - 12, px - 10, py + 8, px + 10, py + 8,
                             fill='#ede9fe', outline=INFO, width=2)
            c.create_text(px, py + 20, text="LIVRAISON",
                          fill=INFO, font=('Segoe UI', 8, 'bold'))
        except (ValueError, KeyError):
            pass

        # Chemins planifiés (aperçu) — ordre : aller, retour livraison, retour origine
        self._tracer_chemin(c, self.chemin_aller,   PATH_GO,     "A")
        self._tracer_chemin(c, self.chemin_retour,  PATH_RETURN, "R")
        self._tracer_chemin(c, self.chemin_origine, PATH_HOME,   "H")

        # Pont (orange)
        ppx, ppy = self.pont_pos
        if ppx > 0 or ppy > 0:
            c.create_oval(ppx - 8, ppy - 8, ppx + 8, ppy + 8,
                          fill=PONT_COLOR, outline=PONT_OUTLINE, width=2)

    def _tracer_chemin(self, c, chemin, couleur, prefix):
        """
        Trace un chemin (liste de waypoints) sur le canvas en pointillés,
        avec un petit cercle pour chaque waypoint intermédiaire.
        """
        if not chemin or len(chemin) < 2:
            return
        for i in range(len(chemin) - 1):
            x1m, y1m = chemin[i]
            x2m, y2m = chemin[i + 1]
            x1, y1 = self._m2px(x1m, y1m)
            x2, y2 = self._m2px(x2m, y2m)
            c.create_line(x1, y1, x2, y2, fill=couleur, width=2, dash=(6, 4))
        # Waypoints intermédiaires (pas A ni B)
        for i, (xm, ym) in enumerate(chemin[1:-1], start=1):
            px, py = self._m2px(xm, ym)
            c.create_oval(px - 5, py - 5, px + 5, py + 5,
                          fill=couleur, outline=BG_PANEL, width=2)
            c.create_text(px + 9, py - 9, text=f"{prefix}{i}",
                          fill=couleur,
                          font=('Consolas', 8, 'bold'))

    # ============ ACTIONS COMMUNES ============

    def _refresh_ports(self):
        ports = self.serial_mgr.lister_ports()
        self.port_cb['values'] = ports
        if ports:
            self.port_cb.set(ports[0])

    def _toggle_conn(self):
        if self.serial_mgr.est_connecte():
            self.serial_mgr.deconnecter()
            self.btn_conn.config(text="Connecter")
        else:
            port = self.port_var.get()
            if port and self.serial_mgr.connecter(port):
                self.btn_conn.config(text="Déconnecter")

    def _check_busy(self):
        """[FIX #5] Log explicite si une action est ignorée car busy."""
        if self.busy:
            self.log_msg("⚠ Action ignorée : une commande est déjà en cours.")
            return True
        return False

    def _do_homing(self):
        if self._check_busy():
            return
        if not self.serial_mgr.est_connecte():
            self.log_msg("✗ Connectez d'abord le port série.")
            return

        def t():
            # [FIX #7] try/finally pour garantir le déblocage de busy
            try:
                self.busy = True
                self.status_var.set("Homing en cours...")
                self.serial_mgr.envoyer("init")
                if self.serial_mgr.attendre_message("ORIGINE", 90):
                    self.status_var.set("Homing OK ✓")
                    self.pont_pos = self._m2px(0, 0)
                    self.root.after(0, self._draw_canvas)
                else:
                    self.status_var.set("Homing échoué ✗")
            finally:
                self.busy = False
        threading.Thread(target=t, daemon=True).start()

    # ============ ACTIONS MODE MANUEL ============

    def _do_move(self):
        if self._check_busy():
            return
        if not self.serial_mgr.est_connecte():
            self.log_msg("✗ Connectez d'abord le port série.")
            return
        try:
            x = float(self.man_x.get())
            y = float(self.man_y.get())
            z = float(self.man_z.get())
            v = float(self.man_v.get())
            a = float(self.man_a.get())
        except ValueError:
            messagebox.showerror("Erreur", "Valeurs invalides.")
            return

        def t():
            try:
                self.busy = True
                self.serial_mgr.envoyer(f"{x} {y} {z} {v} {a}")
                if self.serial_mgr.attendre_message("ARRIVE", 120):
                    self.pont_pos = self._m2px(x, y)
                    self.root.after(0, self._draw_canvas)
            finally:
                self.busy = False
        threading.Thread(target=t, daemon=True).start()

    def _do_cmd(self, cmd, wait_for):
        if self._check_busy():
            return
        if not self.serial_mgr.est_connecte():
            self.log_msg("✗ Connectez d'abord le port série.")
            return

        def t():
            try:
                self.busy = True
                self.serial_mgr.envoyer(cmd)
                self.serial_mgr.attendre_message(wait_for, 5)
                time.sleep(0.3)
            finally:
                self.busy = False
        threading.Thread(target=t, daemon=True).start()

    def _do_rotation(self):
        if self._check_busy():
            return
        if not self.serial_mgr.est_connecte():
            self.log_msg("✗ Connectez d'abord le port série.")
            return
        try:
            angle = float(self.man_angle.get())
        except ValueError:
            return

        def t():
            try:
                self.busy = True
                self.serial_mgr.envoyer(f"rot {angle}")
                self.serial_mgr.attendre_message("ROT_OK", 10)
                time.sleep(1)
            finally:
                self.busy = False
        threading.Thread(target=t, daemon=True).start()

    # ============ ACTIONS MODE AUTO ============

    def _add_obs(self):
        try:
            o = {
                'x': float(self.obs_fields['ox'].get()),
                'y': float(self.obs_fields['oy'].get()),
                'l': float(self.obs_fields['ol'].get()),
                'w': float(self.obs_fields['ow'].get()),
                'z_min': float(self.obs_fields['ozmin'].get()),
                'z_max': float(self.obs_fields['ozmax'].get()),
            }
            self.obstacles.append(o)
            self.obs_list.insert(tk.END,
                f"({o['x']:.2f},{o['y']:.2f}) {o['l']:.2f}×{o['w']:.2f} Z:[{o['z_min']:.2f}-{o['z_max']:.2f}]")
            # Les obstacles ont changé : chemins précédents obsolètes
            self.chemin_aller = None
            self.chemin_retour = None
            self.chemin_origine = None
            self._draw_canvas()
        except ValueError:
            messagebox.showerror("Erreur", "Valeurs invalides.")

    def _del_obs(self):
        sel = self.obs_list.curselection()
        if sel:
            self.obstacles.pop(sel[0])
            self.obs_list.delete(sel[0])
            self.chemin_aller = None
            self.chemin_retour = None
            self.chemin_origine = None
            self._draw_canvas()

    def _clear_obs(self):
        self.obstacles.clear()
        self.obs_list.delete(0, tk.END)
        self.chemin_aller = None
        self.chemin_retour = None
        self.chemin_origine = None
        self._draw_canvas()

    def _update_pont(self, x, y):
        self.pont_pos = self._m2px(x, y)
        self.root.after(0, self._draw_canvas)

    def _lire_params_auto(self):
        """
        Lit les paramètres fardeau/livraison/angles depuis les champs UI.
        Retourne (fardeau, livraison, angle_pick, angle_liv) ou None si invalide.
        """
        try:
            fardeau = {
                'x': float(self.auto_entries['fx'].get()),
                'y': float(self.auto_entries['fy'].get()),
                'z': float(self.auto_entries['fz'].get()),
            }
            livraison = {
                'x': float(self.auto_entries['lx'].get()),
                'y': float(self.auto_entries['ly'].get()),
                'z': float(self.auto_entries['lz'].get()),
            }
            angle_pick = float(self.auto_entries['angle_pick'].get())
            angle_liv = float(self.auto_entries['angle_liv'].get())
            return fardeau, livraison, angle_pick, angle_liv
        except ValueError:
            return None

    def _calculer_chemin_apercu(self):
        """
        Calcule les chemins aller / retour livraison / retour origine planifiés
        sans lancer la mission. Affiche le résultat sur le canvas et dans le journal.
        """
        params = self._lire_params_auto()
        if params is None:
            messagebox.showerror("Erreur", "Valeurs fardeau/livraison invalides.")
            return
        fardeau, livraison, _, _ = params

        # Validation préliminaire
        ok, msg = self.mission._validation_preliminaire(fardeau, livraison, self.obstacles)
        if not ok:
            self.log_msg(f"⚠ Aperçu impossible : {msg}")
            messagebox.showerror("Aperçu impossible", msg)
            self.chemin_aller = None
            self.chemin_retour = None
            self.chemin_origine = None
            self._draw_canvas()
            return

        # Planification
        A_dep = (0.0, 0.0)
        A_far = (fardeau['x'], fardeau['y'])
        A_liv = (livraison['x'], livraison['y'])

        self.chemin_aller   = self.mission._planifier_chemin_xy(A_dep, A_far, self.obstacles)
        self.chemin_retour  = self.mission._planifier_chemin_xy(A_far, A_liv, self.obstacles)
        self.chemin_origine = self.mission._planifier_chemin_xy(A_liv, A_dep, self.obstacles)

        # Diagnostic
        if self.chemin_aller is None:
            self.log_msg("⚠ Aperçu : aucun chemin trouvé départ → fardeau")
            messagebox.showerror("Chemin impossible",
                                 "Aucun chemin trouvé entre le départ et le fardeau.\n"
                                 "Obstacles infranchissables.")
        elif self.chemin_retour is None:
            self.log_msg("⚠ Aperçu : aucun chemin trouvé fardeau → livraison")
            messagebox.showerror("Chemin impossible",
                                 "Aucun chemin trouvé entre le fardeau et la livraison.\n"
                                 "Obstacles infranchissables.")
        else:
            # Fallback retour origine si planification échoue (improbable mais sécurisé)
            if self.chemin_origine is None:
                self.log_msg("⚠ Aperçu : aucun chemin trouvé livraison → origine — fallback : aller inversé")
                self.chemin_origine = list(reversed(self.chemin_aller))

            n_a = len(self.chemin_aller) - 1
            n_r = len(self.chemin_retour) - 1
            n_o = len(self.chemin_origine) - 1
            self.log_msg(f"✓ Aperçu : aller {n_a} segment(s), retour {n_r} segment(s), retour-origine {n_o} segment(s)")

        self._draw_canvas()

    def _launch_mission(self):
        if self._check_busy():
            return
        if not self.serial_mgr.est_connecte():
            messagebox.showwarning("Erreur", "Non connecté.")
            return

        params = self._lire_params_auto()
        if params is None:
            messagebox.showerror("Erreur", "Valeurs invalides.")
            return
        fardeau, livraison, angle_pick, angle_liv = params

        # Validation + planification AVANT démarrage du thread mission
        ok, msg = self.mission._validation_preliminaire(fardeau, livraison, self.obstacles)
        if not ok:
            self.log_msg(f"⚠ Validation : {msg}")
            messagebox.showerror("Validation échouée", msg)
            return

        A_dep = (0.0, 0.0)
        A_far = (fardeau['x'], fardeau['y'])
        A_liv = (livraison['x'], livraison['y'])
        self.chemin_aller   = self.mission._planifier_chemin_xy(A_dep, A_far, self.obstacles)
        self.chemin_retour  = self.mission._planifier_chemin_xy(A_far, A_liv, self.obstacles)
        self.chemin_origine = self.mission._planifier_chemin_xy(A_liv, A_dep, self.obstacles)

        if self.chemin_aller is None:
            messagebox.showerror("Chemin impossible",
                                 "Aucun chemin trouvé : départ → fardeau.\n"
                                 "Vérifiez la position des obstacles.")
            self._draw_canvas()
            return
        if self.chemin_retour is None:
            messagebox.showerror("Chemin impossible",
                                 "Aucun chemin trouvé : fardeau → livraison.\n"
                                 "Vérifiez la position des obstacles.")
            self._draw_canvas()
            return
        # Fallback automatique pour le retour origine
        if self.chemin_origine is None:
            self.log_msg("⚠ Aucun chemin direct livraison → origine — fallback : inversion du chemin aller")
            self.chemin_origine = list(reversed(self.chemin_aller))

        # Affichage du chemin avant lancement
        self._draw_canvas()

        chemin_a = self.chemin_aller
        chemin_r = self.chemin_retour
        chemin_o = self.chemin_origine

        def t():
            try:
                self.busy = True
                self.status_var.set("Mission en cours...")
                self.root.after(0, lambda: self.btn_mission.config(state=tk.DISABLED))
                ok = self.mission.executer(fardeau, livraison, self.obstacles,
                                           angle_pick, angle_liv,
                                           chemin_aller=chemin_a,
                                           chemin_retour=chemin_r,
                                           chemin_origine=chemin_o,
                                           cb_pos=self._update_pont)
                self.status_var.set("Mission réussie ✓" if ok else "Mission échouée ✗")
            finally:
                self.busy = False
                self.root.after(0, lambda: self.btn_mission.config(state=tk.NORMAL))
        threading.Thread(target=t, daemon=True).start()

    # ============ LOG ============

    def log_msg(self, msg):
        def _a():
            self.log_text.insert(tk.END, msg + "\n")
            self.log_text.see(tk.END)
        try:
            self.root.after(0, _a)
        except Exception:
            pass


# ================== MAIN ======================================

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry(f"{CANVAS_W + 390}x{CANVAS_H + 280}")
    root.minsize(1000, 700)
    app = PontRoulantGUI(root)
    root.mainloop()
