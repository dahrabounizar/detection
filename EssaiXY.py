import cv2
import time
from ultralytics import YOLO

# Fonctions de stereo vision
import triangulation as tri
import calibration


# =========================
# CHARGEMENT DES MODELES YOLO
# =========================

model_best2     = YOLO(r"C:/Users/SARA GHERRAS/Desktop/lib/best (2).pt")       # mytruck / fardeaux
model_best1     = YOLO(r"C:/Users/SARA GHERRAS/Desktop/lib/best (1).pt")       # person
model_best3     = YOLO(r"C:/Users/SARA GHERRAS/Desktop/lib/best (3) (1).pt")   # cone
model_best4     = YOLO(r"C:/Users/SARA GHERRAS/Desktop/lib/best (5).pt")       # Box
model_obstacles = YOLO("yolov8n.pt")                                          # obstacles généraux

FARDEAU_MIN_CONF = 0.04
CONE_MIN_CONF = 0.10
TRUCK_MIN_ASPECT_RATIO = 1.15
CONE_MAX_ASPECT_RATIO = 1.45

ALLOWED_CLASSES = {
    "truck",
    "mytruck",
    "fardeau",
    "fardeaux",
    "plot",
    "cone",
}

DISPLAY_CLASS_ALIASES = {
    "mytruck": "truck",
    "plot": "cone",
}

MATCH_CLASS_ALIASES = {
    "mytruck": "truck",
    "plot": "cone",
    "cone": "cone",
    "fardeau": "fardeaux",
    "fardeaux": "fardeaux",
}


def normalize_class_name(class_name):
    return class_name.strip().lower()


def get_display_class_name(class_name):
    class_name_key = normalize_class_name(class_name)

    return DISPLAY_CLASS_ALIASES.get(class_name_key, class_name_key)


def get_match_class_name(class_name):
    class_name_key = normalize_class_name(class_name)

    return MATCH_CLASS_ALIASES.get(class_name_key, get_display_class_name(class_name_key))


def calculate_box_aspect_ratio(box):
    x1, y1, x2, y2 = box
    width = max(x2 - x1, 1)
    height = max(y2 - y1, 1)

    return width / height


def is_shape_valid_for_class(match_class_name, box):
    aspect_ratio = calculate_box_aspect_ratio(box)

    if match_class_name == "truck":
        return aspect_ratio >= TRUCK_MIN_ASPECT_RATIO

    if match_class_name == "cone":
        return aspect_ratio <= CONE_MAX_ASPECT_RATIO

    return True

models = [
    {
        "name": "person_custom",
        "model": model_best1,
        "conf": 0.50,
    },
    {
        "name": "fardeaux_truck_custom",
        "model": model_best2,
        "conf": 0.30,
        "class_conf": {
            "fardeau": FARDEAU_MIN_CONF,
            "fardeaux": FARDEAU_MIN_CONF,
        },
    },
    {
        "name": "cone_custom",
        "model": model_best3,
        "conf": CONE_MIN_CONF,
    },
    {
        "name": "box_custom",
        "model": model_best4,
        "conf": 0.40,
    },
    {
        "name": "coco_obstacles",
        "model": model_obstacles,
        "conf": 0.50,
        "exclude_classes": {
            "frisbee",
        },
    },
]

COEXISTING_CLASS_PAIRS = {
    frozenset(("truck", "fardeaux")),
}


# =========================
# PARAMETRES STEREO
# =========================

frame_rate = 30
B = 7          # distance entre les deux caméras en cm
f = 3.6        # focale en mm

# Calibration empirique de la distance:
# si 90 cm reels sont affiches comme 50 cm, le facteur est 90 / 50.
KNOWN_DISTANCE_CM = 90.0
DISPLAYED_DISTANCE_CM = 50.0
DISTANCE_SCALE = KNOWN_DISTANCE_CM / DISPLAYED_DISTANCE_CM


def calculate_xy_from_depth(center_point, depth, intrinsics):
    """Calcule X et Y dans EssaiXY.py a partir du pixel (u, v) et de Z."""
    u, v = center_point
    fx = intrinsics["fx"]
    fy = intrinsics["fy"]
    cx = intrinsics["cx"]
    cy = intrinsics["cy"]

    x = ((u - cx) * depth) / fx
    y = ((v - cy) * depth) / fy

    return x, y


def calculate_xprime_yprime(x_coord, y_coord):
    """Calcule Xprime et Yprime a partir de X et Y."""
    x_prime = 47.5 + y_coord
    y_prime = 17.5 + x_coord

    return x_prime, y_prime


def draw_xprime_yprime(frame, x_prime, y_prime, position):
    """Affiche Xprime et Yprime sur une image."""
    text_xprime_yprime = f"X_reel={x_prime:.1f} cm, Y_reel={y_prime:.1f} cm"

    cv2.putText(
        frame,
        text_xprime_yprime,
        position,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2
    )


def get_raw_camera_intrinsics(side, frame, alpha_degrees):
    """
    Recupere les intrinseques adaptees aux images brutes.

    Dans ce fichier, les images ne sont pas rectifiees, donc il faut utiliser
    cameraMatrixR/L et non projMatrixR/L.
    """
    side = side.lower()
    suffix = "R" if side == "right" else "L"
    matrix = calibration.calibration_data.get(f"cameraMatrix{suffix}")

    if matrix is not None and matrix.shape[0] >= 3 and matrix.shape[1] >= 3:
        return {
            "fx": float(matrix[0, 0]),
            "fy": float(matrix[1, 1]),
            "cx": float(matrix[0, 2]),
            "cy": float(matrix[1, 2]),
            "source": f"cameraMatrix{suffix}",
        }

    return calibration.get_camera_intrinsics(
        side=side,
        frame=frame,
        alpha_degrees=alpha_degrees
    )


alpha = 84     # champ de vision horizontal en degrés


# =========================
# OUVERTURE DES CAMERAS
# =========================

cap_right = cv2.VideoCapture(2, cv2.CAP_DSHOW)
cap_left  = cv2.VideoCapture(0, cv2.CAP_DSHOW)

cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


# =========================
# FONCTION DE CALCUL D'IoU
# =========================

def calculate_iou(box1, box2):
    """Calcule l'Intersection over Union entre deux boîtes englobantes"""
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2
    
    # Intersection
    inter_xmin = max(x1_min, x2_min)
    inter_ymin = max(y1_min, y2_min)
    inter_xmax = min(x1_max, x2_max)
    inter_ymax = min(y1_max, y2_max)
    
    if inter_xmax < inter_xmin or inter_ymax < inter_ymin:
        return 0.0
    
    inter_area = (inter_xmax - inter_xmin) * (inter_ymax - inter_ymin)
    
    # Union
    box1_area = (x1_max - x1_min) * (y1_max - y1_min)
    box2_area = (x2_max - x2_min) * (y2_max - y2_min)
    union_area = box1_area + box2_area - inter_area
    
    if union_area == 0:
        return 0.0
    
    return inter_area / union_area


def calculate_overlap_ratio(box1, box2):
    """Calcule la part de la plus petite boite couverte par l'intersection."""
    x1_min, y1_min, x1_max, y1_max = box1
    x2_min, y2_min, x2_max, y2_max = box2

    inter_xmin = max(x1_min, x2_min)
    inter_ymin = max(y1_min, y2_min)
    inter_xmax = min(x1_max, x2_max)
    inter_ymax = min(y1_max, y2_max)

    if inter_xmax <= inter_xmin or inter_ymax <= inter_ymin:
        return 0.0

    inter_area = (inter_xmax - inter_xmin) * (inter_ymax - inter_ymin)
    box1_area = (x1_max - x1_min) * (y1_max - y1_min)
    box2_area = (x2_max - x2_min) * (y2_max - y2_min)
    min_area = min(box1_area, box2_area)

    if min_area <= 0:
        return 0.0

    return inter_area / min_area


def can_classes_coexist(det1, det2):
    class_pair = frozenset((
        det1["match_class_name"],
        det2["match_class_name"]
    ))

    return class_pair in COEXISTING_CLASS_PAIRS


# =========================
# FONCTION DE FILTRAGE PAR SCORE
# =========================

def filter_best_detections(detections, iou_threshold=0.25, overlap_threshold=0.60):
    """
    Filtre les détections en gardant seulement celle avec le meilleur score
    pour chaque objet (boîtes qui se chevauchent).
    """
    if not detections:
        return detections
    
    # Trier par score: la forme des boites filtre deja les faux truck/cone.
    detections_sorted = sorted(detections, key=lambda d: d["conf"], reverse=True)
    
    filtered = []
    used = set()
    
    for i, det in enumerate(detections_sorted):
        if i in used:
            continue
        
        # Marquer toutes les détections similaires comme utilisées
        for j in range(i + 1, len(detections_sorted)):
            if j in used:
                continue
            
            other = detections_sorted[j]
            iou = calculate_iou(det["box"], other["box"])
            overlap = calculate_overlap_ratio(det["box"], other["box"])
            
            if (
                (iou > iou_threshold or overlap > overlap_threshold)
                and not can_classes_coexist(det, other)
            ):
                used.add(j)
        
        filtered.append(det)
    
    return filtered


# =========================
# FONCTION DE DETECTION YOLO
# =========================

def detect_objects(frame, model_configs):
    detections = []

    for config in model_configs:
        model = config["model"]
        conf_threshold = config["conf"]
        class_conf = {
            normalize_class_name(class_name): threshold
            for class_name, threshold in config.get("class_conf", {}).items()
        }
        exclude_classes = {
            normalize_class_name(class_name)
            for class_name in config.get("exclude_classes", set())
        }
        model_conf = min([conf_threshold] + list(class_conf.values()))
        results = model(frame, conf=model_conf, verbose=False)[0]

        if results.boxes is None:
            continue

        for box in results.boxes:
            conf = float(box.conf[0])

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])

            raw_class_name = model.names[cls_id] if cls_id in model.names else "object"
            class_name_key = normalize_class_name(raw_class_name)
            display_class_name = get_display_class_name(class_name_key)
            match_class_name = get_match_class_name(class_name_key)

            if class_name_key not in ALLOWED_CLASSES:
                continue

            if class_name_key in exclude_classes:
                continue

            detection_box = (x1, y1, x2, y2)

            if not is_shape_valid_for_class(match_class_name, detection_box):
                continue

            threshold = class_conf.get(class_name_key, conf_threshold)

            if class_name_key in class_conf:
                if conf < threshold:
                    continue
            elif conf < threshold:
                continue

            center = (
                int((x1 + x2) / 2),
                int((y1 + y2) / 2)
            )

            detections.append({
                "box": detection_box,
                "center": center,
                "conf": conf,
                "class_name": display_class_name,
                "raw_class_name": raw_class_name,
                "match_class_name": match_class_name,
                "model_name": config["name"],
            })

    # Filtrer et garder seulement le meilleur score pour chaque objet
    detections = filter_best_detections(detections)
    
    return detections


# =========================
# FONCTION POUR DESSINER
# =========================

def draw_detections(frame, detections):
    for det in detections:
        x1, y1, x2, y2 = det["box"]
        cx, cy = det["center"]
        conf = det["conf"]
        class_name = det["class_name"]

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        label = f"{class_name} {conf:.2f}"
        cv2.putText(
            frame,
            label,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )


# =========================
# FONCTION POUR CHOISIR L'OBJET PRINCIPAL
# =========================

def get_main_detection(detections):
    if len(detections) == 0:
        return None

    # On prend l'objet avec la plus grande boîte
    detections = sorted(
        detections,
        key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]),
        reverse=True
    )

    return detections[0]


# =========================
# BOUCLE PRINCIPALE
# =========================

while cap_right.isOpened() and cap_left.isOpened():

    success_right, frame_right = cap_right.read()
    success_left, frame_left = cap_left.read()

    if not success_right or not success_left:
        print("Erreur : impossible de lire une des deux caméras.")
        break

    start = time.time()

    # Calibration / rectification
    # frame_right, frame_left = calibration.undistortRectify(frame_right, frame_left)

    # Détection YOLO
    detections_right = detect_objects(frame_right, models)
    detections_left  = detect_objects(frame_left, models)

    # Dessiner les détections
    draw_detections(frame_right, detections_right)
    draw_detections(frame_left, detections_left)

    # Choisir l'objet principal dans chaque caméra
    main_right = get_main_detection(detections_right)
    main_left  = get_main_detection(detections_left)

    if len(detections_right) == 0 or len(detections_left) == 0:
        cv2.putText(
            frame_right,
            "TRACKING LOST",
            (75, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

        cv2.putText(
            frame_left,
            "TRACKING LOST",
            (75, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    else:
        # Calculer la distance pour chaque objet détecté
        y_offset = 50
        
        for det_right in detections_right:
            center_point_right = det_right["center"]
            
            # Chercher l'objet correspondant dans l'image de gauche
            # Critères : même classe ET position X proche
            best_match = None
            min_score = float('inf')
            
            for det_left in detections_left:
                # Privilégier les appariements avec la même classe
                class_match = 0 if det_right["match_class_name"] == det_left["match_class_name"] else 100
                
                # Distance en X (max 50 pixels)
                x_diff = abs(center_point_right[0] - det_left["center"][0])
                # Close objects have larger disparity; cap X only for matching.
                x_diff = min(x_diff, 80)
                y_diff = abs(center_point_right[1] - det_left["center"][1])
                if y_diff > 60:
                    continue
                if x_diff > 80:  # Augmenter la tolérance pour images non rectifiées
                    continue
                
                # Score combiné : classe + position
                score = class_match + y_diff + (0.25 * x_diff)
                
                if score < min_score:
                    min_score = score
                    best_match = det_left
            
            if best_match is not None:
                center_point_left = best_match["center"]
                
                intrinsics_right = get_raw_camera_intrinsics(
                    side="right",
                    frame=frame_right,
                    alpha_degrees=alpha
                )

                depth = tri.find_depth(
                    center_point_right,
                    center_point_left,
                    frame_right,
                    frame_left,
                    B,
                    f,
                    alpha,
                    DISTANCE_SCALE,
                    intrinsics=intrinsics_right
                )
                
                # Afficher seulement si la profondeur est valide.
                if depth != float('inf'):
                    x_coord, y_coord = calculate_xy_from_depth(
                        center_point_right,
                        depth,
                        intrinsics_right
                    )
                    x_prime, y_prime = calculate_xprime_yprime(x_coord, y_coord)

                    text_depth = f"{det_right['class_name']}: Z={depth:.1f} cm"
                    text_xy = f"X={x_coord:.1f} cm, Y={y_coord:.1f} cm"
                    
                    # Afficher sur l'image droite
                    cv2.putText(
                        frame_right,
                        text_depth,
                        (50, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                    cv2.putText(
                        frame_right,
                        text_xy,
                        (50, y_offset + 22),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                    draw_xprime_yprime(
                        frame_right,
                        x_prime,
                        y_prime,
                        (50, y_offset + 44)
                    )
                    
                    # Afficher sur l'image gauche
                    cv2.putText(
                        frame_left,
                        text_depth,
                        (50, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                    cv2.putText(
                        frame_left,
                        text_xy,
                        (50, y_offset + 22),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )
                    draw_xprime_yprime(
                        frame_left,
                        x_prime,
                        y_prime,
                        (50, y_offset + 44)
                    )
                    
                    print(
                        f"{det_right['class_name']}: "
                        f"X={x_coord:.1f} cm, Y={y_coord:.1f} cm, Z={depth:.1f} cm, "
                        f"Xprime={x_prime:.1f} cm, Yprime={y_prime:.1f} cm"
                    )
                    
                    y_offset += 70

    # FPS
    end = time.time()
    total_time = end - start

    if total_time > 0:
        fps = 1 / total_time
    else:
        fps = 0

    cv2.putText(
        frame_right,
        f"FPS: {int(fps)}",
        (20, 450),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (0, 255, 0),
        2
    )

    cv2.putText(
        frame_left,
        f"FPS: {int(fps)}",
        (20, 450),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (0, 255, 0),
        2
    )

    # Affichage
    cv2.imshow("frame right", frame_right)
    cv2.imshow("frame left", frame_left)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


# =========================
# FERMETURE
# =========================

cap_right.release()
cap_left.release()
cv2.destroyAllWindows()
