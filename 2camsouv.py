import cv2

# Ouvre la caméra 0 et 1
cap1 = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap2 = cv2.VideoCapture(2, cv2.CAP_DSHOW)


if not cap1.isOpened() or not cap2.isOpened():
    print("Impossible d'ouvrir une ou plusieurs caméras.")
    exit()

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if not ret1 or not ret2:
        print("Erreur de lecture d'une caméra.")
        break

    # Redimensionner pour éviter une fenêtre trop grande
    frame1 = cv2.resize(frame1, (640, 480))
    frame2 = cv2.resize(frame2, (640, 480))

    # Concaténer horizontalement
    combined = cv2.hconcat([frame1, frame2])

    cv2.imshow("Deux Caméras", combined)

    # Quitter avec la touche Q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
