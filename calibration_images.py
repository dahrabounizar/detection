import cv2
import os

# Vérifier que les dossiers existent
os.makedirs("images/stereoLeft", exist_ok=True)
os.makedirs("images/stereoRight", exist_ok=True)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap2 = cv2.VideoCapture(2, cv2.CAP_DSHOW)  # probablement 1, pas 2

if not cap.isOpened():
    print("Impossible d'ouvrir la caméra 0")
    exit()

if not cap2.isOpened():
    print("Impossible d'ouvrir la caméra 1")
    exit()

num = 0

while True:
    succes1, img = cap.read()
    succes2, img2 = cap2.read()

    if not succes1:
        print("Erreur capture caméra 0")
        break
    if not succes2:
        print("Erreur capture caméra 1")
        break

    cv2.imshow('Caméra 1', img)
    cv2.imshow('Caméra 2', img2)

    k = cv2.waitKey(5)
    if k == 27:  # ESC
        break
    elif k == ord('s'):
        cv2.imwrite(f'images/stereoLeft/imageL{num}.png', img)
        cv2.imwrite(f'images/stereoRight/imageR{num}.png', img2)
        print("Images enregistrées !")
        num += 1

cap.release()
cap2.release()
cv2.destroyAllWindows()
