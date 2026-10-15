"""
Bildverarbeitung:
Bild aufnehmen -> entzerren -> Blöcke finden -> Weltkoordinaten
-> gültige Blöcke einzeln an den Arduino senden, jeweils auf DONE warten.

Textformat: Farbe,X,Y,Orientierung (Farbe: 0=rot, 1=grün, 2=blau)
"""

import time
import os
import cv2 as cv
import numpy as np
import serial

import welt_opt

CAMERA_ID = 0
WIDTH, HEIGHT = 1600, 1200
KALIBRIERUNG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calibration.npz")
WARMUP_FRAMES = 15
MIN_FLAECHE = 1500
KERNEL = np.ones((11, 11), np.uint8)

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
READY_TIMEOUT = 15.0
DONE_TIMEOUT = 300.0

FARB_IDS = {"rot": 0, "grün": 1, "blau": 2}

FARBBEREICHE = {
    "rot": [
        (np.array([170, 25, 0]), np.array([179, 255, 255])),
        (np.array([0, 25, 0]), np.array([10, 255, 255]))],
    "grün": [(np.array([40, 25, 0]), np.array([80, 255, 255]))],
    "blau": [(np.array([100, 25, 0]), np.array([130, 255, 255]))],
}


def bild_aufnehmen():
    cap = cv.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        raise RuntimeError("Kamera konnte nicht geöffnet werden.")
    cap.set(cv.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, HEIGHT)

    for _ in range(WARMUP_FRAMES):
        cap.read()
    ret, frame = cap.read()
    cap.release()

    if not ret:
        raise RuntimeError("Kein Bild von der Kamera erhalten.")
    if frame.shape[:2] != (HEIGHT, WIDTH):
        raise RuntimeError(f"Falsche Auflösung: {frame.shape[1]} x {frame.shape[0]}")
    return frame


def entzerren(frame):
    h, w = frame.shape[:2]
    daten = np.load(KALIBRIERUNG)
    K, D = daten["K"], daten["D"]

    new_K = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (w, h), np.eye(3), balance=1.0)
    map1, map2 = cv.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv.CV_16SC2)
    return cv.remap(frame, map1, map2, interpolation=cv.INTER_LINEAR)


def maske_bauen(hsv, bereiche):
    maske = np.zeros(hsv.shape[:2], np.uint8)
    for lower, upper in bereiche:
        maske |= cv.inRange(hsv, lower, upper)
    maske = cv.morphologyEx(maske, cv.MORPH_OPEN, KERNEL)    # kleine Störpunkte weg
    return cv.morphologyEx(maske, cv.MORPH_CLOSE, KERNEL)    # Löcher im Block füllen


def bloecke_finden(img):
    """Im zugeschnittenen Bild. Rückgabe: Liste (farbe, u, v, winkel)
    mit Pixelkoordinaten im ganzen entzerrten Bild."""
    h, w = img.shape[:2]
    seite = min(h, w)
    x_off = (w - seite) // 2
    y_off = (h - seite) // 2
    quadrat = img[y_off:y_off + seite, x_off:x_off + seite]
    hsv = cv.cvtColor(quadrat, cv.COLOR_BGR2HSV)

    bloecke = []
    for farbe, bereiche in FARBBEREICHE.items():
        maske = maske_bauen(hsv, bereiche)
        konturen, _ = cv.findContours(maske, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for c in konturen:
            if cv.contourArea(c) <= MIN_FLAECHE:
                continue
            (cx, cy), _, winkel = cv.minAreaRect(c)
            bloecke.append((farbe, cx + x_off, cy + y_off, winkel))
    return bloecke


def warte_auf(arduino, erwartet, timeout):
    start = time.time()
    while time.time() - start < timeout:
        antwort = arduino.readline().decode("ascii", errors="ignore").strip()
        if not antwort:
            continue
        print(f"  <- Arduino: {antwort}")
        if antwort == erwartet:
            return
        if antwort == "ERROR":
            raise RuntimeError("Arduino meldet ERROR.")
    raise TimeoutError(f"Kein '{erwartet}' innerhalb von {timeout:.0f} s.")


def sende_ziel(arduino, farbe, x, y, winkel):
    nachricht = f"{FARB_IDS[farbe]},{x:.2f},{y:.2f},{winkel:.2f}\n"
    print(f"-> Pi sendet: {nachricht.strip()}")
    arduino.write(nachricht.encode("ascii"))
    warte_auf(arduino, "DONE", DONE_TIMEOUT)


def main():
    frame = bild_aufnehmen()
    bloecke = bloecke_finden(entzerren(frame))
    print(f"{len(bloecke)} Blöcke erkannt.")

    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1) as arduino:
        warte_auf(arduino, "ROBOTER BEREIT", READY_TIMEOUT)

        for farbe, u, v, winkel in bloecke:
            x, y, a = welt_opt.block_zu_welt(u, v, winkel)

            if not welt_opt.im_arbeitsbereich(x, y):
                print(f"{farbe}: X={x:.1f} Y={y:.1f} ausserhalb -> skip")
                continue

            print(f"{farbe}: X={x:.1f} Y={y:.1f} Winkel={a:.1f}")
            sende_ziel(arduino, farbe, x, y, a)

    print("Sortiervorgang beendet.")

if __name__ == "__main__":
    main()
