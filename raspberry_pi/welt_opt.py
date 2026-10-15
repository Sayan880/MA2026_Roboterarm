"""
Umrechnung Pixel (entzerrtes Vollbild) -> Weltkoordinaten in cm auf dem Tisch,
mit der Homographie aus homographie.npy
"""
import cv2 as cv
import numpy as np

ARM_BASIS = (50.0, 50.0)        # Drehachse des Arms in Weltkoordinaten, cm
REICHWEITE_MIN = 8.0            # cm
REICHWEITE_MAX = 40.0           # cm

H = np.load("homographie.npy")

"""Umrechnung der Punkte, Für den Winkel wird ein weiterer Punkt transformiert."""
def block_zu_welt(u, v, winkel_px):
    a = np.deg2rad(winkel_px)
    punkte = np.float32([[[u, v]],
                         [[u + 40 * np.cos(a), v + 40 * np.sin(a)]]])
    (x1, y1), (x2, y2) = cv.perspectiveTransform(punkte, H).reshape(-1, 2)

    winkel = np.degrees(np.arctan2(y2 - y1, x2 - x1)) % 90.0
    return float(x1), float(y1), float(winkel)


def im_arbeitsbereich(x, y):
    r = np.hypot(x - ARM_BASIS[0], y - ARM_BASIS[1])
    return REICHWEITE_MIN <= r <= REICHWEITE_MAX
