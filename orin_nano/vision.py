"""Board reading on the Orin Nano, with any USB camera instead of the Cognex.

Four ArUco markers just outside the board corners say where the board is in the
picture, wherever the camera sits, so every frame can be warped to a top-down
view laid out like a chess diagram. Telling a bishop from a pawn from above is
fragile, and not needed: the game is followed from the start, so after the
person's turn the only question is which legal move they made. Each square is
compared before and after, and the legal move whose squares changed the most,
with nothing else changing more, is the one that was played.

    python vision.py --markers      write the four markers to print
    python vision.py --calibrate    lens calibration from a printed checkerboard
    python vision.py                live view with the board grid drawn over it
"""
import sys

import chess
import cv2
import numpy as np

import config as C
import chess_board as B

PX = 2                                    # Pixels per millimetre in the warped view
CHANGED = 3.0                             # A square has changed when it moved this many times the noise
SIZE = int(B.BOARD * PX)                  # 640 x 640, the board edge to edge
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
DETECTOR = cv2.aruco.ArucoDetector(DICTIONARY, cv2.aruco.DetectorParameters())

# Marker ids 0..3 sit beyond a1, h1, h8 and a8, in board millimetres
EDGE = B.BOARD / 2 + C.MARKER_OFFSET
MARKERS = {0: (EDGE, -EDGE), 1: (-EDGE, -EDGE), 2: (-EDGE, EDGE), 3: (EDGE, EDGE)}
LENS = np.load(C.CALIBRATION) if C.CALIBRATION.exists() else None


def to_pixel(x, y):
    """Board millimetres to the warped view: file a on the left, rank 8 at the top."""
    return (B.BOARD / 2 - x) * PX, (B.BOARD / 2 - y) * PX


def open_camera():
    camera = cv2.VideoCapture(C.CAMERA)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, C.RESOLUTION[0])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, C.RESOLUTION[1])
    return camera


def grab(camera, undistort=True):
    """A fresh frame. The driver keeps a few old ones while the robot moves, drop them."""
    for _ in range(5):
        camera.grab()
    ok, frame = camera.read()
    if not ok:
        raise RuntimeError("the camera returned no frame")
    return cv2.undistort(frame, LENS['matrix'], LENS['distortion']) if undistort and LENS is not None else frame


def warp(frame):
    """Top-down view of the board, or None while any of the four markers is hidden."""
    corners, ids, _ = DETECTOR.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    found = {int(i): c[0].mean(axis=0) for i, c in zip(ids.flatten(), corners)} if ids is not None else {}
    if not all(k in found for k in MARKERS):
        return None
    image = np.float32([found[k] for k in MARKERS])
    board = np.float32([to_pixel(*MARKERS[k]) for k in MARKERS])
    return cv2.warpPerspective(frame, cv2.getPerspectiveTransform(image, board), (SIZE, SIZE))


def board_image(camera, tries=10):
    for _ in range(tries):
        view = warp(grab(camera))
        if view is not None:
            return view
    raise RuntimeError("could not see all four board markers")


def square_changes(before, after):
    """How much the centre of each square changed, in multiples of the noise.

    Nearly every square is untouched by a move, so the median change is the
    noise. Measuring against it rather than against the biggest change matters
    for a white piece on a light square, which barely shows: four times the
    noise is weak next to a dark square, but it is still plainly a change.
    """
    a, b = (cv2.GaussianBlur(cv2.cvtColor(v, cv2.COLOR_BGR2LAB), (7, 7), 0).astype(np.float32) for v in (before, after))
    b[..., 0] *= np.median(a[..., 0]) / max(np.median(b[..., 0]), 1.0)   # Same overall brightness
    half = int(C.PATCH / 2 * PX)
    change = {}
    for f in B.FILES:
        for r in B.RANKS:
            u, v = (int(p) for p in to_pixel(*B.get_coordinates(f + r)[:2]))
            patch = np.s_[v - half:v + half, u - half:u + half]
            change[f + r] = float(np.linalg.norm(a[patch] - b[patch], axis=2).mean())
    noise = max(float(np.median(list(change.values()))), 1e-6)
    return {square: c / noise for square, c in change.items()}


def infer_move(board, before, after):
    """The legal move that best explains what changed, and how sure that is, 0 to 1.

    A move scores for every square it touches that changed by more than CHANGED
    times the noise, loses for every touched square that did not, and loses for
    every square it leaves alone that changed anyway. Sure means well clear of
    the best explanation that moves a different piece or to a different square;
    the four promotions of one pawn look the same from above, so a queen is assumed.
    """
    change = square_changes(before, after)

    def score(move):
        touched = B.touched_squares(board, move)
        return sum(change[s] - CHANGED if s in touched else -max(change[s] - CHANGED, 0) for s in change)

    ranked = sorted(board.legal_moves, key=lambda m: (score(m), m.promotion in (None, chess.QUEEN)), reverse=True)
    best = ranked[0]
    rival = next((m for m in ranked if (m.from_square, m.to_square) != (best.from_square, best.to_square)), None)
    top, second = score(best), score(rival) if rival else 0.0
    return best, max(0.0, min(1.0, (top - second) / top)) if top > 0 else 0.0


def calibrate(camera, pattern=(9, 6), square=25.0):
    """Lens calibration: SPACE keeps a view of the checkerboard, ENTER solves after ten."""
    grid = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square
    views = []
    while True:
        frame = grab(camera, undistort=False)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern)
        cv2.drawChessboardCorners(frame, pattern, corners, found)
        cv2.putText(frame, f"{len(views)} views", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("calibrate", frame)
        key = cv2.waitKey(30)
        if key == 32 and found:
            views.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)))
        if key == 13 and len(views) >= 10:
            break
    error, matrix, distortion, _, _ = cv2.calibrateCamera([grid] * len(views), views, gray.shape[::-1], None, None)
    np.savez(C.CALIBRATION, matrix=matrix, distortion=distortion)
    print(f"Saved {C.CALIBRATION.name}, reprojection error {error:.2f} px")


if __name__ == "__main__":
    if '--markers' in sys.argv:
        for k in MARKERS:
            image = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(DICTIONARY, k, 400),
                                       40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
            cv2.imwrite(f"marker_{k}.png", image)
        print("Wrote marker_0..3.png. Print them about 30 mm wide, centres "
              f"{C.MARKER_OFFSET:.0f} mm beyond a1, h1, h8 and a8 along both edges.")
        sys.exit()

    camera = open_camera()
    if '--calibrate' in sys.argv:
        calibrate(camera)
        sys.exit()

    while cv2.waitKey(30) != 27:                  # Esc to quit
        frame = grab(camera)
        view = warp(frame)
        if view is None:
            cv2.putText(frame, "markers not all visible", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("board", frame)
            continue
        for k in range(9):                        # Square edges, and each square's name
            p = int(k * B.SQUARE * PX)
            cv2.line(view, (p, 0), (p, SIZE), (0, 255, 0), 1)
            cv2.line(view, (0, p), (SIZE, p), (0, 255, 0), 1)
        for f in B.FILES:
            for r in B.RANKS:
                u, v = to_pixel(*B.get_coordinates(f + r)[:2])
                cv2.putText(view, f + r, (int(u) - 12, int(v) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.imshow("board", view)
