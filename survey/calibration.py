"""Fisheye calibration from checkerboard views at the capture resolution."""
import cv2
import numpy as np

BOARD = (9, 6)  # default inner corners; the device settings choose the board


def corners(jpeg, board=BOARD):
    gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
    found, pixels = cv2.findChessboardCornersSB(gray, tuple(board))
    if not found:
        raise ValueError("Board not detected. Include the full board and its white margin.")
    return pixels.reshape(-1, 1, 2).astype(float)


def solve(samples, board=BOARD, size=(640, 480)):
    if len(samples) < 12:
        raise ValueError("Collect at least 12 distinct board views.")
    grid = np.zeros((board[0] * board[1], 1, 3))
    grid[:, 0, :2] = np.mgrid[:board[0], :board[1]].T.reshape(-1, 2)
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW | cv2.fisheye.CALIB_CHECK_COND
    try:
        rms, k, d, _, _ = cv2.fisheye.calibrate([grid] * len(samples), samples, tuple(size),
                                                 None, None, flags=flags)
    except cv2.error as exc:
        raise ValueError("Calibration is ill-conditioned. Reset and vary board tilt and position.") from exc
    params = [k[0, 0], k[1, 1], k[0, 2], k[1, 2], *d.ravel()]
    if not np.isfinite([rms, *params]).all() or rms > 2 or min(params[:2]) <= 0:
        raise ValueError("Calibration fit is poor. Retake sharp views across the image.")
    return dict(params=params, rms_px=rms, size=list(size), views=len(samples), board=list(board))
