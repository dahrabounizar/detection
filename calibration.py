import os

import cv2
import numpy as np


CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "stereoMap.xml")

_warned_missing_intrinsics = False
_warned_missing_maps = False


def _empty_calibration():
    return {
        "stereoMapL_x": None,
        "stereoMapL_y": None,
        "stereoMapR_x": None,
        "stereoMapR_y": None,
        "cameraMatrixL": None,
        "cameraMatrixR": None,
        "newCameraMatrixL": None,
        "newCameraMatrixR": None,
        "projMatrixL": None,
        "projMatrixR": None,
        "Q": None,
    }


def _read_matrix(file_storage, name):
    node = file_storage.getNode(name)

    if node.empty():
        return None

    matrix = node.mat()
    if matrix is None or matrix.size == 0:
        return None

    return matrix


def load_calibration(path=CALIBRATION_FILE):
    data = _empty_calibration()
    cv_file = cv2.FileStorage()

    if not cv_file.open(path, cv2.FileStorage_READ):
        return data

    for key in data:
        data[key] = _read_matrix(cv_file, key)

    cv_file.release()
    return data


calibration_data = load_calibration()

# Compatibility with the old code.
stereoMapL_x = calibration_data["stereoMapL_x"]
stereoMapL_y = calibration_data["stereoMapL_y"]
stereoMapR_x = calibration_data["stereoMapR_x"]
stereoMapR_y = calibration_data["stereoMapR_y"]


def undistortRectify(frameR, frameL):
    global _warned_missing_maps

    maps_available = all(
        matrix is not None
        for matrix in (stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y)
    )

    if not maps_available:
        if not _warned_missing_maps:
            print("Calibration: stereo maps missing, using raw frames.")
            _warned_missing_maps = True
        return frameR, frameL

    undistortedL = cv2.remap(
        frameL,
        stereoMapL_x,
        stereoMapL_y,
        cv2.INTER_LANCZOS4,
        cv2.BORDER_CONSTANT,
        0,
    )
    undistortedR = cv2.remap(
        frameR,
        stereoMapR_x,
        stereoMapR_y,
        cv2.INTER_LANCZOS4,
        cv2.BORDER_CONSTANT,
        0,
    )

    return undistortedR, undistortedL


def _intrinsics_from_matrix(matrix, source):
    if matrix is None or matrix.shape[0] < 3 or matrix.shape[1] < 3:
        return None

    return {
        "fx": float(matrix[0, 0]),
        "fy": float(matrix[1, 1]),
        "cx": float(matrix[0, 2]),
        "cy": float(matrix[1, 2]),
        "source": source,
    }


def _intrinsics_from_fov(frame, alpha_degrees):
    height, width = frame.shape[:2]
    fx = (width * 0.5) / np.tan(np.deg2rad(alpha_degrees * 0.5))

    return {
        "fx": float(fx),
        "fy": float(fx),
        "cx": float(width * 0.5),
        "cy": float(height * 0.5),
        "source": "horizontal_fov_fallback",
    }


def get_camera_intrinsics(side="right", frame=None, alpha_degrees=None):
    """
    Return fx, fy, cx, cy for the selected camera.

    Exact mode:
        Uses projMatrixR/L or cameraMatrixR/L saved in stereoMap.xml.

    Fallback mode:
        Uses the horizontal field of view alpha_degrees. This keeps the program
        running with old stereoMap.xml files, but it is only approximate.
    """
    global _warned_missing_intrinsics

    side = side.lower()
    if side not in ("right", "left"):
        raise ValueError("side must be 'right' or 'left'")

    suffix = "R" if side == "right" else "L"
    candidates = (
        f"projMatrix{suffix}",
        f"newCameraMatrix{suffix}",
        f"cameraMatrix{suffix}",
    )

    for name in candidates:
        intrinsics = _intrinsics_from_matrix(calibration_data.get(name), name)
        if intrinsics is not None:
            return intrinsics

    if frame is None or alpha_degrees is None:
        raise ValueError(
            "No calibrated intrinsics found. Provide frame and alpha_degrees "
            "to use the approximate FOV fallback."
        )

    if not _warned_missing_intrinsics:
        print(
            "Calibration: camera intrinsics missing in stereoMap.xml. "
            "Using approximate alpha/FOV intrinsics."
        )
        _warned_missing_intrinsics = True

    return _intrinsics_from_fov(frame, alpha_degrees)


def get_q_matrix():
    return calibration_data["Q"]