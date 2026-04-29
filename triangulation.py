import numpy as np


def _focal_length_from_frame(frame, alpha_degrees):
    _, width = frame.shape[:2]
    return (width * 0.5) / np.tan(np.deg2rad(alpha_degrees * 0.5))


def _get_fx_pixels(frame, alpha_degrees, intrinsics=None):
    if intrinsics is None:
        return _focal_length_from_frame(frame, alpha_degrees)

    if isinstance(intrinsics, dict):
        return float(intrinsics["fx"])

    matrix = np.asarray(intrinsics, dtype=float)
    if matrix.ndim >= 2 and matrix.shape[0] >= 3 and matrix.shape[1] >= 3:
        return float(matrix[0, 0])

    return float(matrix.flat[0])


def find_depth(
    right_point,
    left_point,
    frame_right,
    frame_left,
    baseline,
    f,
    alpha,
    distance_scale=1.0,
    intrinsics=None,
    min_depth_cm=10,
    max_depth_cm=500,
):
    """
    Calcule seulement la profondeur Z en cm avec la stereo vision.

    Z = baseline * fx / disparity
    """
    del f  # Garde pour compatibilite avec l'ancien appel.

    _, width_right = frame_right.shape[:2]
    _, width_left = frame_left.shape[:2]

    if width_right != width_left:
        print("Left and right camera frames do not have the same pixel width")
        return float("inf")

    fx_pixels = _get_fx_pixels(frame_right, alpha, intrinsics)

    x_right = float(right_point[0])
    x_left = float(left_point[0])
    disparity = abs(x_right - x_left)

    if disparity <= 0:
        return float("inf")

    z_depth = ((baseline * fx_pixels) / disparity) * distance_scale

    if not np.isfinite(z_depth) or z_depth < min_depth_cm or z_depth > max_depth_cm:
        return float("inf")

    return float(z_depth)