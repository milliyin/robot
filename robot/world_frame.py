from typing import Tuple


MOBILE_BASE_HEIGHT_M = 0.18


def arm_base_xyz_to_world_xyz(xyz):
    x, y, z = xyz
    return (float(x), float(y), float(z) + MOBILE_BASE_HEIGHT_M)
