MOBILE_BASE_TO_S1_HEIGHT_M = 0.18
BASE_LINK_TO_S1_HEIGHT_M = 0.03
MOBILE_BASE_TO_ARM_BASE_HEIGHT_M = MOBILE_BASE_TO_S1_HEIGHT_M - BASE_LINK_TO_S1_HEIGHT_M


def arm_base_xyz_to_world_xyz(xyz):
    x, y, z = xyz
    return (float(x), float(y), float(z) + MOBILE_BASE_TO_ARM_BASE_HEIGHT_M)
