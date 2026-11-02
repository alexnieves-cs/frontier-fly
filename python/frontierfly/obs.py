"""Observation layout v2 (``src/sim/OBS_LAYOUT.md``): index the observation by name, never by literal.

    from frontierfly import obs as O
    lidar = observation[O.LIDAR]            # 32 horizontal ranges / 8 m
    wp_forward, wp_left = observation[O.WP_DIR]

The test suite checks these against the simulator's own table.
"""

OBS_VERSION = 2
OBS_DIM = 1088
MAP_SIZE = 16
MAP_CELLS = MAP_SIZE * MAP_SIZE
LIDAR_RAYS = 32
LIDAR_MAX_M = 8.0
FLOOR_HEIGHT_M = 3.0

#: Start offset of each field, exactly as the simulator's ``OBS`` table.
OFFSETS = {
    "lidar": 0,
    "tilt": 32,
    "vel": 40,
    "yawRate": 43,
    "sinYaw": 44,
    "cosYaw": 45,
    "alt": 46,
    "ceil": 47,
    "roll": 48,
    "pitch": 49,
    "floor": 50,
    "time": 51,
    "nav": 52,
    "wpDir": 52,
    "pathLen": 54,
    "hasFrontier": 55,
    "clusterDir": 56,
    "clusterDist": 58,
    "clusterMass": 59,
    "stall": 60,
    "bfsTruncated": 61,
    "frontierTotal": 62,
    "frontierDist": 63,
    "localVisited": 64,
    "localFrontier": 64 + MAP_CELLS,
    "coarseVisited": 64 + 2 * MAP_CELLS,
    "coarseFrontier": 64 + 3 * MAP_CELLS,
    "end": 64 + 4 * MAP_CELLS,
}

LIDAR = slice(0, 32)  #: horizontal lidar, yaw + i * 2pi/32, range / 8 m (1 = miss)
TILT = slice(32, 40)  #: tilted lidar, +-0.45 rad elevation, every 45 deg, range / 8 m
VEL = slice(40, 43)  #: body-frame velocity forward / left / up over 2, 2, 1 m/s
YAW_RATE = 43
SIN_YAW, COS_YAW = 44, 45
ALT, CEIL = 46, 47  #: distance to floor and ceiling / floor height
ROLL, PITCH = 48, 49
FLOOR = 50
TIME = 51  #: fraction of episode steps remaining
WP_DIR = slice(52, 54)  #: body-frame unit vector to the BFS lookahead waypoint
PATH_LEN = 54
HAS_FRONTIER = 55
CLUSTER_DIR = slice(56, 58)
CLUSTER_DIST, CLUSTER_MASS = 58, 59
STALL = 60
BFS_TRUNCATED = 61
FRONTIER_TOTAL = 62
FRONTIER_DIST = 63
LOCAL_VISITED = slice(64, 64 + MAP_CELLS)  #: egocentric 16x16 at 0.5 m, cell (ix, iz) at iz * 16 + ix
LOCAL_FRONTIER = slice(64 + MAP_CELLS, 64 + 2 * MAP_CELLS)
COARSE_VISITED = slice(64 + 2 * MAP_CELLS, 64 + 3 * MAP_CELLS)  #: egocentric 16x16 at 2 m
COARSE_FRONTIER = slice(64 + 3 * MAP_CELLS, 64 + 4 * MAP_CELLS)


def maps(observation):
    """The four egocentric maps as a (4, 16, 16) array, indexed [channel, iz, ix]."""
    return observation[64:OBS_DIM].reshape(4, MAP_SIZE, MAP_SIZE)
