# constants.py
import numpy as np

tolerance = 2  # degrees

# Geometric parameters (mutable dicts like original; use constants module to tweak in one place)
GEOM = [
    {},  # placeholder for index 0..4
    {}, {}, {}, {}
]

# link/joint 0 (prismatic) geometric properties:
GEOM[0] = {
    "l_crank": 0.06,  # [m]
    "l_crod": 0.15,  # [m]
    "ht_diff": 0.07,  # [m]
}
# link/joint 1 (revolute) geometric properties:
GEOM[1] = {
    "h_tower": 0.1,  # [m]
    "offset": -0.01,  # [m]
}
# link/joint 2 (revolute) geometric properties:
GEOM[2] = {
    "l_link": 0.08,  # [m]
    "offset": -0.01,  # [m]
}
# link/joint 3 (revolute) geometric properties:
GEOM[3] = {
    "l_link": 0.08,  # [m]
    "offset": 0.01,  # [m]
}
# end effector geometric properties:
GEOM[4] = {
    "l_eff": 0.02,  # [m]
    "red":      0., # [rad] angular offset of each color
    "yellow":   np.pi/4, # [rad]
    "green":    2*np.pi/4, # [rad]
    "blue":     3*np.pi/4, # [rad]
    "white":    np.pi, # [rad]

}

# static transforms computed from geometry
# T_B0: position of frame {0} in base frame {B}
T_B0 = np.array(
    [
        [0.0, -1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, -GEOM[1]["h_tower"]],
        [
            0.0,
            0.0,
            1.0,
            -(
                (np.sqrt((GEOM[0]["l_crod"] - GEOM[0]["l_crank"]) ** 2 - GEOM[0]["ht_diff"] ** 2)
                 + np.sqrt((GEOM[0]["l_crod"] + GEOM[0]["l_crank"]) ** 2 - GEOM[0]["ht_diff"] ** 2)
                 )
                / 2.0
                + GEOM[1]["offset"]
                + GEOM[2]["offset"]
                + GEOM[3]["offset"]
            ),
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

# workspace limits (z & radial)
LIMS = {
    "z_min": (
        np.sqrt((GEOM[0]["l_crod"] - GEOM[0]["l_crank"]) ** 2 - GEOM[0]["ht_diff"] ** 2)
        - np.sqrt((GEOM[0]["l_crod"] + GEOM[0]["l_crank"]) ** 2 - GEOM[0]["ht_diff"] ** 2)
    )
    / 2.0,
    "z_max": (
        np.sqrt((GEOM[0]["l_crod"] + GEOM[0]["l_crank"]) ** 2 - GEOM[0]["ht_diff"] ** 2)
        - np.sqrt((GEOM[0]["l_crod"] - GEOM[0]["l_crank"]) ** 2 - GEOM[0]["ht_diff"] ** 2)
    )
    / 2.0,
    "r_max": np.sqrt(GEOM[2]["l_link"] ** 2 + GEOM[3]["l_link"] ** 2),
}

# transform from frame {4} (last joint) to frame {f} (end effector tip)
T_4F = np.array(
    [
        [1.0, 0.0, 0.0, GEOM[4]["l_eff"]],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
