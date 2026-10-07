##### PYDRAKE IMPORTS
from pydrake.systems.sensors import CameraConfig

##### OTHER IMPORTS
import numpy as np

##### CAMERA MODEL FUNCTION (inputs camera config, camera transform, and target state and returns measured pixel output)
def CameraMeasure(config: CameraConfig, X_CW: np.array, x_T: np.array) -> tuple:
    # extracting camera config parameters
    fx = config.focal_x()
    fy = config.focal_y()
    u0 = config.principal_point()[0]
    v0 = config.principal_point()[1]

    # creating projection transformation matrix
    K = np.array([[fx, 0, u0, 0],
                  [0, fy, v0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    # transferring to homoegenous coordinates
    if len(x_T) != 4: x_T = np.append(x_T, 1)

    T_CW = X_CW.GetAsMatrix4()

    # performing projection
    p_T = K @ T_CW @ x_T

    # extracting pixels
    w = p_T[2] # finding depth normalization
    u = p_T[0] / w # finding x pixel coordinate
    v = p_T[1] / w # finding y pixel coordinate

    if (u < config.width and u > 0) and (v < config.height and v > 0): # if target is in frame
        return u, v
    else:
        return None, None # return empty set