import numpy as np


def mapTOWorldSpace(point,z,fxypxy):
    '''
        Maps a point from pixel space to world space
        :param point: The point to be mapped    
        :param z: The depth of the point
        :param fxypxy: The camera matrix and distortion coefficients
        :return: The point in world space
    '''
    cy,cx = point[0],point[1]
    fx, fy = fxypxy[0], fxypxy[1]
    px, py = fxypxy[2], fxypxy[3]
    # x = (point[0] - px) * z / fx
    # y = (point[1] - py) * -z / fy
    # z = -z
    # point = np.array([x, y, z])

    x = z * (cx - px) / fx
    y = -z * (cy - py) / fy
    z = -z
            

    return (x,y,z)