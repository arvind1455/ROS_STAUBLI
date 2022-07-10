import numpy as np
from scipy.spatial.transform import Rotation
import math

def sqrt(value):
    """Returns the square root of a value"""
    return math.sqrt(value)

def sqrtA(value):
    """Returns the square root of a value if it's greater than 0, else 0 (differs from IEEE-754)."""
    if value <= 0:
        return 0
    return sqrt(value)

def hom(matrix):
    transformation_1 = [matrix[0][0], matrix[0][1], matrix[0][2], 0]
    transformation_2 = [matrix[1][0], matrix[1][1], matrix[1][2], 0]
    transformation_3 = [matrix[2][0], matrix[2][1], matrix[2][2], 0]
    transformation_4 = [0, 0, 0, 1]
    np.set_printoptions(precision=4, suppress=True)
    transformation_matrix = np.array([transformation_1, transformation_2, transformation_3, transformation_4])
    return transformation_matrix

def pose_2_quat(Ti):
    """Returns the quaternion orientation vector of a pose (4x4 matrix)"""
    tol_0 = 1e-9
    tol_180 = 1e-7

    cosangle = min(max(((Ti[0, 0] + Ti[1, 1] + Ti[2, 2] - 1.0) * 0.5), -1.0), 1.0)  # Calculate the rotation angle
    if cosangle > 1.0 - tol_0:
        # Identity matrix
        q1 = 1.0
        q2 = 0.0
        q3 = 0.0
        q4 = 0.0

    elif cosangle < -1.0 + tol_180:
        # 180 rotation around an axis
        diag = [Ti[0, 0], Ti[1, 1], Ti[2, 2]]
        k = diag.index(max(diag))
        col = [Ti[0, k], Ti[1, k], Ti[2, k]]
        col[k] = col[k] + 1.0
        rotvector = [n / sqrtA(2.0 * (1.0 + diag[k])) for n in col]

        q1 = 0.0
        q2 = rotvector[0]
        q3 = rotvector[1]
        q4 = rotvector[2]

    else:
        # No edge case, normal calculation
        a = Ti[0, 0]
        b = Ti[1, 1]
        c = Ti[2, 2]
        sign2 = 1.0
        sign3 = 1.0
        sign4 = 1.0
        if Ti[2, 1] - Ti[1, 2] < 0.0:
            sign2 = -1.0
        if Ti[0, 2] - Ti[2, 0] < 0.0:
            sign3 = -1.0
        if Ti[1, 0] - Ti[0, 1] < 0.0:
            sign4 = -1.0
        q1 =         sqrt(max( a + b + c + 1.0, 0.0)) / 2.0
        q2 = sign2 * sqrt(max( a - b - c + 1.0, 0.0)) / 2.0
        q3 = sign3 * sqrt(max(-a + b - c + 1.0, 0.0)) / 2.0
        q4 = sign4 * sqrt(max(-a - b + c + 1.0, 0.0)) / 2.0

    return [q1, q2, q3, q4]

def euler_2_quat(rz,ry,rx,x,y,z):
    rot = Rotation.from_euler('zyx', [rz, ry, rx], degrees=True)
    q = rot.as_quat()
    matrix = rot.as_matrix()
    transformation_1 = [matrix[0][0], matrix[0][1], matrix[0][2], x]
    transformation_2 = [matrix[1][0], matrix[1][1], matrix[1][2], y]
    transformation_3 = [matrix[2][0], matrix[2][1], matrix[2][2], z]
    transformation_4 = [0, 0, 0, 1]
    np.set_printoptions(precision=4, suppress=True)
    transformation_matrix = np.array([transformation_1, transformation_2, transformation_3, transformation_4])
    quat = pose_2_quat(transformation_matrix)
    return quat

def new_point(x,y,z,rz,ry,rx):
    rot = Rotation.from_euler('zyx', [rz, ry, rx], degrees=True)
    q = rot.as_quat()
    rot_matrix = rot.as_matrix()
    t = np.array([[1,0,0,-x],
                    [0,1,0,-y],
                    [0,0,1,-z],
                    [0,0,0,1]])
    t_inverse = np.linalg.inv(t)

    rx = Rotation.from_euler('x', rx, degrees=True)
    rx = rx.as_matrix()
    print(rx)
    rx = hom(rx)
    rx_inverse = np.linalg.inv(rx)

    ry = Rotation.from_euler('y', ry, degrees=True)
    ry = ry.as_matrix()
    ry = hom(ry)
    ry_inverse = np.linalg.inv(ry)

    rz = Rotation.from_euler('z', rz, degrees=True)
    rz = rz.as_matrix()
    rz = hom(rz)
    rz_inverse = np.linalg.inv(rz)

    trans = [x,y,z,1]
    new_p = rz.dot(ry).dot(rx).dot(t).dot(trans)


    return new_p[0],new_p[1],new_p[2]



