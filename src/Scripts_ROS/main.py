import numpy as np
from scipy.spatial.transform import Rotation


def normalize(v):
    return v / np.linalg.norm(v)


def find_additional_vertical_vector(vector):
    ez = np.array([0, 0, 1])
    look_at_vector = normalize(vector)
    up_vector = normalize(ez - np.dot(look_at_vector, ez) * look_at_vector)
    return up_vector


def calc_rotation_matrix(v1_start, v2_start, v1_target, v2_target):
    """
    calculating M the rotation matrix from base U to base V
    M @ U = V
    M = V @ U^-1
    """

    def get_base_matrices():
        u1_start = normalize(v1_start)
        u2_start = normalize(v2_start)
        u3_start = normalize(np.cross(u1_start, u2_start))

        u1_target = normalize(v1_target)
        u2_target = normalize(v2_target)
        u3_target = normalize(np.cross(u1_target, u2_target))

        U = np.hstack([u1_start.reshape(3, 1), u2_start.reshape(3, 1), u3_start.reshape(3, 1)])
        V = np.hstack([u1_target.reshape(3, 1), u2_target.reshape(3, 1), u3_target.reshape(3, 1)])

        return U, V

    def calc_base_transition_matrix():
        return np.dot(V, np.linalg.inv(U))

    if not np.isclose(np.dot(v1_target, v2_target), 0, atol=1e-03):
        raise ValueError("v1_target and v2_target must be vertical")

    U, V = get_base_matrices()
    return calc_base_transition_matrix()


def get_euler_rotation_angles(start_look_at_vector, target_look_at_vector, start_up_vector=None, target_up_vector=None):
    if start_up_vector is None:
        start_up_vector = find_additional_vertical_vector(start_look_at_vector)

    if target_up_vector is None:
        target_up_vector = find_additional_vertical_vector(target_look_at_vector)

    rot_mat = calc_rotation_matrix(start_look_at_vector, start_up_vector, target_look_at_vector, target_up_vector)
    is_equal = np.allclose(rot_mat @ start_look_at_vector, target_look_at_vector, atol=1e-03)
    print(f"rot_mat @ start_look_at_vector1 == target_look_at_vector1 is {is_equal}")
    rotation = Rotation.from_matrix(rot_mat)
    return rotation.as_euler(seq="xyz", degrees=True)

if __name__ == "__main__":
    # Example 1
    start_look_at_vector = np.array([0.76235,0.003,0.17119])
    target_look_at_vector = normalize(np.array([0.60437,-0.04223,0.092198]))

    phi, theta, psi = get_euler_rotation_angles(start_look_at_vector, target_look_at_vector)
    print(f"phi_x_rotation={phi}, theta_y_rotation={theta}, psi_z_rotation={psi}")