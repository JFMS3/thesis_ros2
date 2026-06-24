import numpy as np

class FrameTransformer:
    def quat_to_euler(self, orientation):
        if isinstance(orientation, (np.ndarray, list)):
            x = orientation[0]
            y = orientation[1]
            z = orientation[2]
            w = orientation[3]
        else:
            x = orientation.x
            y = orientation.y
            z = orientation.z
            w = orientation.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        phi = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)
        theta = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        psi = np.arctan2(siny_cosp, cosy_cosp)

        return phi, theta, psi

    def optitrack_to_ros(self, position):
        return np.array([
            position.z,
            -position.x,
            position.y
        ])

    def normalise_quat(self, q):
        q = np.asarray(q, dtype=float)
        return q / np.linalg.norm(q)

    def quat_conjugate(self, q):
        return np.array([-q[0], -q[1], -q[2], q[3]])

    def quat_multiply(self, a, b):
        x1, y1, z1, w1 = a
        x2, y2, z2, w2 = b

        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])
