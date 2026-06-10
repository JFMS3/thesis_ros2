import numpy as np

class FrameTransformer:
    def quat_to_euler(orientation):
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