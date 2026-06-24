import rclpy
from rclpy.node import Node

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.declare_parameter('r_matrix_diagonal', [1e-8] * 5)
        r_diag = self.get_parameter('r_matrix_diagonal').get_parameter_value().double_array_value

    def update(self, y_meas):
        self.y_meas = y_meas

    def predict(self):
        return y_meas