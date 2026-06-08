class Observer:
    def __init__(self):
        pass

    def update(self, y_meas):
        self.y_meas = y_meas

    def predict(self):
        return y_meas