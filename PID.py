import time


class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.time_stamp = time.time()
        self.last_time = self.time_stamp
        self.dt = 0
        self.inputs = [0, 0, 0]
        self.errors = [0, 0, 0]
        self.outputs = [0, 0, 0]
        self.intergration_separation = 5
        self.differential_limit = 0.1
        self.intergrations_sum_error_limit = 10
        self.output_limit = 15
        self.sum_error = 0

    def PID_Driver(self, target, Input):
        ctime = time.time()
        self.dt = self.time_stamp - ctime
        if self.dt > 1:
            self.time_stamp = time.time()
            return 0
        self.time_stamp = ctime
        self.inputs[2] = self.inputs[1]
        self.inputs[1] = self.inputs[0]
        self.inputs[0] = Input
        self.errors[2] = target - self.inputs[2]
        self.errors[1] = target - self.inputs[1]
        self.errors[0] = target - self.inputs[0]
        pterm = self.Kp * self.errors[0]
        if abs(self.errors[0]) < self.intergration_separation:
            self.sum_error += (self.errors[0] + self.errors[1]) * self.dt / 2
            if abs(self.sum_error) > self.intergrations_sum_error_limit:
                if self.sum_error > 0:
                    self.sum_error = self.intergrations_sum_error_limit
                if self.sum_error < 0:
                    self.sum_error = -self.intergrations_sum_error_limit
            iterm = self.Ki * self.sum_error
        else:
            iterm = 0
        dterm = self.Kd * (self.errors[0] - self.errors[1]) / self.dt

        new_output = pterm + iterm + dterm
        if abs(new_output) > (self.output_limit * 0.9):
            if new_output > 0:
                new_output = self.output_limit
            else:
                new_output = -self.output_limit
        else:
            new_output = (new_output + self.outputs[0] + self.outputs[1] + self.outputs[2]) / 4
        self.outputs[2] = self.outputs[1]
        self.outputs[1] = self.outputs[0]
        self.outputs[0] = new_output
        # print(new_output,target,Input)
        return int(new_output)

    def PID_SetGain(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
