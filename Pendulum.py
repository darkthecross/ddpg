import math
import numpy as np
import matplotlib.pyplot as plt

class Pendulum():

    def __init__(self, gravity=9.8, length=0.042, mass=0.055, timestep=0.05):
        self.g = gravity
        self.l = length
        self.ts = timestep
        self.m = mass
        self.theta = math.pi
        self.dtheta = 0

    def reset(self):
        self.theta = math.pi
        self.dtheta = 0
        st_res = np.zeros((2,), dtype=np.float32)
        st_res[0] = self.theta
        st_res[1] = self.dtheta
        return st_res

    def step(self, torque):
        tt = torque + self.m * self.g * self.l * math.sin(self.theta)
        inertia = self.m * self.l * self.l
        #RK4
        k1 = [self.dtheta * self.ts, self.ts * tt / inertia]
        y1 = [self.theta + k1[0]/2, self.dtheta + k1[1] / 2]
        t1 = torque + self.m * self.g * self.l * math.sin(y1[0])
        k2 = [y1[1] * self.ts, self.ts * t1 / inertia]
        y2 = [self.theta +  k2[0]/2, self.dtheta + k2[1] / 2]
        t2 = torque + self.m * self.g * self.l * math.sin(y2[0])
        k3 = [y2[1] * self.ts, self.ts * t2 / inertia]
        y3 = [self.theta +  k3[0], self.dtheta + k3[1]]
        t3 = torque + self.m * self.g * self.l * math.sin(y3[0])
        k4 = [y3[1] * self.ts, self.ts * t3 / inertia]
        self.theta += (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])/6
        self.dtheta += (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])/6
        # limit dtheta and theta
        if self.theta > 2*math.pi:
            ct = int(self.theta / 2 / math.pi)
            self.theta -= ct * 2 * math.pi
        elif self.theta < -2 * math.pi:
            ct = int((-self.theta)/2/math.pi)
            self.theta += ct * 2 * math.pi
        if self.dtheta > 10.0:
            self.dtheta = 10.0
        elif self.dtheta < -10.0:
            self.dtheta = -10.0
        rwd = -5*self.theta*self.theta-0.1*self.dtheta*self.dtheta-torque*torque
        st_res = np.zeros((2,), dtype=np.float32)
        st_res[0] = self.theta
        st_res[1] = self.dtheta
        return st_res, float(rwd)

def main():
    env = Pendulum()
    st = env.reset()
    ts_rec = []
    theta_rec = []
    dtheta_rec = []
    for i in range(50):
        ts_rec.append(i)
        theta_rec.append(st[0])
        dtheta_rec.append(st[1])
        st, _ = env.step(-0.013)
    plt.plot(ts_rec, theta_rec, 'b-', ts_rec, dtheta_rec, 'r-')
    plt.show()

if __name__ == '__main__':
    main()
