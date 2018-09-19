from Pendulum import Pendulum
import math
import matplotlib.pyplot as plt

def main():
    kp = 0.1
    ki = 1 * kp
    kd = 0.5 * kp
    env = Pendulum()
    st = env.reset()
    ts_rec = []
    theta_rec = []
    dtheta_rec = []
    inter = 0
    for i in range(10000):
        ts_rec.append(i)
        theta_rec.append(st[0])
        dtheta_rec.append(st[1])
        inter += (-st[0]) * 0.001
        dev = -st[1]
        u = kp * (-st[0]) + ki * inter + kd * dev
        st, _ = env.step(u)
    plt.plot(ts_rec, theta_rec, 'b-', ts_rec, dtheta_rec, 'r-')
    plt.show()

if __name__ == '__main__':
    main()
