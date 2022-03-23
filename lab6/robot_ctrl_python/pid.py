from matplotlib import pyplot as plt
import numpy as np
from scipy.signal import butter, lfilter, freqz
'''
ECE 4960: Designing a PID controller
'''


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


cutoff = 3.667
fs = 30.0
order = 6

T = 5.0  # value taken in seconds
n = int(T * fs)  # indicates total samples
t = np.linspace(0, T, n, endpoint=False)
data = np.sin(1.2 * 2 * np.pi * t) + 1.5 * np.cos(
    9 * 2 * np.pi * t) + 0.5 * np.sin(12.0 * 2 * np.pi * t)

y = butter_lowpass_filter(data, cutoff, fs, order)

plt.subplot(2, 1, 2)
plt.plot(t, data, 'b-', label='data')
plt.plot(t, y, 'g-', linewidth=2, label='filtered data')
plt.xlabel('Time [sec]')
plt.grid()
plt.legend()

plt.subplots_adjust(hspace=0.35)
plt.show()


class System:
    def __init__(self,
                 A=[[0, 1], [0, -0.2]],
                 B=[0, 1],
                 x0=[0, 0],
                 sigma=0,
                 dt=0.005):

        self.x = np.array(x0)
        self.t = 0
        self.dt = dt

        self.sigma = sigma

        self.A = np.array(A)
        self.B = np.array(B)

        self.x_hist = [x0]
        self.y_hist = [0]
        self.t_hist = [self.t]
        self.e_hist = [0]
        '''
        Controller parameters
        '''
        self.I = 0
        self.dF = 0

    def step(self, u):
        self.x = self.x + self.dt * (np.dot(self.A, self.x) + u * self.B)
        self.t += self.dt
        self.t_hist.append(self.t)
        self.x_hist.append(tuple(self.x))
        self.y_hist.append(self.x[1] + np.random.normal(scale=self.sigma))

    def PID(self, setpoint, KP=0, KI=0, KD=0, alpha=0):
        '''
        1) get error signal
        2) Update integral
        3) Compute d for derivative
        4) Compute and return u
        '''
        e = setpoint - self.y_hist[-1]
        self.e_hist.append(e)

        self.I += e * self.dt

        if len(self.e_hist) >= 2:
            d = (self.e_hist[-1] - self.e_hist[-2]) / self.dt
            self.dF = alpha * d + (1 - alpha) * self.dF
        else:
            self.dF = 0

        return KP * e + KD * self.dF + KI * self.I

    def runPID(self, setpoint, KP=0, KI=0, KD=0, Tfinal=10, alpha=0):
        while self.t < Tfinal:
            u = self.PID(setpoint, 1.0 * KP, 1.0 * KI, 1.0 * KD, alpha=alpha)
            self.step(u)


sys = System(sigma=0)

setpoint = 2
final_time = 10

sys.runPID(setpoint, KP=1, KI=0, KD=0, alpha=1, Tfinal=final_time)

#Plot Controller Output
#plt.plot(sys.t_hist,sys.y_hist)
#Plot Actual State
plt.plot(sys.t_hist, tuple(x[1] for x in sys.x_hist), 'r--')
#Plot Setpoint
plt.plot([0, final_time], [setpoint, setpoint], 'k:')
plt.grid()
plt.show()
