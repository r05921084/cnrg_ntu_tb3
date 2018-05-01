import numpy as np
import warnings
import timeit

class SignalCollector(object):
    def __init__(self, data_dim, callback, initial_value=0., maxpooling=0):
        self.data_dim = tuple(data_dim)        
        self.callback = callback
        
        self.in_buf = initial_value * np.ones(self.data_dim)        
        self.done = np.zeros((self.data_dim[0],), dtype=np.bool)
        self.t = 0.

        self.maxpooling = max(1, int(maxpooling))
        self.mp_cnt = self.maxpooling
        self.mp_buf = np.zeros(self.data_dim)

    def updater_factory(self, index):

        def updater(t, x):
            if t > self.t:
                self.t = t
                self.done[:] = 0
            elif t < self.t:
                warnings.warn('[%d]: t mismatch (%f, %f), ignore!' % (index, t, self.t), Warning)
                return
            elif self.done[index]:
                warnings.warn('[%d]: re-entrant, ignore!' % (index,), Warning)
                return

            self.in_buf[index] = x
            self.done[index] = True

            if np.all(self.done):
                np.max([self.mp_buf, self.in_buf], axis=0, out=self.mp_buf)
                self.mp_cnt -= 1

                if not self.mp_cnt:
                    self.callback(self.t, self.mp_buf)
                    self.mp_buf[:] = 0.
                    self.mp_cnt = self.maxpooling


        return updater


def simple_cb(t, x):
    # return
    print t
    print x


if __name__ == '__main__':
    sc = SignalCollector((7, 40), callback=simple_cb, initial_value=0.05, maxpooling=2)

    updater = []

    for i in range(7):
        updater.append(sc.updater_factory(i))

    t = 0.001

    for i in range(7):
        updater[i](t, 10 * t * i * np.ones([40]))

    t = 0.001

    for i in range(7):
        updater[i](t, t * i * np.ones([40]))

    t = 0.002

    for i in range(7):
        updater[i](t, t * i * np.ones([40]))

    t = 0.003

    for i in range(7):
        updater[i](t, t * i * np.ones([40]))


    t = 0.004

    for i in range(7):
        updater[i](t, t * i * np.ones([40]))

            
