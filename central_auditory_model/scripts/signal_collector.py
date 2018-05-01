import numpy as np
import warnings
import timeit

class SignalCollector(object):
    def __init__(self, data_dim, callback, initial_value=0.):
        self.data_dim = tuple(data_dim)
        self.callback = callback
        self.buffer = initial_value * np.ones(self.data_dim)
        self.done = np.zeros((self.data_dim[0],), dtype=np.bool)
        self.n = 0
        self.t = 0.

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

            self.buffer[index] = x
            self.done[index] = True

            if np.all(self.done):
                self.callback(self.t, self.buffer)

        return updater


def simple_cb(t, x):
    # return
    print t
    # print x


if __name__ == '__main__':
    sc = SignalCollector((7, 40), callback=simple_cb, initial_value=0.05)

    updater = []

    for i in range(7):
        updater.append(sc.updater_factory(i))

    t = 0.001

    for i in range(7):
        updater[i](t, t * i * np.ones([40]))

    t = 0.001

    for i in range(7):
        updater[i](t, t * i * np.ones([40]))

    t = 0.002

    for i in range(7):
        updater[i](t, t * i * np.ones([40]))

            
