import numpy as np

class DelayLine(object):
    def __init__(self, data_dim, sample_rate, max_delay=1.0, initial_value=0.):
        self.data_dim = data_dim
        self.sample_rate = float(sample_rate)
        self.max_delay = max_delay
        self.length = int(max_delay * sample_rate)
        self.buffer = initial_value * np.ones((self.length,) + tuple(data_dim))
        self.current_step = 0
        self.head = 0

        print self.buffer.shape

    def update(self, data):
        # print data.shape
        # if data.shape[0] > self.length:
        #     print 'buffer overflow: updating %d steps while buffer is only %d steps.' % (data.shape[0], self.length)
        #     return

        stop = self.head + data.shape[0]
        wrapped_index = np.arange(self.head, stop)
        wrapped_index[wrapped_index >= self.length] -= self.length
        self.head = (stop - self.length) if stop >= self.length else stop

        self.buffer[wrapped_index] = data

        self.current_step += data.shape[0]        
        # print 'head', self.head, self.current_step / self.sample_rate
        

    def delayed_view(self, delay):

        def view(t):
            steps_behind = self.current_step - int((t - delay) * self.sample_rate)
            if steps_behind > self.length:
                # print steps_behind, 'already forgot!'
                return self.buffer[0]
            elif steps_behind <= 0:
                # print steps_behind, 'not there yet!'
                return self.buffer[0]
            else:
                index = self.head - steps_behind
                index = index + self.length if index < 0 else index
                # print 'view', index, steps_behind
                return self.buffer[index]

        return view


if __name__ == '__main__':
    dl = DelayLine((40,), sample_rate=11025, initial_value=0.05)
    dl.update(np.arange(11025)[:,np.newaxis] * 40 + np.arange(40))
    view = dl.delayed_view(0.)
    view(-0.5)
    view(0.)
    view(0.1)
    view(0.5)
    view(1.0)
    view(1.5)
