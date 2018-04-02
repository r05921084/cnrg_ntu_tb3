import os
import threading
import time
import subprocess

# FIFO_PATH = '/tmp/ipem'
# FIFO_PATH = '.'
# FIFO_PCM = 'ipem_pcm'
# FIFO_ANI = 'ipem_ani'
# FIFO_PCM_WITH_PATH = FIFO_PATH + '/' + FIFO_PCM
# FIFO_ANI_WITH_PATH = FIFO_PATH + '/' + FIFO_ANI

class IPEM_pipe:
    def __init__(self, channel_number=40, first_freq=2.0, freq_distance=0.5, sample_frequency=None,
                 new_ani_callback=None, raw_ani_string=True, fifo_path='.'):
        self.channel_number = channel_number
        self.first_freq = first_freq
        self.freq_distance = freq_distance
        self.sample_frequency = sample_frequency
        self.new_ani_callback = new_ani_callback
        self.FIFO_PATH = fifo_path
        self.FIFO_PCM = 'ipem_pcm'
        self.FIFO_ANI = 'ipem_ani'
        self.FIFO_PCM_WITH_PATH = self.FIFO_PATH + '/' + self.FIFO_PCM
        self.FIFO_ANI_WITH_PATH = self.FIFO_PATH + '/' + self.FIFO_ANI

        try:
            os.makedirs(self.FIFO_PATH)
        except OSError as oe:
            print oe

        try:
            os.mkfifo(self.FIFO_PCM_WITH_PATH)
        except OSError as oe:
            print oe

        try:
            os.mkfifo(self.FIFO_ANI_WITH_PATH)
        except OSError as oe:
            print oe

        def pcm_feeder_generator():
            with open(self.FIFO_PCM_WITH_PATH, 'wb') as fifo_pcm:
                print("%s opened" % self.FIFO_PCM)
                while True:
                    samples = (yield 0)
                    # print 'feed %d bytes' % len(samples)
                    try:
                        fifo_pcm.write(samples)
                        fifo_pcm.flush()
                    except IOError:
                        print 'closing pcm_feeder due to fifo IOError'
                        return

                    if (yield 1):
                        print 'closing pcm_feeder due to close()'
                        return

        self.pcm_feeder = pcm_feeder_generator()

        def ani_receiver():
            with open(self.FIFO_ANI_WITH_PATH, 'rb') as fifo_ani:
                print("%s opened" % self.FIFO_ANI)
                while True:
                    ani_str = fifo_ani.readline()
                    # print('READ(%d): %s' % (len(ani_str), ani_str))
                    if len(ani_str):
                        if raw_ani_string:
                            self.new_ani_callback(ani_str)
                        else:
                            print 'not implement yet'
                            # self.new_ani_callback(np_data)
                    else:
                        break

            print('"%s" closed' % self.FIFO_ANI_WITH_PATH)

        self.ani_receiver_thread = threading.Thread(target=ani_receiver)
        self.ani_receiver_thread.start()

        def ipem_process():
            try:
                t1 = time.time()
                log_str = subprocess.check_output(['./IPEMAuditoryModelConsole',
                                                   '-nc', str(40),
                                                   '-if', self.FIFO_PCM,
                                                   '-id', self.FIFO_PATH,
                                                   '-of', self.FIFO_ANI,
                                                   '-od', self.FIFO_PATH,
                                                   '-fs', str(22050),
                                                   '-ff', 'pcm'])
                print log_str
                print 'subprocess returned in %f sec' % (time.time() - t1)
            except subprocess.CalledProcessError as e:
                print e

        self.ipem_process_thread = threading.Thread(target=ipem_process)
        self.ipem_process_thread.start()

    def feed_pcm_samples(self, samples):
        try:
            self.pcm_feeder.next()
            self.pcm_feeder.send(samples)
        except StopIteration as e:
            pass

    def close(self):
        try:
            self.pcm_feeder.send(True)
            print 'sent close()'
        except StopIteration:
            pass


if __name__ == '__main__':
    def just_print(data):
        print data

    ipem_pipe = IPEM_pipe(new_ani_callback=just_print)
    ipem_pipe.feed_pcm_samples('test')
    ipem_pipe.feed_pcm_samples('test2')
    ipem_pipe.close()


# with open('input.wav', 'r') as wavfile:
#     data = wavfile.read()
#     print len(data[44:]), data[:44], '%x %x %x %x' % (ord(data[44]), ord(data[45]), ord(data[46]), ord(data[47]))
#     data = data[44:]


# def pcm_feeder():
#     with open(FIFO_PCM_WITH_PATH, 'wb') as fifo_pcm:
#         print("%s opened" % FIFO_PCM)
#
#         # fifo_pcm.write(data)
#         # fifo_pcm.flush()
#
#         for i in range(len(data) // 2):
#             fifo_pcm.write(data[i*2:i*2+2])
#             # fifo_pcm.flush()
#             # time.sleep(1.0 / 44100)
#
#
# def ani_receiver():
#     with open(FIFO_ANI_WITH_PATH, 'rb') as fifo_ani:
#         print("%s opened" % FIFO_ANI)
#
#         ani_str = 'EMPTY'
#
#         while len(ani_str):
#             ani_str = fifo_ani.readline()
#             # print('READ(%d): %s' % (len(ani_str), ani_str))
#
#         print('"%s" closed' % FIFO_ANI_WITH_PATH)
#
#
# pcm_feeder_thread = threading.Thread(target=pcm_feeder)
# ani_receiver_thread = threading.Thread(target=ani_receiver)
#
# pcm_feeder_thread.start()
# ani_receiver_thread.start()
#
# try:
#     t1 = time.time()
#     log_str = subprocess.check_output(['./IPEMAuditoryModelConsole',
#                                        '-nc', str(40),
#                                        '-if', FIFO_PCM,
#                                        '-id', FIFO_PATH,
#                                        '-of', FIFO_ANI,
#                                        '-od', FIFO_PATH,
#                                        '-fs', str(44100),
#                                        '-ff', 'pcm'])
#     print log_str
#     print 'subprocess returned in %f sec' % (time.time() - t1)
# except subprocess.CalledProcessError as e:
#     print 'CalledProcessError'
#     print 'Command:', ' '.join(e.cmd)
#     print 'Returncode:', e.returncode
#     print e.output
#
# pcm_feeder_thread.join()
# ani_receiver_thread.join()