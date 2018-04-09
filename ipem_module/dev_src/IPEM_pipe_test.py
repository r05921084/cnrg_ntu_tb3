import os
import threading
import time
import subprocess

FIFO_PATH = '/tmp/ipem'
FIFO_PCM = 'ipem_pcm'
FIFO_ANI = 'ipem_ani'
FIFO_PCM_WITH_PATH = FIFO_PATH + '/' + FIFO_PCM
FIFO_ANI_WITH_PATH = FIFO_PATH + '/' + FIFO_ANI

try:
    os.mkdir(FIFO_PATH)
except OSError as oe:
    print oe

try:
    os.mkfifo(FIFO_PCM_WITH_PATH)
except OSError as oe:
    print oe

try:
    os.mkfifo(FIFO_ANI_WITH_PATH)
except OSError as oe:
    print oe


with open('input.wav', 'r') as wavfile:
    data = wavfile.read()
    print len(data[44:]), data[:44], '%x %x %x %x' % (ord(data[44]), ord(data[45]), ord(data[46]), ord(data[47]))
    data = data[44:]


def pcm_feeder():
    with open(FIFO_PCM_WITH_PATH, 'wb') as fifo_pcm:
        print("%s opened" % FIFO_PCM)

        # fifo_pcm.write(data)
        # fifo_pcm.flush()

        for i in range(len(data) // 2):
            fifo_pcm.write(data[i*2:i*2+2])
            # fifo_pcm.flush()
            # time.sleep(1.0 / 44100)


def ani_receiver():
    with open(FIFO_ANI_WITH_PATH, 'rb') as fifo_ani:
        print("%s opened" % FIFO_ANI)

        ani_str = 'EMPTY'

        while len(ani_str):
            ani_str = fifo_ani.readline()
            # print('READ(%d): %s' % (len(ani_str), ani_str))

        print('"%s" closed' % FIFO_ANI_WITH_PATH)


pcm_feeder_thread = threading.Thread(target=pcm_feeder)
ani_receiver_thread = threading.Thread(target=ani_receiver)

pcm_feeder_thread.start()
ani_receiver_thread.start()

try:
    t1 = time.time()
    log_str = subprocess.check_output(['./IPEMAuditoryModelConsole',
                                       '-nc', str(40),
                                       '-if', FIFO_PCM,
                                       '-id', FIFO_PATH,
                                       '-of', FIFO_ANI,
                                       '-od', FIFO_PATH,
                                       '-fs', str(44100),
                                       '-ff', 'pcm'])
    print log_str
    print 'subprocess returned in %f sec' % (time.time() - t1)
except subprocess.CalledProcessError as e:
    print 'CalledProcessError'
    print 'Command:', ' '.join(e.cmd)
    print 'Returncode:', e.returncode
    print e.output

pcm_feeder_thread.join()
ani_receiver_thread.join()