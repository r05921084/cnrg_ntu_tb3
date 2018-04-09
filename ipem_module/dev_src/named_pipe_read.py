import os

FIFO = 'fifo_x'

try:
    os.mkfifo(FIFO)
except OSError as oe:
    print oe

while True:
    print("Opening FIFO...")
    with open(FIFO) as fifo:
        print("FIFO opened")
        while True:
            data = fifo.read()
            if len(data) == 0:
                print("Writer closed")
                break
            print('Read: "{0}"'.format(data))