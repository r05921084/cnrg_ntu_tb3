import numpy as np


x = np.arange(10)
print((x+1)%10)

# exit()
# image_buffer = np.zeros((50, 128))

buffer = np.zeros(5)

count = 0



while True:
    index = np.arange(5)
    # print(buffer)
    print('---',buffer[(index+count)%5])

    buffer[count%5]=count
    # print('-----------------')
    count +=1
    if count == 20:
        break
