import subprocess
import os

def create_ramdisk(dir='/tmp/ramdisk', size='10M'):
    try:
        if os.path.isdir(dir):
            print('%s already exist.' % dir)
        else:
            subprocess.check_call(['mkdir', dir])
            print('mkdir %s... ok!' % dir)
        subprocess.check_call(['chmod', '777', dir])
        print('chmod 777 %s... ok!' % dir)
        subprocess.check_call(['mount', '-t', 'tmpfs', '-o', 'size=' + size, 'tmpfs', dir])
        print('mount -t tmpfs -o size=' + size + ' tmpfs %s... ok!' % dir)
        return True
    except subprocess.CalledProcessError:
        return False


if __name__ == '__main__':
    create_ramdisk(dir='/tmp/ramdisk3')