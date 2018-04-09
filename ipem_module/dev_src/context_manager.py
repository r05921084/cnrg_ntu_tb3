#/usr/bin/python
# -*- coding: utf-8 -*-
from contextlib import contextmanager


@contextmanager
def open_file(name, mode):
    f = open(name, mode)

    yield f

    f.close()


if __name__ == "__main__":
    print open_file('file.txt', 'w')
    with open_file('file.txt', 'w') as f:
        f.write("Hello, world.")