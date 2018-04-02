#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFSIZE 1024
#define err(mess)                            \
    {                                        \
        fprintf(stderr, "Error: %s.", mess); \
        exit(1);                             \
    }

int main()
{
    int fd, n;
    char buf[BUFFSIZE];

    if (mkfifo("fifo_x", 0666) < 0)
        err("mkfifo");

    if ((fd = open("fifo_x", O_RDONLY)) < 0)
        err("open");

    while ((n = read(fd, buf, BUFFSIZE)) > 0)
    {
        if (write(STDOUT_FILENO, buf, n) != n)
        {
            exit(1);
        }
    }
    close(fd);

    return 0;
}
