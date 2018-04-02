#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define BUFFSIZE 1024
#define err(mess)                            \
    {                                        \
        fprintf(stderr, "Error: %s.", mess); \
        exit(1);                             \
    }

int main()
{
    int fd, n;
    FILE *fp;
    char buf[BUFFSIZE];

    if ((fd = open("fifo_x", O_WRONLY)) < 0)
        err("open");

    if((fp = fdopen(fd, "w")) == NULL)
        err("fdopen");

    while ((n = read(STDIN_FILENO, buf, BUFFSIZE)) > 0)
    {
        if (fwrite(buf, sizeof(char), n, fp) != n)
        {
            err("write");
        }
        fflush(fp);
    }
    fclose(fp);
    close(fd);

    return 0;
}
