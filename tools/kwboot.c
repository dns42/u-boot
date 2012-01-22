#include <stdio.h>
#include <string.h>
#include <libgen.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <assert.h>

static unsigned char bootrom_pat_boot[8] = {
    0xBB, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77
};

#if 0
static unsigned char bootrom_pat_debug[8] = {
    0xDD, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77
};
#endif

#define SOH  1
#define EOT  4
#define ACK  6
#define NAK 21
#define CAN 24

#define BOOTUA_TIMEO 50   /* ms */
#define XMODEM_TIMEO 1000 /* ms */

static int
kwboot_recv(int fd, void *buf, size_t len, int timeo)
{
    int rc, nfds;
    fd_set rfds;
    struct timeval tv;
    ssize_t n;

    rc = -1;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    tv.tv_sec = 0;
    tv.tv_usec = timeo * 1000;
    if (tv.tv_usec > 1000000) {
        tv.tv_sec += tv.tv_usec / 1000000;
        tv.tv_usec %= 1000000;
    }

    do {
        nfds = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (nfds < 0)
            goto out;

        n = read(fd, buf, len);
        if (n < 0)
            goto out;

        buf = (char*)buf + n;
        len -= n;
    } while (len > 0);

    rc = 0;
out:
    return rc;
}

static int
kwboot_send(int fd, const void *buf, size_t len)
{
    int rc;
    ssize_t n;

    rc = -1;

    do {
        n = write(fd, buf, len);
        if (n < 0)
            goto out;

        buf = (char*)buf + n;
        len -= n;
    } while (len > 0);

    rc = tcdrain(fd);
out:
    return rc;
}

static int
kwboot_open_tty(const char *path, speed_t speed)
{
    int rc, fd;
    struct termios tio;

    rc = -1;

    fd = open(path, O_RDWR|O_NOCTTY|O_NDELAY);
    if (fd < 0)
        goto out;

    memset(&tio, 0, sizeof(tio));

    tio.c_iflag = 0;
    tio.c_cflag = CREAD|CLOCAL|CS8;

    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 10;

    cfsetospeed(&tio, speed);
    cfsetispeed(&tio, speed);

    rc = tcsetattr(fd, TCSANOW, &tio);
    if (rc)
        goto out;

    rc = fd;
out:
    if (rc < 0) {
        if (fd >= 0)
            close(fd);
    }

    return rc;
}

static int
kwboot_bootmsg(int tty)
{
    int rc;
    char c;

    do {

        rc = tcflush(tty, TCIOFLUSH);
        if (rc)
            break;

        rc = kwboot_send(tty,
                         bootrom_pat_boot,
                         sizeof(bootrom_pat_boot));
        if (rc) {
            usleep(10000);
            continue;
        }

        rc = kwboot_recv(tty, &c, 1, BOOTUA_TIMEO);

    } while (rc || c != NAK);

    return rc;
}

static int
kwboot_xmodem_send(int fd, const void *buf, size_t len)
{
    int rc, retries;
    char c;

    retries = 15;
    do {
        rc = kwboot_send(fd, buf, len);
        if (rc)
            break;

        rc = kwboot_recv(fd, &c, 1, XMODEM_TIMEO);
        if (rc)
            break;

        rc = -1;

        switch (c) {
        case ACK:
            rc = 0;
            break;
        case NAK:
            if (!--retries) {
                errno = ECANCELED;
                goto out;
            }
            break;
        case CAN:
            errno = ESHUTDOWN;
            goto out;
        default:
            errno = EPROTO;
            goto out;
        }
    } while (c == NAK);

out:
    return rc;
}

static int
kwboot_xmodem(int tty, FILE *stream)
{
    struct {
        char soh;
        char pnum;
        char _pnum;
        char data[128];
        char csum;
    } block;
    const size_t blksz = sizeof(block.data);
    const char eot = EOT;
    int rc, n, i;

    rc = -1;

    block.soh = SOH;
    block.pnum = 1;

    do {
        block._pnum = ~block.pnum;

        n = 0;
        do {
            int _n;

            _n = fread(&block.data[n], 1, blksz - n, stream);
            if (!_n) {
                memset(&block.data[n], 0, blksz - n);
                break;
            }

            n += _n;
        } while (n < blksz);

        for (i = 0, block.csum = 0; i < n; i++)
            block.csum += block.data[i];

        rc = kwboot_xmodem_send(tty, &block, sizeof(block));
        if (rc)
            goto out;

        block.pnum += 1;

    } while (!feof(stream));

    rc = kwboot_xmodem_send(tty, &eot, 1);
out:
    return rc;
}

static void
kwboot_usage(FILE *stream, char *progname)
{
    fprintf(stream, "Usage: %s <tty> <image>\n", progname);
}

int
main(int argc, char **argv)
{
    const char *ttypath, *imgpath;
    int rv, rc, tty;
    FILE *img;

    rv = 1;
    img = 0;
    tty = -1;

    if (argc - optind < 2)
        goto usage;

    ttypath = argv[optind++];
    imgpath = argv[optind++];

    tty = kwboot_open_tty(ttypath, B115200);
    if (tty < 0)
        goto out;

    img = fopen(imgpath, "r");
    if (!img)
        goto out;

    rc = kwboot_bootmsg(tty);
    if (rc) {
        perror("bootmsg");
        goto out;
    }

    rc = kwboot_xmodem(tty, img);
    if (rc) {
        perror("xmodem");
        goto out;
    }

    rv = 0;
out:
    if (tty >= 0)
        close(tty);
    if (img)
        fclose(img);

    return rv;

usage:
    kwboot_usage(rv ? stderr : stdout, basename(argv[0]));
    goto out;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
