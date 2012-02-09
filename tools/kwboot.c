/*
 * Boot a Marvell Kirkwood SoC, with Xmodem over UART0.
 *
 * (c) 2012 Daniel Stodden <daniel.stodden@gmail.com>
 *
 * References: marvell.com, "88F6180, 88F6190, 88F6192, and 88F6281
 *   Integrated Controller: Functional Specifications" December 2,
 *   2008. Chapter 24.2 "BootROM Firmware".
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <libgen.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>

#ifdef __GNUC__
#define PACKED __attribute((packed))
#else
#define PACKED
#endif

/*
 * Marvell BootROM UART Sensing
 */

static unsigned char kwboot_msg_boot[] = {
    0xBB, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77
};

static unsigned char kwboot_msg_debug[] = {
    0xDD, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77
};

#define KWBOOT_MSG_REQ_DELAY 10  /* ms */
#define KWBOOT_MSG_RSP_TIMEO 50  /* ms */

/*
 * Xmodem Transfers
 */

#define SOH  1 /* sender start of block header */
#define EOT  4 /* sender end of block transfer */
#define ACK  6 /* target block ack */
#define NAK 21 /* target block negative ack */
#define CAN 24 /* target/sender transfer cancellation */

struct kwboot_block {
    uint8_t soh;
    uint8_t pnum;
    uint8_t _pnum;
    uint8_t data[128];
    uint8_t csum;
} PACKED;

#define KWBOOT_BLK_RSP_TIMEO 1000 /* ms */

static int kwboot_verbose = 0;

static void
kwboot_printv(const char *fmt, ...)
{
    va_list ap;

    if (kwboot_verbose) {
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
        fflush(stdout);
    }
}

static int
kwboot_tty_recv(int fd, void *buf, size_t len, int timeo)
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
kwboot_tty_send(int fd, const void *buf, size_t len)
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
kwboot_tty_send_char(int fd, unsigned char c)
{
    return kwboot_tty_send(fd, &c, 1);
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
kwboot_bootmsg(int tty, void *msg)
{
    int rc;
    char c;

    kwboot_printv("Sending boot message. "
                  "Please power/reset the target. ");

    do {
        rc = tcflush(tty, TCIOFLUSH);
        if (rc)
            break;

        rc = kwboot_tty_send(tty, msg, 8);
        if (rc) {
            usleep(KWBOOT_MSG_REQ_DELAY * 1000);
            continue;
        }

        rc = kwboot_tty_recv(tty, &c, 1,
                             KWBOOT_MSG_RSP_TIMEO);

        kwboot_printv(".");

    } while (rc || c != NAK);

    kwboot_printv("\n");

    return rc;
}

static ssize_t
kwboot_xm_readblock(struct kwboot_block *block, FILE *stream, int pnum)
{
    const size_t blksz = sizeof(block->data);
    ssize_t rc;
    size_t n;
    int i;

    block->pnum = pnum;

    n = 0;
    do {
        int _n;

        _n = fread(&block->data[n], 1, blksz - n, stream);
        if (!_n)
            break;

        n += _n;
    } while (n < blksz);

    if (n < blksz) {
        rc = ferror(stream) ? -1 : 0;
        if (rc)
            goto out;

        memset(&block->data[n], 0, blksz - n);
    }

    block->_pnum = ~block->pnum;

    for (i = 0, block->csum = 0; i < n; i++)
        block->csum += block->data[i];

    rc = n;
out:
    return rc;
}

static int
kwboot_xm_sendblock(int fd, struct kwboot_block *block)
{
    int rc, retries;
    char c;

    retries = 16;
    do {
        rc = kwboot_tty_send(fd, block, sizeof(*block));
        if (rc)
            break;

        rc = kwboot_tty_recv(fd, &c, 1, KWBOOT_BLK_RSP_TIMEO);
        if (rc)
            break;

        if (c == ACK)
            kwboot_printv(".");
        else
            kwboot_printv("+");

    } while (c == NAK && retries-- > 0);

    rc = c == ACK ? 0 : -1;

    switch (c) {
    case ACK:
        break;
    case NAK:
        errno = EBADMSG;
        break;
    case CAN:
        errno = ECANCELED;
        break;
    default:
        errno = EPROTO;
        break;
    }

    return rc;
}

static int
kwboot_xmodem(int tty, FILE *stream)
{
    int rc, pnum, err;
    ssize_t n;

    pnum = 1;

    kwboot_printv("Sending boot image. ");

    do {
        struct kwboot_block block;

        rc = -1;

        n = kwboot_xm_readblock(&block, stream, pnum++);
        if (n < 0)
            goto can;

        if (!n)
            break;

        rc = kwboot_xm_sendblock(tty, &block);
        if (rc)
            goto out;
    } while (1);

    rc = kwboot_tty_send_char(tty, EOT);

out:
    kwboot_printv("\n");
    return rc;

can:
    err = errno;
    kwboot_tty_send_char(tty, CAN);
    errno = err;
    goto out;
}

static int
kwboot_term_esc(const char *buf, int n, char *esc, int *s)
{
    int i;

    for (i = 0; i < n; i++) {
        if (*buf == esc[*s]) {
            (*s)++;
            if (!esc[*s])
                break;
        } else
            *s = 0;
    }

    return esc[*s] ? -1 : 0;
}

static int
kwboot_term_pipe(int in, int out, char *esc, int *s)
{
    ssize_t nin, nout;
    char buf[128];

    nin = read(in, buf, sizeof(buf));
    if (nin < 0)
        return -1;

    if (esc && !kwboot_term_esc(buf, nin, esc, s))
        return 0;

    while (nin > 0) {
        nout = write(out, buf, nin);
        if (nout <= 0)
            return -1;
        nin -= nout;
    }

    return 0;
}

static int
kwboot_terminal(int tty)
{
    int rc, in, s;
    char *esc = "\34c";
    struct termios otio, tio;

    rc = -1;

    in = STDIN_FILENO;
    if (isatty(in)) {
        rc = tcgetattr(in, &otio);
        if (!rc) {
            tio = otio;
            cfmakeraw(&tio);
            rc = tcsetattr(in, TCSANOW, &tio);
        }
        if (rc) {
            perror("tcsetattr");
            goto out;
        }

        kwboot_printv("[Type Ctrl-%c then %c to quit]\r\n",
                      esc[0]|0100, esc[1]);
    } else
        in = -1;

    rc = 0;
    s = 0;

    do {
        fd_set rfds;
        int nfds = 0;

        FD_SET(tty, &rfds);
        nfds = nfds < tty ? tty : nfds;

        if (in >= 0) {
            FD_SET(in, &rfds);
            nfds = nfds < in ? in : nfds;
        }

        nfds = select(nfds + 1, &rfds, NULL, NULL, NULL);
        if (nfds < 0)
            break;

        if (FD_ISSET(tty, &rfds)) {
            rc = kwboot_term_pipe(tty, STDOUT_FILENO, NULL, NULL);
            if (rc)
                break;
        }

        if (FD_ISSET(in, &rfds)) {
            rc = kwboot_term_pipe(in, tty, esc, &s);
            if (rc)
                break;
        }
    } while (esc[s] != 0);

    tcsetattr(in, TCSANOW, &otio);
out:
    return rc;
}

static void
kwboot_usage(FILE *stream, char *progname)
{
    fprintf(stream, "Usage: %s { -b <image> [-p] | -d } [ -t ] <tty>\n", progname);
    fprintf(stream, "\n");

    fprintf(stream, "  -b: boot <image>\n");
    fprintf(stream, "  -p: patch <image> to type 0x69 (uart boot)\n");
    fprintf(stream, "\n");

    fprintf(stream, "  -d: enter BootRom debug mode\n");
    fprintf(stream, "\n");

    fprintf(stream, "  -t: mini terminal\n");
    fprintf(stream, "\n");
}

int
main(int argc, char **argv)
{
    const char *ttypath, *imgpath;
    int rv, rc, tty, term;
    void *bootmsg;
    FILE *img;

    rv = 1;
    tty = -1;
    bootmsg = NULL;
    imgpath = NULL;
    img = NULL;
    term = 0;
    kwboot_verbose = isatty(STDOUT_FILENO);

    do {
        int c = getopt(argc, argv, "hb:dt");
        if (c < 0)
            break;

        switch (c) {
        case 'b':
            bootmsg = kwboot_msg_boot;
            imgpath = optarg;
            break;

        case 'd':
            bootmsg = kwboot_msg_debug;
            imgpath = NULL;
            break;

        case 't':
            term = 1;
            break;

        case 'h':
            rv = 0;
        default:
            goto usage;
        }
    } while (1);

    if (!bootmsg && !term)
        goto usage;

    if (argc - optind < 1)
        goto usage;

    ttypath = argv[optind++];

    tty = kwboot_open_tty(ttypath, B115200);
    if (tty < 0) {
        perror(ttypath);
        goto out;
    }

    if (imgpath) {
        img = fopen(imgpath, "r");
        if (!img) {
            perror(imgpath);
            goto out;
        }
    }

    if (bootmsg) {
        rc = kwboot_bootmsg(tty, bootmsg);
        if (rc) {
            perror("bootmsg");
            goto out;
        }
    }

    if (img) {
        rc = kwboot_xmodem(tty, img);
        if (rc) {
            perror("xmodem");
            goto out;
        }
    }

    if (term) {
        rc = kwboot_terminal(tty);
        if (rc && !(errno == EINTR)) {
            perror("terminal");
            goto out;
        }
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
