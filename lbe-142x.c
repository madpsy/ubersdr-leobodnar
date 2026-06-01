#define _DEFAULT_SOURCE
#define _GNU_SOURCE

//#include <iostream>
//using namespace std;

/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <dirent.h>
#include <pthread.h>
#include <signal.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <getopt.h>

#include <time.h>

//Leo LBE-142x defines, dont need anything external this time
#define VID_LBE		0x1dd2

#define PID_LBE_1420	0x2443
#define PID_LBE_1421	0xffff


#define GPS_LOCK_BIT		0x01
#define PLL_LOCK_BIT		0x02
#define ANT_OK_BIT		0x04
#define LED1_BIT		0x08
#define OUT1_EN_BIT		0x10
#define OUT2_EN_BIT		0x20
#define PPS1_BIT		0x40
//SetFeatureReportIDs
#define LBE_1420_EN_OUT1	0x01
#define LBE_1420_BLINK_OUT1	0x02
#define LBE_1420_SET_F1_NO_SAVE	0x03
#define LBE_1420_SET_F1		0x04
#define LBE_1420_SET_PWR1	0x07

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 */
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

#define HIDIOCGRAWNAME(len)     _IOC(_IOC_READ, 'H', 0x04, len)

/* NMEA sentence types we care about */
#define NMEA_TIMEOUT_SEC  5
#define NMEA_LINE_MAX     256

typedef struct {
    int    fix_quality;      /* 0=no fix, 1=GPS, 2=DGPS */
    int    fix_mode;         /* 1=no fix, 2=2D, 3=3D (from GSA) */
    int    sats_used;        /* satellites used in fix */
    double hdop;             /* horizontal dilution of precision */
    double vdop;             /* vertical dilution of precision (from GSA) */
    double pdop;             /* position dilution of precision (from GSA) */
    double altitude_m;       /* altitude in metres */
    int    gps_sats_view;    /* GPS satellites in view */
    int    glo_sats_view;    /* GLONASS satellites in view */
    double speed_knots;      /* speed over ground in knots (from VTG) */
    int    has_speed;        /* 1 if speed is valid */
    /* position/time from RMC */
    double lat_deg;          /* latitude in decimal degrees (negative = S) */
    double lon_deg;          /* longitude in decimal degrees (negative = W) */
    int    has_position;     /* 1 if position is valid */
    char   utc_time[16];     /* UTC time string "HH:MM:SS" */
    char   utc_date[16];     /* UTC date string "DD/MM/YYYY" */
    char   iso_datetime[32]; /* ISO 8601 "YYYY-MM-DDTHH:MM:SSZ" */
    int    valid;            /* 1 if we got a GGA sentence */
} NmeaInfo;

/* Convert NMEA ddmm.mmmm to decimal degrees */
static double nmea_to_deg(const char *val, char hemi)
{
    double raw = atof(val);
    int deg = (int)(raw / 100);
    double min = raw - deg * 100.0;
    double result = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') result = -result;
    return result;
}

/* Parse a $GNGGA or $GPGGA sentence into NmeaInfo */
static void parse_gga(const char *line, NmeaInfo *info)
{
    /* $GNGGA,hhmmss.ss,lat,N,lon,E,quality,sats,hdop,alt,M,... */
    char tmp[NMEA_LINE_MAX];
    strncpy(tmp, line, NMEA_LINE_MAX - 1);
    tmp[NMEA_LINE_MAX - 1] = '\0';

    char *tok = strtok(tmp, ",");
    int field = 0;
    while (tok) {
        switch (field) {
            case 6: info->fix_quality = atoi(tok); break;
            case 7: info->sats_used   = atoi(tok); break;
            case 8: info->hdop        = atof(tok); break;
            case 9: info->altitude_m  = atof(tok); break;
        }
        field++;
        tok = strtok(NULL, ",");
    }
    info->valid = 1;
}

/* Split a comma-separated string into fields, preserving empty fields.
 * Returns number of fields found. fields[] must have capacity for max_fields. */
static int split_csv(const char *line, char fields[][NMEA_LINE_MAX], int max_fields)
{
    int n = 0;
    const char *p = line;
    while (n < max_fields) {
        const char *comma = strchr(p, ',');
        size_t len = comma ? (size_t)(comma - p) : strlen(p);
        if (len >= NMEA_LINE_MAX) len = NMEA_LINE_MAX - 1;
        strncpy(fields[n], p, len);
        fields[n][len] = '\0';
        n++;
        if (!comma) break;
        p = comma + 1;
    }
    return n;
}

/* Parse a $GNRMC or $GPRMC sentence for position, time and date */
static void parse_rmc(const char *line, NmeaInfo *info)
{
    /* $GNRMC,hhmmss.ss,A,lat,N,lon,W,speed,course,ddmmyy,... */
    char fields[16][NMEA_LINE_MAX];
    int nf = split_csv(line, fields, 16);

    char *time_str = (nf > 1) ? fields[1] : "";
    char  status   = (nf > 2 && fields[2][0]) ? fields[2][0] : 'V';
    char *lat_str  = (nf > 3) ? fields[3] : "";
    char  lat_hemi = (nf > 4 && fields[4][0]) ? fields[4][0] : 'N';
    char *lon_str  = (nf > 5) ? fields[5] : "";
    char  lon_hemi = (nf > 6 && fields[6][0]) ? fields[6][0] : 'E';
    char *date_str = (nf > 9) ? fields[9] : "";

    /* Format time as HH:MM:SS from hhmmss.ss */
    if (time_str[0] && strlen(time_str) >= 6) {
        snprintf(info->utc_time, sizeof(info->utc_time),
                 "%c%c:%c%c:%c%c UTC",
                 time_str[0], time_str[1],
                 time_str[2], time_str[3],
                 time_str[4], time_str[5]);
    }

    /* Format date as DD/MM/YYYY from ddmmyy */
    if (date_str[0] && strlen(date_str) >= 6) {
        snprintf(info->utc_date, sizeof(info->utc_date),
                 "%c%c/%c%c/20%c%c",
                 date_str[0], date_str[1],
                 date_str[2], date_str[3],
                 date_str[4], date_str[5]);
    }

    /* Build ISO 8601 datetime: YYYY-MM-DDTHH:MM:SSZ */
    if (time_str[0] && strlen(time_str) >= 6 &&
        date_str[0] && strlen(date_str) >= 6) {
        snprintf(info->iso_datetime, sizeof(info->iso_datetime),
                 "20%c%c-%c%c-%c%cT%c%c:%c%c:%c%cZ",
                 date_str[4], date_str[5],   /* year */
                 date_str[2], date_str[3],   /* month */
                 date_str[0], date_str[1],   /* day */
                 time_str[0], time_str[1],   /* hour */
                 time_str[2], time_str[3],   /* minute */
                 time_str[4], time_str[5]);  /* second */
    }

    if (status == 'A' && lat_str[0] && lon_str[0]) {
        info->lat_deg      = nmea_to_deg(lat_str, lat_hemi);
        info->lon_deg      = nmea_to_deg(lon_str, lon_hemi);
        info->has_position = 1;
    }
}

/* Parse a $GPGSV or $GLGSV sentence — read total sats in view from first message */
static void parse_gsv(const char *line, int *sats_view)
{
    /* $GPGSV,total_msgs,msg_num,sats_in_view,... */
    char tmp[NMEA_LINE_MAX];
    strncpy(tmp, line, NMEA_LINE_MAX - 1);
    tmp[NMEA_LINE_MAX - 1] = '\0';

    char *tok = strtok(tmp, ",");
    int field = 0;
    while (tok) {
        if (field == 3) {
            *sats_view = atoi(tok);
            break;
        }
        field++;
        tok = strtok(NULL, ",");
    }
}

/* Parse a $GNGSA sentence — fix mode and DOP values.
 * Only update on the first GSA sentence per cycle (system ID 1 = GPS).
 * $GNGSA,A,fixmode,sv,sv,...,pdop,hdop,vdop,systemID */
static void parse_gsa(const char *line, NmeaInfo *info)
{
    char fields[20][NMEA_LINE_MAX];
    int nf = split_csv(line, fields, 20);
    /* field 2 = fix mode (1/2/3) */
    if (nf > 2 && fields[2][0])
        info->fix_mode = atoi(fields[2]);
    /* fields 15,16,17 = PDOP, HDOP, VDOP */
    if (nf > 15 && fields[15][0]) info->pdop = atof(fields[15]);
    if (nf > 16 && fields[16][0]) info->hdop = atof(fields[16]);
    if (nf > 17 && fields[17][0]) info->vdop = atof(fields[17]);
}

/* Parse a $GNVTG sentence — speed over ground.
 * $GNVTG,course_T,T,course_M,M,speed_N,N,speed_K,K,mode */
static void parse_vtg(const char *line, NmeaInfo *info)
{
    char fields[10][NMEA_LINE_MAX];
    int nf = split_csv(line, fields, 10);
    /* field 5 = speed in knots */
    if (nf > 5 && fields[5][0]) {
        info->speed_knots = atof(fields[5]);
        info->has_speed   = 1;
    }
}

/*
 * Open serial port, read NMEA for up to NMEA_TIMEOUT_SEC seconds,
 * parse GGA + GSV sentences, fill NmeaInfo.
 * Returns 0 on success, -1 on error.
 */
static int read_nmea(const char *port, NmeaInfo *info)
{
    memset(info, 0, sizeof(*info));

    int sfd = open(port, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (sfd < 0) {
        perror("    Unable to open serial port");
        return -1;
    }

    /* Switch to blocking mode now that the port is open */
    int flags = fcntl(sfd, F_GETFL, 0);
    if (flags >= 0)
        fcntl(sfd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty;
    if (tcgetattr(sfd, &tty) != 0) {
        perror("    tcgetattr");
        close(sfd);
        return -1;
    }
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag  = IGNBRK;
    tty.c_lflag  = 0;
    tty.c_oflag  = 0;
    tcsetattr(sfd, TCSANOW, &tty);

    /* Set back to non-blocking for the read loop (we use select) */
    fcntl(sfd, F_SETFL, flags | O_NONBLOCK);

    char linebuf[NMEA_LINE_MAX];
    int  linelen = 0;

    int got_gga      = 0;
    int got_gps_gsv  = 0;
    int got_glo_gsv  = 0;

    time_t deadline = time(NULL) + NMEA_TIMEOUT_SEC;

    while (time(NULL) < deadline) {
        /* Wait up to 500ms for data */
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sfd, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };
        if (select(sfd + 1, &rfds, NULL, NULL, &tv) <= 0)
            continue;

        char ch;
        while (read(sfd, &ch, 1) == 1) {
            if (ch == '\r') continue;
            if (ch == '\n') {
                linebuf[linelen] = '\0';

                /* Strip checksum (*XX) before parsing */
                char *star = strchr(linebuf, '*');
                if (star) *star = '\0';

                if (strncmp(linebuf, "$GNGGA", 6) == 0 ||
                    strncmp(linebuf, "$GPGGA", 6) == 0) {
                    parse_gga(linebuf, info);
                    got_gga = 1;
                } else if (strncmp(linebuf, "$GNRMC", 6) == 0 ||
                           strncmp(linebuf, "$GPRMC", 6) == 0) {
                    parse_rmc(linebuf, info);
                } else if (strncmp(linebuf, "$GNGSA", 6) == 0 ||
                           strncmp(linebuf, "$GPGSA", 6) == 0) {
                    parse_gsa(linebuf, info);
                } else if (strncmp(linebuf, "$GNVTG", 6) == 0 ||
                           strncmp(linebuf, "$GPVTG", 6) == 0) {
                    parse_vtg(linebuf, info);
                } else if (strncmp(linebuf, "$GPGSV", 6) == 0) {
                    /* Only read first message of the sequence for total count */
                    char tmp2[NMEA_LINE_MAX];
                    strncpy(tmp2, linebuf, NMEA_LINE_MAX - 1);
                    char *t = strtok(tmp2, ",");
                    int f = 0; int msg_num = 0;
                    while (t) { if (f == 2) { msg_num = atoi(t); break; } f++; t = strtok(NULL, ","); }
                    if (msg_num == 1) {
                        parse_gsv(linebuf, &info->gps_sats_view);
                        got_gps_gsv = 1;
                    }
                } else if (strncmp(linebuf, "$GLGSV", 6) == 0) {
                    char tmp2[NMEA_LINE_MAX];
                    strncpy(tmp2, linebuf, NMEA_LINE_MAX - 1);
                    char *t = strtok(tmp2, ",");
                    int f = 0; int msg_num = 0;
                    while (t) { if (f == 2) { msg_num = atoi(t); break; } f++; t = strtok(NULL, ","); }
                    if (msg_num == 1) {
                        parse_gsv(linebuf, &info->glo_sats_view);
                        got_glo_gsv = 1;
                    }
                }

                linelen = 0;

                if (got_gga && got_gps_gsv && got_glo_gsv)
                    goto done;
            } else {
                if (linelen < NMEA_LINE_MAX - 1)
                    linebuf[linelen++] = ch;
            }
        }
    }

done:
    close(sfd);
    return (got_gga ? 0 : -1);
}


/* -----------------------------------------------------------------------
 * Device auto-discovery
 * ----------------------------------------------------------------------- */

#define MAX_DEVICES 16

typedef struct {
    char hidraw_path[64];   /* e.g. /dev/hidraw0 */
    char serial_path[64];   /* e.g. /dev/ttyACM0, or "" if not found */
    char name[128];
} LbeDevice;

/*
 * Given a hidraw node name (e.g. "hidraw0"), try to find the paired
 * ttyACM device by walking sysfs:
 *   /sys/class/hidraw/hidrawN/device -> USB interface
 *   go up two levels to USB device, then look for tty in all interfaces
 */
static void find_paired_serial(const char *hidraw_name, char *out, size_t outsz)
{
    out[0] = '\0';

    /*
     * /sys/class/hidraw/hidrawN/device is a symlink to the HID interface
     * (e.g. 0003:1DD2:2443.0001). We need to go up two levels:
     *   device/..   = USB interface (e.g. 1-3:1.2)
     *   device/../.. = USB device   (e.g. 1-3)
     * then scan all USB interfaces under the USB device for a tty/ subdir.
     *
     * Use the canonical path via /sys/class/hidraw/hidrawN/device/../..
     */
    char usbdev_rel[256];
    snprintf(usbdev_rel, sizeof(usbdev_rel),
             "/sys/class/hidraw/%s/device/../..", hidraw_name);

    /* Resolve to absolute path */
    char usbdev[1024];
    if (realpath(usbdev_rel, usbdev) == NULL) return;

    /* Scan the USB device directory for interface dirs containing tty/ */
    DIR *d = opendir(usbdev);
    if (!d) return;

    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;

        char ttydir[1536];
        snprintf(ttydir, sizeof(ttydir), "%s/%s/tty", usbdev, ent->d_name);

        DIR *td = opendir(ttydir);
        if (!td) continue;

        struct dirent *tent;
        while ((tent = readdir(td)) != NULL) {
            if (tent->d_name[0] == '.') continue;
            /* Found a tty entry, e.g. "ttyACM0" */
            snprintf(out, outsz, "/dev/%s", tent->d_name);
            closedir(td);
            closedir(d);
            return;
        }
        closedir(td);
    }
    closedir(d);
}

/*
 * Scan /dev/hidraw0..hidraw15, check VID/PID, collect matching devices.
 * Returns number of devices found.
 */
static int find_lbe_devices(LbeDevice *devs, int maxdevs)
{
    int count = 0;
    for (int i = 0; i < 16 && count < maxdevs; i++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/hidraw%d", i);

        int fd = open(path, O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;

        struct hidraw_devinfo info;
        memset(&info, 0, sizeof(info));
        if (ioctl(fd, HIDIOCGRAWINFO, &info) < 0) {
            close(fd);
            continue;
        }

        if (info.vendor == VID_LBE &&
            (info.product == PID_LBE_1420 || info.product == PID_LBE_1421)) {

            strncpy(devs[count].hidraw_path, path, sizeof(devs[count].hidraw_path) - 1);

            /* Get device name */
            u_int8_t nbuf[256] = {0};
            if (ioctl(fd, HIDIOCGRAWNAME(256), nbuf) >= 0)
                strncpy(devs[count].name, (char *)nbuf, sizeof(devs[count].name) - 1);
            else
                strncpy(devs[count].name, "Unknown", sizeof(devs[count].name) - 1);

            /* Try to find paired serial port */
            char hidraw_name[16];
            snprintf(hidraw_name, sizeof(hidraw_name), "hidraw%d", i);
            find_paired_serial(hidraw_name, devs[count].serial_path,
                               sizeof(devs[count].serial_path));

            count++;
        }
        close(fd);
    }
    return count;
}

/* -----------------------------------------------------------------------
 * Serve mode — shared state, background threads, HTTP server
 * ----------------------------------------------------------------------- */
#define HTTP_DEFAULT_PORT 5123

typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t  new_data;  /* broadcast by NMEA thread on each $GNGGA */

    /* Device paths — updated by monitor thread under lock */
    char hidraw_path[64];
    char serial_port[64];
    int  serial_changed;   /* set by monitor, cleared by NMEA thread */

    /* HID state — updated by NMEA thread on each $GNGGA (1Hz) */
    uint8_t  hid_status;
    uint8_t  hid_fll;
    uint8_t  hid_out1low;
    uint32_t frequency_hz;
    int      hid_valid;

    /* NMEA state (updated continuously from serial streaming thread) */
    NmeaInfo nmea;
    int      nmea_valid;
} ServerState;

/* ---- Device monitor thread: re-scans every 5s, updates paths silently - */
static void *device_monitor_thread(void *arg)
{
    ServerState *st = (ServerState *)arg;
    while (1) {
        LbeDevice devs[MAX_DEVICES];
        int ndevs = find_lbe_devices(devs, MAX_DEVICES);
        pthread_mutex_lock(&st->lock);
        if (ndevs > 0) {
            if (strcmp(st->hidraw_path, devs[0].hidraw_path) != 0)
                strncpy(st->hidraw_path, devs[0].hidraw_path,
                        sizeof(st->hidraw_path) - 1);
            if (strcmp(st->serial_port, devs[0].serial_path) != 0) {
                strncpy(st->serial_port, devs[0].serial_path,
                        sizeof(st->serial_port) - 1);
                st->serial_changed = 1;
                memset(&st->nmea, 0, sizeof(st->nmea));
                st->nmea_valid = 0;
            }
        } else if (st->hidraw_path[0]) {
            st->hidraw_path[0] = '\0';
            st->serial_port[0] = '\0';
            st->serial_changed = 1;
            memset(&st->nmea, 0, sizeof(st->nmea));
            st->nmea_valid = 0;
        }
        pthread_mutex_unlock(&st->lock);
        sleep(5);
    }
    return NULL;
}

/* ---- NMEA streaming thread: reads serial port continuously ------------ */
static void *nmea_stream_thread(void *arg)
{
    ServerState *st = (ServerState *)arg;
    char cur_port[64] = {0};

wait_for_port:
    /* Wait until monitor thread provides a serial path */
    while (1) {
        pthread_mutex_lock(&st->lock);
        strncpy(cur_port, st->serial_port, sizeof(cur_port) - 1);
        st->serial_changed = 0;
        pthread_mutex_unlock(&st->lock);
        if (cur_port[0]) break;
        sleep(1);
    }

retry:;
    int sfd = open(cur_port, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (sfd < 0) {
        sleep(2);
        pthread_mutex_lock(&st->lock);
        int chg = st->serial_changed;
        if (chg) strncpy(cur_port, st->serial_port, sizeof(cur_port) - 1);
        st->serial_changed = 0;
        pthread_mutex_unlock(&st->lock);
        if (chg) goto wait_for_port;
        goto retry;
    }

    /* Configure serial port */
    int flags = fcntl(sfd, F_GETFL, 0);
    if (flags >= 0) fcntl(sfd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty;
    if (tcgetattr(sfd, &tty) == 0) {
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_iflag  = IGNBRK;
        tty.c_lflag  = 0;
        tty.c_oflag  = 0;
        tcsetattr(sfd, TCSANOW, &tty);
    }
    if (flags >= 0) fcntl(sfd, F_SETFL, flags | O_NONBLOCK);

    char linebuf[NMEA_LINE_MAX];
    int  linelen = 0;

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sfd, &rfds);
        struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
        int sel = select(sfd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) break;   /* fd gone — reopen */
        if (sel == 0) continue;

        char ch;
        while (read(sfd, &ch, 1) == 1) {
            if (ch == '\r') continue;
            if (ch == '\n') {
                linebuf[linelen] = '\0';
                linelen = 0;

                /* Strip checksum */
                char *star = strchr(linebuf, '*');
                if (star) *star = '\0';

                /* For GGA we need to drop the lock to do HID I/O, so handle
                 * locking manually; all other sentences use the common path. */
                int is_gga = (strncmp(linebuf, "$GNGGA", 6) == 0 ||
                              strncmp(linebuf, "$GPGGA", 6) == 0);

                if (is_gga) {
                    /* Parse GGA under lock, snapshot HID path, then unlock */
                    pthread_mutex_lock(&st->lock);
                    parse_gga(linebuf, &st->nmea);
                    st->nmea_valid = 1;
                    char hid_snap[64];
                    strncpy(hid_snap, st->hidraw_path, sizeof(hid_snap) - 1);
                    hid_snap[sizeof(hid_snap)-1] = '\0';
                    pthread_mutex_unlock(&st->lock);

                    /* Read HID outside the lock — no blocking with mutex held */
                    uint8_t  h_status = 0, h_fll = 0, h_out1low = 0;
                    uint32_t h_freq   = 0;
                    int      h_valid  = 0;
                    if (hid_snap[0]) {
                        int hfd = open(hid_snap, O_RDWR | O_NONBLOCK);
                        if (hfd >= 0) {
                            u_int8_t hbuf[60];
                            hbuf[0] = 0x1;  /* Report 0x01: status + frequency */
                            int hres = ioctl(hfd, HIDIOCGFEATURE(256), hbuf);
                            if (hres >= 0 && hbuf[0] == 0x01) {
                                h_status  = hbuf[1];
                                h_fll     = hbuf[18];
                                h_out1low = hbuf[10]; /* out1_power_low at byte 10 */
                                /* frequency is at bytes 6-9 little-endian */
                                h_freq    = ((uint32_t)hbuf[9] << 24) |
                                            ((uint32_t)hbuf[8] << 16) |
                                            ((uint32_t)hbuf[7] <<  8) |
                                             (uint32_t)hbuf[6];
                                h_valid   = 1;
                            }
                            close(hfd);
                        }
                    }

                    /* Re-lock, store HID results, broadcast to SSE clients */
                    pthread_mutex_lock(&st->lock);
                    st->hid_status   = h_status;
                    st->hid_fll      = h_fll;
                    st->hid_out1low  = h_out1low;
                    st->frequency_hz = h_freq;
                    st->hid_valid    = h_valid;
                    pthread_cond_broadcast(&st->new_data);
                    pthread_mutex_unlock(&st->lock);
                } else {
                    /* All other sentences: common lock/parse/unlock */
                    pthread_mutex_lock(&st->lock);
                    if (strncmp(linebuf, "$GNRMC", 6) == 0 ||
                               strncmp(linebuf, "$GPRMC", 6) == 0) {
                        parse_rmc(linebuf, &st->nmea);
                    } else if (strncmp(linebuf, "$GNGSA", 6) == 0 ||
                               strncmp(linebuf, "$GPGSA", 6) == 0) {
                        parse_gsa(linebuf, &st->nmea);
                    } else if (strncmp(linebuf, "$GNVTG", 6) == 0 ||
                               strncmp(linebuf, "$GPVTG", 6) == 0) {
                        parse_vtg(linebuf, &st->nmea);
                    } else if (strncmp(linebuf, "$GPGSV", 6) == 0) {
                        char tmp2[NMEA_LINE_MAX];
                        strncpy(tmp2, linebuf, NMEA_LINE_MAX - 1);
                        char *t = strtok(tmp2, ",");
                        int f = 0, msg_num = 0;
                        while (t) { if (f == 2) { msg_num = atoi(t); break; } f++; t = strtok(NULL, ","); }
                        if (msg_num == 1) parse_gsv(linebuf, &st->nmea.gps_sats_view);
                    } else if (strncmp(linebuf, "$GLGSV", 6) == 0) {
                        char tmp2[NMEA_LINE_MAX];
                        strncpy(tmp2, linebuf, NMEA_LINE_MAX - 1);
                        char *t = strtok(tmp2, ",");
                        int f = 0, msg_num = 0;
                        while (t) { if (f == 2) { msg_num = atoi(t); break; } f++; t = strtok(NULL, ","); }
                        if (msg_num == 1) parse_gsv(linebuf, &st->nmea.glo_sats_view);
                    }
                    pthread_mutex_unlock(&st->lock);
                }
            } else {
                if (linelen < NMEA_LINE_MAX - 1)
                    linebuf[linelen++] = ch;
            }
        }
    }

    close(sfd);
    goto retry;
    return NULL;
}

/* ---- Serialise cached state to a malloc'd JSON string ----------------- */
/* device_present is set to 1 if the HID device was readable, 0 otherwise */
static char *state_to_json(ServerState *st, int *device_present)
{
    /* Snapshot everything under the lock */
    pthread_mutex_lock(&st->lock);
    char     hidraw_snap[64];
    char     serial_snap[64];
    strncpy(hidraw_snap, st->hidraw_path, sizeof(hidraw_snap) - 1);
    hidraw_snap[sizeof(hidraw_snap)-1] = '\0';
    strncpy(serial_snap, st->serial_port, sizeof(serial_snap) - 1);
    serial_snap[sizeof(serial_snap)-1] = '\0';
    uint8_t  hid_status  = st->hid_status;
    uint8_t  hid_fll     = st->hid_fll;
    uint8_t  hid_out1low = st->hid_out1low;
    uint32_t freq        = st->frequency_hz;
    int      hid_valid   = st->hid_valid;
    NmeaInfo nmea        = st->nmea;
    int      nmea_valid  = st->nmea_valid;
    pthread_mutex_unlock(&st->lock);

    *device_present = hid_valid;

    size_t cap = 4096;
    char *out = malloc(cap);
    if (!out) return NULL;
    size_t pos = 0;

/* JCAT: append formatted text; clamps pos so it never exceeds cap,
 * preventing size_t underflow in (cap - pos) on subsequent calls. */
#define JCAT(...) do { \
    if (pos < cap) { \
        int _n = snprintf(out + pos, cap - pos, __VA_ARGS__); \
        if (_n > 0) pos += (size_t)(_n < (int)(cap - pos) ? _n : (int)(cap - pos - 1)); \
    } \
} while(0)

    JCAT("{\n");
    if (hidraw_snap[0])
        JCAT("  \"device\": \"%s\",\n", hidraw_snap);
    else
        JCAT("  \"device\": null,\n");
    if (serial_snap[0])
        JCAT("  \"serial\": \"%s\",\n", serial_snap);
    else
        JCAT("  \"serial\": null,\n");

    if (hid_valid) {
        JCAT("  \"device_status\": {\n");
        JCAT("    \"gps_lock\":        %s,\n", (hid_status & GPS_LOCK_BIT) ? "true" : "false");
        JCAT("    \"pll_lock\":        %s,\n", (hid_status & PLL_LOCK_BIT) ? "true" : "false");
        JCAT("    \"antenna_ok\":      %s,\n", (hid_status & ANT_OK_BIT)   ? "true" : "false");
        JCAT("    \"mode\":            \"%s\",\n", hid_fll ? "FLL" : "PLL");
        JCAT("    \"output1_enabled\": %s,\n", (hid_status & OUT1_EN_BIT) ? "true" : "false");
        JCAT("    \"output1_pps\":     %s,\n", (hid_status & PPS1_BIT)    ? "true" : "false");
        JCAT("    \"output1_drive\":   \"%s\",\n", hid_out1low ? "low" : "normal");
        JCAT("    \"frequency_hz\":    %u\n",  freq);
        JCAT("  }");
    } else {
        JCAT("  \"device_status\": null");
    }

    if (nmea_valid) {
        const char *fix_str;
        switch (nmea.fix_quality) {
            case 0:  fix_str = "none";    break;
            case 1:  fix_str = "GPS";     break;
            case 2:  fix_str = "DGPS";    break;
            default: fix_str = "unknown"; break;
        }
        const char *fix_mode_str;
        switch (nmea.fix_mode) {
            case 2:  fix_mode_str = "2D"; break;
            case 3:  fix_mode_str = "3D"; break;
            default: fix_mode_str = "none"; break;
        }
        JCAT(",\n  \"gps\": {\n");
        if (nmea.iso_datetime[0]) JCAT("    \"datetime_utc\": \"%s\",\n", nmea.iso_datetime);
        JCAT("    \"fix\":          \"%s\",\n",  fix_str);
        JCAT("    \"fix_mode\":     \"%s\",\n",  fix_mode_str);
        JCAT("    \"sats_used\":    %d,\n",      nmea.sats_used);
        JCAT("    \"gps_in_view\":  %d,\n",      nmea.gps_sats_view);
        JCAT("    \"glo_in_view\":  %d,\n",      nmea.glo_sats_view);
        JCAT("    \"hdop\":         %.2f,\n",    nmea.hdop);
        JCAT("    \"vdop\":         %.2f,\n",    nmea.vdop);
        JCAT("    \"pdop\":         %.2f,\n",    nmea.pdop);
        JCAT("    \"altitude_m\":   %.1f",       nmea.altitude_m);
        if (nmea.has_speed)
            JCAT(",\n    \"speed_knots\":  %.3f", nmea.speed_knots);
        if (nmea.has_position) {
            JCAT(",\n    \"latitude\":     %.7f", nmea.lat_deg);
            JCAT(",\n    \"longitude\":    %.7f", nmea.lon_deg);
        }
        JCAT("\n  }");
    } else {
        JCAT(",\n  \"gps\": null");
    }
    JCAT("\n}\n");

#undef JCAT
    return out;
}

/* ---- Minimal JSON body helpers ---------------------------------------- */

/* Extract a numeric value for "key" from a JSON object string.
 * Returns 1 and sets *val on success, 0 if key not found. */
static int json_get_long(const char *body, const char *key, long *val)
{
    /* Search for "key": <number> */
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(body, needle);
    if (!p) return 0;
    p += strlen(needle);
    while (*p == ' ' || *p == '\t' || *p == ':' || *p == ' ') p++;
    if (*p == '\0') return 0;
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) return 0;
    *val = v;
    return 1;
}

/* Extract a boolean value for "key" (true/false) from a JSON object string.
 * Returns 1 and sets *val (1=true, 0=false) on success, 0 if not found. */
static int json_get_bool(const char *body, const char *key, int *val)
{
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *p = strstr(body, needle);
    if (!p) return 0;
    p += strlen(needle);
    while (*p == ' ' || *p == '\t' || *p == ':' || *p == ' ') p++;
    if (strncmp(p, "true",  4) == 0) { *val = 1; return 1; }
    if (strncmp(p, "false", 5) == 0) { *val = 0; return 1; }
    return 0;
}

/* ---- HID write helper ------------------------------------------------- */
/* Opens hidraw_path, sends a 60-byte feature report, closes it.
 * Returns 0 on success, -1 on open failure, -2 on ioctl failure.
 * On failure, errbuf (size errbuf_sz) is filled with a description. */
static int hid_write_op(const char *hidraw_path,
                        uint8_t buf[60],
                        char *errbuf, size_t errbuf_sz)
{
    if (!hidraw_path || !hidraw_path[0]) {
        snprintf(errbuf, errbuf_sz, "device not present");
        return -1;
    }
    int fd = open(hidraw_path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        snprintf(errbuf, errbuf_sz, "open %s: %s", hidraw_path, strerror(errno));
        return -1;
    }
    int res = ioctl(fd, HIDIOCSFEATURE(60), buf);
    close(fd);
    if (res < 0) {
        snprintf(errbuf, errbuf_sz, "ioctl: %s", strerror(errno));
        return -2;
    }
    return 0;
}

/* ---- Send a simple JSON API response ---------------------------------- */
static void send_api_response(int cfd, int http_ok, int ok, const char *msg)
{
    char body[256];
    int blen;
    if (ok)
        blen = snprintf(body, sizeof(body), "{\"ok\":true}\n");
    else
        blen = snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}\n", msg);

    const char *status = http_ok ? "HTTP/1.1 200 OK" :
                         (http_ok == 0) ? "HTTP/1.1 400 Bad Request" :
                                          "HTTP/1.1 503 Service Unavailable";
    /* Reuse http_ok==0 for 400, http_ok==-1 for 503, http_ok==-2 for 500 */
    if      (http_ok ==  0) status = "HTTP/1.1 400 Bad Request";
    else if (http_ok == -1) status = "HTTP/1.1 503 Service Unavailable";
    else if (http_ok == -2) status = "HTTP/1.1 500 Internal Server Error";
    else                    status = "HTTP/1.1 200 OK";

    char header[256];
    int hlen = snprintf(header, sizeof(header),
        "%s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n"
        "\r\n",
        status, blen);
    write(cfd, header, hlen);
    write(cfd, body, blen);
}

/* ---- SSE client thread: streams JSON events on each $GNGGA ----------- */
typedef struct { ServerState *st; int fd; } SseArg;

static void *sse_client_thread(void *arg)
{
    SseArg *a = (SseArg *)arg;
    ServerState *st = a->st;
    int cfd = a->fd;
    free(a);

    /* Send SSE headers */
    static const char sse_hdr[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: keep-alive\r\n"
        "\r\n";
    if (send(cfd, sse_hdr, sizeof(sse_hdr) - 1, MSG_NOSIGNAL) < 0) {
        close(cfd); return NULL;
    }

    while (1) {
        /* Wait for next $GNGGA broadcast */
        pthread_mutex_lock(&st->lock);
        pthread_cond_wait(&st->new_data, &st->lock);
        pthread_mutex_unlock(&st->lock);

        /* Snapshot state */
        int device_present = 0;
        char *json = state_to_json(st, &device_present);
        if (!json) continue;

        /* SSE framing per RFC 8895 §9.2:
         * Each line of the payload must be sent as "data: <line>\n".
         * The event is terminated by a blank line "\n".
         * This allows multi-line (pretty-printed) JSON to be parsed
         * correctly by the browser's EventSource API. */
        int ok = 1;
        char *p = json;
        while (ok) {
            char *nl = strchr(p, '\n');
            size_t llen = nl ? (size_t)(nl - p) : strlen(p);

            /* Skip empty trailing line produced by trailing \n in JSON */
            if (llen == 0 && nl == NULL) break;

            ssize_t w;
            w = send(cfd, "data: ", 6, MSG_NOSIGNAL); if (w < 0) { ok = 0; break; }
            if (llen > 0) {
                w = send(cfd, p, llen, MSG_NOSIGNAL); if (w < 0) { ok = 0; break; }
            }
            w = send(cfd, "\n", 1, MSG_NOSIGNAL); if (w < 0) { ok = 0; break; }

            if (!nl) break;   /* last line (no trailing newline) */
            p = nl + 1;
        }
        free(json);

        /* Blank line terminates the SSE event */
        if (ok && send(cfd, "\n", 1, MSG_NOSIGNAL) < 0) ok = 0;

        if (!ok) break;  /* client disconnected */
    }

    close(cfd);
    return NULL;
}

/* ---- HTTP listener: GET / → JSON snapshot, GET /events → SSE stream -- */
static void serve_http(int port)
{
    /* Ignore SIGPIPE so writes to closed sockets return -1 instead of
     * killing the process (affects SSE client threads). */
    signal(SIGPIPE, SIG_IGN);

    /* Initialise shared state — monitor thread fills in device paths */
    static ServerState st;
    memset(&st, 0, sizeof(st));
    pthread_mutex_init(&st.lock, NULL);
    pthread_cond_init(&st.new_data, NULL);

    /* Start background threads */
    pthread_t monitor_tid, nmea_tid;
    pthread_create(&monitor_tid, NULL, device_monitor_thread, &st);
    pthread_create(&nmea_tid,    NULL, nmea_stream_thread,    &st);

    /* TCP listener */
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return; }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)port);

    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(srv); return;
    }
    if (listen(srv, 16) < 0) {
        perror("listen"); close(srv); return;
    }

    fprintf(stderr, "Serving on http://0.0.0.0:%d/\n", port);
    fprintf(stderr, "  GET  /                   -> HTML dashboard\n");
    fprintf(stderr, "  GET  /json               -> JSON snapshot\n");
    fprintf(stderr, "  GET  /events             -> SSE stream\n");
    fprintf(stderr, "  POST /config/frequency   -> {\"frequency_hz\":N[,\"save\":true]}\n");
    fprintf(stderr, "  POST /config/output1     -> {\"enabled\":true|false}\n");
    fprintf(stderr, "  POST /config/power1      -> {\"low\":true|false}\n");
    fprintf(stderr, "  POST /config/blink       -> {}\n");

    /* Embedded HTML dashboard */
    static const char html_body[] =
"<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>Leo Bodnar LBE-1420</title>"
"<link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\"/>"
"<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>"
"<style>"
"body{font-family:monospace;background:#111;color:#0f0;margin:0;padding:1em}"
"h1{color:#0f0;font-size:1.2em;margin:0 0 .5em}"
".card{background:#1a1a1a;border:1px solid #0f0;border-radius:4px;padding:.8em;margin:.5em 0}"
".label{color:#888;font-size:.85em}"
".val{color:#0f0;font-size:1em}"
".ok{color:#0f0}.warn{color:#ff0}.err{color:#f00}"
".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:.5em}"
"#status{color:#888;font-size:.8em;margin-bottom:.5em}"
"#map{height:300px;margin:.5em 0;border:1px solid #0f0;border-radius:4px;display:none}"
".ctrl-row{display:flex;align-items:center;gap:.8em;flex-wrap:wrap;margin:.3em 0}"
".ctrl-row label{color:#888;font-size:.85em;min-width:7em}"
"input[type=number]{background:#111;color:#0f0;border:1px solid #0f0;padding:.2em .4em;font-family:monospace;width:10em}"
"input[type=checkbox]{accent-color:#0f0;width:1.1em;height:1.1em;cursor:pointer}"
"button{background:#111;color:#0f0;border:1px solid #0f0;padding:.25em .8em;font-family:monospace;cursor:pointer}"
"button:hover{background:#0f0;color:#111}"
"#ctrl_status{color:#888;font-size:.8em;margin-top:.4em}"
"</style></head><body>"
"<h1>&#x1F4E1; Leo Bodnar LBE-1420</h1>"
"<div id=\"status\">Connecting...</div>"
"<div class=\"card\">"
"  <div class=\"label\">Device</div>"
"  <div class=\"grid\">"
"    <div><div class=\"label\">HID path</div><div class=\"val\" id=\"device\">-</div></div>"
"    <div><div class=\"label\">Serial</div><div class=\"val\" id=\"serial\">-</div></div>"
"    <div><div class=\"label\">GPS lock</div><div class=\"val\" id=\"gps_lock\">-</div></div>"
"    <div><div class=\"label\">PLL lock</div><div class=\"val\" id=\"pll_lock\">-</div></div>"
"    <div><div class=\"label\">Antenna</div><div class=\"val\" id=\"antenna\">-</div></div>"
"    <div><div class=\"label\">Mode</div><div class=\"val\" id=\"mode\">-</div></div>"
"    <div><div class=\"label\">Output 1</div><div class=\"val\" id=\"out1\">-</div></div>"
"    <div><div class=\"label\">Drive</div><div class=\"val\" id=\"drive\">-</div></div>"
"    <div><div class=\"label\">Frequency</div><div class=\"val\" id=\"freq\">-</div></div>"
"  </div>"
"</div>"
"<div class=\"card\">"
"  <div class=\"label\">GPS</div>"
"  <div class=\"grid\">"
"    <div><div class=\"label\">UTC</div><div class=\"val\" id=\"utc\">-</div></div>"
"    <div><div class=\"label\">Fix</div><div class=\"val\" id=\"fix\">-</div></div>"
"    <div><div class=\"label\">Sats used</div><div class=\"val\" id=\"sats\">-</div></div>"
"    <div><div class=\"label\">GPS in view</div><div class=\"val\" id=\"gps_view\">-</div></div>"
"    <div><div class=\"label\">GLO in view</div><div class=\"val\" id=\"glo_view\">-</div></div>"
"    <div><div class=\"label\">HDOP</div><div class=\"val\" id=\"hdop\">-</div></div>"
"    <div><div class=\"label\">VDOP</div><div class=\"val\" id=\"vdop\">-</div></div>"
"    <div><div class=\"label\">PDOP</div><div class=\"val\" id=\"pdop\">-</div></div>"
"    <div><div class=\"label\">Altitude</div><div class=\"val\" id=\"alt\">-</div></div>"
"    <div><div class=\"label\">Speed</div><div class=\"val\" id=\"speed\">-</div></div>"
"    <div><div class=\"label\">Position</div><div class=\"val\" id=\"pos\">-</div></div>"
"  </div>"
"</div>"
"<div id=\"map\"></div>"
"<div class=\"card\" id=\"ctrl_card\">"
"  <div class=\"label\">Controls</div>"
"  <div class=\"ctrl-row\">"
"    <label>Output 1</label>"
"    <input type=\"checkbox\" id=\"ctrl_out1\" onchange=\"setOutput1(this.checked)\">"
"    <span class=\"val\" id=\"ctrl_out1_lbl\">-</span>"
"  </div>"
"  <div class=\"ctrl-row\">"
"    <label>Frequency (Hz)</label>"
"    <input type=\"number\" id=\"ctrl_freq\" min=\"1\" max=\"800000000\" step=\"1\" placeholder=\"Hz\" oninput=\"this.value=this.value.replace(/[^0-9]/g,'')\">"
"    <button onclick=\"setFreq()\">Set</button>"
"    <button onclick=\"setFreqVal(27000000)\">27 MHz</button>"
"    <input type=\"checkbox\" id=\"ctrl_save\" checked>"
"    <span class=\"label\">save to flash</span>"
"  </div>"
"  <div class=\"ctrl-row\">"
"    <label>Output 1 drive</label>"
"    <select id=\"ctrl_pwr1\" onchange=\"setPower1(this.value==='low')\">"
"      <option value=\"normal\">Normal</option>"
"      <option value=\"low\">Low power</option>"
"    </select>"
"  </div>"
"  <div class=\"ctrl-row\">"
"    <label>LED</label>"
"    <button onclick=\"doBlink()\">Blink output 1</button>"
"  </div>"
"  <div id=\"ctrl_status\"></div>"
"</div>"
"<script>"
"var evtCount=0,map=null,marker=null;"
"function fmtFreq(hz){"
"  if(hz>=1e6) return (hz/1e6).toFixed(6)+' MHz';"
"  if(hz>=1e3) return (hz/1e3).toFixed(3)+' kHz';"
"  return hz+' Hz';"
"}"
"function set(id,val,cls){"
"  var e=document.getElementById(id);"
"  if(e){e.textContent=val;if(cls)e.className='val '+cls;else e.className='val';}"
"}"
"function initMap(lat,lng){"
"  var el=document.getElementById('map');"
"  el.style.display='block';"
"  map=L.map('map').setView([lat,lng],15);"
"  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{"
"    attribution:'\\u00a9 <a href=\"https://www.openstreetmap.org/copyright\">OpenStreetMap</a> contributors'"
"  }).addTo(map);"
"}"
"function updateMap(lat,lng,alt){"
"  var latlng=[lat,lng];"
"  var tip=lat.toFixed(6)+'\\u00b0, '+lng.toFixed(6)+'\\u00b0'"
"           +(alt!=null?'\\n'+alt.toFixed(1)+' m':'');"
"  if(!map) initMap(lat,lng);"
"  if(!marker){"
"    marker=L.marker(latlng).addTo(map);"
"    marker.bindTooltip(tip,{permanent:true,direction:'top'}).openTooltip();"
"  } else {"
"    marker.setLatLng(latlng);"
"    marker.setTooltipContent(tip);"
"  }"
"}"
"function update(d){"
"  set('device', d.device||'none');"
"  set('serial', d.serial||'none');"
"  var s=d.device_status;"
"  if(s){"
"    set('gps_lock', s.gps_lock?'LOCKED':'NO LOCK', s.gps_lock?'ok':'err');"
"    set('pll_lock', s.pll_lock?'LOCKED':'unlocked', s.pll_lock?'ok':'warn');"
"    set('antenna',  s.antenna_ok?'OK':'SHORT CIRCUIT', s.antenna_ok?'ok':'err');"
"    set('mode',     s.mode);"
"    var o1=s.output1_enabled?(s.output1_pps?'enabled (1-PPS)':'enabled'):'disabled';"
"    set('out1', o1, s.output1_enabled?'ok':'warn');"
"    set('drive',    s.output1_drive);"
"    set('freq',     fmtFreq(s.frequency_hz));"
"    /* Sync controls from live state (only if user isn't actively editing) */"
"    var cb=document.getElementById('ctrl_out1');"
"    if(cb&&document.activeElement!==cb){"
"      cb.checked=s.output1_enabled;"
"      document.getElementById('ctrl_out1_lbl').textContent=s.output1_enabled?'enabled':'disabled';"
"    }"
"    var fi=document.getElementById('ctrl_freq');"
"    if(fi&&document.activeElement!==fi){"
"      fi.value=s.frequency_hz;"
"    }"
"    var pw=document.getElementById('ctrl_pwr1');"
"    if(pw&&document.activeElement!==pw){"
"      pw.value=(s.output1_drive==='low')?'low':'normal';"
"    }"
"  } else {"
"    ['gps_lock','pll_lock','antenna','mode','out1','drive','freq'].forEach(function(i){set(i,'-');});"
"  }"
"  var g=d.gps;"
"  if(g){"
"    set('utc',      g.datetime_utc||'-');"
"    set('fix',      (g.fix||'-')+' '+(g.fix_mode||''));"
"    set('sats',     g.sats_used);"
"    set('gps_view', g.gps_in_view);"
"    set('glo_view', g.glo_in_view);"
"    set('hdop',     g.hdop!=null?g.hdop.toFixed(2):'-');"
"    set('vdop',     g.vdop!=null?g.vdop.toFixed(2):'-');"
"    set('pdop',     g.pdop!=null?g.pdop.toFixed(2):'-');"
"    set('alt',      g.altitude_m!=null?g.altitude_m.toFixed(1)+' m':'-');"
"    set('speed',    g.speed_knots!=null?g.speed_knots.toFixed(3)+' kn':'-');"
"    if(g.latitude!=null&&g.longitude!=null){"
"      set('pos', g.latitude.toFixed(6)+'\\u00b0, '+g.longitude.toFixed(6)+'\\u00b0');"
"      updateMap(g.latitude,g.longitude,g.altitude_m!=null?g.altitude_m:null);"
"    } else {"
"      set('pos','-');"
"    }"
"  } else {"
"    ['utc','fix','sats','gps_view','glo_view','hdop','vdop','pdop','alt','speed','pos'].forEach(function(i){set(i,'-');});"
"  }"
"}"
"function apiPost(path,body,cb){"
"  fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})"
"  .then(function(r){return r.json();})"
"  .then(function(j){"
"    var el=document.getElementById('ctrl_status');"
"    if(j.ok) el.textContent='OK';"
"    else el.textContent='Error: '+j.error;"
"    if(cb) cb(j);"
"  })"
"  .catch(function(e){document.getElementById('ctrl_status').textContent='Request failed: '+e;});"
"}"
"function setOutput1(enabled){"
"  document.getElementById('ctrl_out1_lbl').textContent=enabled?'enabled':'disabled';"
"  apiPost('/config/output1',{enabled:enabled});"
"}"
"function setFreqVal(hz){"
"  document.getElementById('ctrl_freq').value=hz;"
"  var save=document.getElementById('ctrl_save').checked;"
"  apiPost('/config/frequency',{frequency_hz:hz,save:save});"
"}"
"function setFreq(){"
"  var hz=parseInt(document.getElementById('ctrl_freq').value,10);"
"  if(!hz||hz<1){document.getElementById('ctrl_status').textContent='Enter a valid frequency in Hz';return;}"
"  var save=document.getElementById('ctrl_save').checked;"
"  apiPost('/config/frequency',{frequency_hz:hz,save:save});"
"}"
"function setPower1(low){"
"  apiPost('/config/power1',{low:low});"
"}"
"function doBlink(){"
"  apiPost('/config/blink',{});"
"}"
"var es=new EventSource('/events');"
"es.onopen=function(){document.getElementById('status').textContent='Connected - live updates via SSE';};"
"es.onerror=function(){document.getElementById('status').textContent='SSE disconnected - retrying...';};"
"es.onmessage=function(e){"
"  evtCount++;"
"  document.getElementById('status').textContent='Connected - events received: '+evtCount;"
"  try{update(JSON.parse(e.data));}catch(ex){"
"    document.getElementById('status').textContent='JSON parse error: '+ex.message;"
"  }"
"};"
"</script></body></html>\n";

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int cfd = accept(srv, (struct sockaddr *)&client_addr, &client_len);
        if (cfd < 0) continue;

        /* Read HTTP request: headers + body.
         * Use a generous buffer; for POST endpoints the body is small
         * but browser headers can be 600-800 bytes on their own.
         * If the body didn't fit in the first read (Content-Length says
         * there's more), do a second read to get the rest. */
        char req[4096] = {0};
        ssize_t rlen = read(cfd, req, sizeof(req) - 1);
        if (rlen > 0) {
            req[rlen] = '\0';
            /* Check if we have the full body for POST requests */
            const char *hdr_end = strstr(req, "\r\n\r\n");
            if (hdr_end) {
                /* Find Content-Length header */
                const char *cl = strcasestr(req, "Content-Length:");
                if (cl) {
                    long clen = strtol(cl + 15, NULL, 10);
                    long body_have = (long)rlen - (long)(hdr_end + 4 - req);
                    if (clen > body_have && clen < 1024) {
                        /* Body was split — read the rest */
                        ssize_t r2 = read(cfd, req + rlen,
                                          (size_t)(clen - body_have));
                        if (r2 > 0) {
                            rlen += r2;
                            req[rlen] = '\0';
                        }
                    }
                }
            }
        }

        /* Determine method and path */
        int is_get  = (strncmp(req, "GET ",  4) == 0);
        int is_post = (strncmp(req, "POST ", 5) == 0);
        int is_sse  = is_get  && (strncmp(req + 4, "/events", 7) == 0);
        int is_json = is_get  && (strncmp(req + 4, "/json",   5) == 0);
        int is_post_freq  = is_post && (strncmp(req + 5, "/config/frequency", 17) == 0);
        int is_post_out1  = is_post && (strncmp(req + 5, "/config/output1",   15) == 0);
        int is_post_pwr1  = is_post && (strncmp(req + 5, "/config/power1",    14) == 0);
        int is_post_blink = is_post && (strncmp(req + 5, "/config/blink",     13) == 0);

        if (is_sse) {
            /* Hand off to a dedicated SSE thread (keeps connection open) */
            SseArg *a = malloc(sizeof(SseArg));
            if (!a) { close(cfd); continue; }
            a->st = &st;
            a->fd = cfd;
            pthread_t tid;
            pthread_attr_t attr;
            pthread_attr_init(&attr);
            pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
            if (pthread_create(&tid, &attr, sse_client_thread, a) != 0) {
                free(a); close(cfd);
            }
            pthread_attr_destroy(&attr);
        } else if (is_json) {
            /* One-shot JSON response */
            int device_present = 0;
            char *json = state_to_json(&st, &device_present);

            static const char oom_json[] = "{\"error\":\"out of memory\"}\n";
            const char *body     = json ? json : oom_json;
            size_t      body_len = strlen(body);
            const char *status_line =
                !json          ? "HTTP/1.1 500 Internal Server Error" :
                device_present ? "HTTP/1.1 200 OK"                    :
                                 "HTTP/1.1 503 Service Unavailable";

            char header[256];
            int hlen = snprintf(header, sizeof(header),
                "%s\r\n"
                "Content-Type: application/json\r\n"
                "Content-Length: %zu\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "Connection: close\r\n"
                "\r\n",
                status_line, body_len);
            write(cfd, header, hlen);
            write(cfd, body, body_len);
            if (json) free(json);
            close(cfd);
        } else if (is_post_freq || is_post_out1 || is_post_pwr1 || is_post_blink) {
            /* --- Config POST endpoints ---------------------------------- */

            /* Extract body: find \r\n\r\n separator */
            const char *body_start = strstr(req, "\r\n\r\n");
            if (body_start) body_start += 4;
            else body_start = "";

            /* Snapshot HID path under lock */
            char hid_snap[64] = {0};
            pthread_mutex_lock(&st.lock);
            strncpy(hid_snap, st.hidraw_path, sizeof(hid_snap) - 1);
            pthread_mutex_unlock(&st.lock);

            uint8_t hbuf[60];
            memset(hbuf, 0, sizeof(hbuf));
            char errbuf[128] = {0};
            int rc;

            if (is_post_freq) {
                long freq_hz = 0;
                if (!json_get_long(body_start, "frequency_hz", &freq_hz) || freq_hz <= 0) {
                    send_api_response(cfd, 0, 0, "missing or invalid frequency_hz");
                    close(cfd); continue;
                }
                int save_flag = 1;  /* default: save to flash */
                int save_val  = 1;
                if (json_get_bool(body_start, "save", &save_val))
                    save_flag = save_val;

                hbuf[0] = save_flag ? LBE_1420_SET_F1 : LBE_1420_SET_F1_NO_SAVE;
                hbuf[1] = (freq_hz >>  0) & 0xff;
                hbuf[2] = (freq_hz >>  8) & 0xff;
                hbuf[3] = (freq_hz >> 16) & 0xff;
                hbuf[4] = (freq_hz >> 24) & 0xff;
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));

            } else if (is_post_out1) {
                int enabled = -1;
                if (!json_get_bool(body_start, "enabled", &enabled)) {
                    send_api_response(cfd, 0, 0, "missing or invalid enabled");
                    close(cfd); continue;
                }
                hbuf[0] = LBE_1420_EN_OUT1;
                hbuf[1] = enabled & 0x01;
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));

            } else if (is_post_pwr1) {
                int low = -1;
                if (!json_get_bool(body_start, "low", &low)) {
                    send_api_response(cfd, 0, 0, "missing or invalid low");
                    close(cfd); continue;
                }
                hbuf[0] = LBE_1420_SET_PWR1;
                hbuf[1] = low ? 0x01 : 0x00;
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));

            } else {  /* is_post_blink */
                hbuf[0] = LBE_1420_BLINK_OUT1;
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));
            }

            if (rc == 0)
                send_api_response(cfd, 1, 1, NULL);
            else if (rc == -1)
                send_api_response(cfd, -1, 0, errbuf);
            else
                send_api_response(cfd, -2, 0, errbuf);
            close(cfd);

        } else {
            /* HTML dashboard */
            size_t html_len = sizeof(html_body) - 1;
            char header[128];
            int hlen = snprintf(header, sizeof(header),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html; charset=utf-8\r\n"
                "Content-Length: %zu\r\n"
                "Connection: close\r\n"
                "\r\n",
                html_len);
            write(cfd, header, hlen);
            write(cfd, html_body, html_len);
            close(cfd);
        }
    }
    close(srv);
}

int processCommandLineArguments(int argc, char **argv, int *freq, int *blink, int *enable, int *save, char **serial_port, int *json_out, int *serve, int *port);

int main(int argc, char **argv)
{
      /* Quick pre-scan for --json / -j and --serve before any output */
      int json_mode = 0;
      int serve_mode = 0;
      int serve_port = HTTP_DEFAULT_PORT;
      for (int i = 1; i < argc; i++) {
          if (strcmp(argv[i], "--json") == 0 || strcmp(argv[i], "-j") == 0) {
              json_mode = 1;
          } else if (strcmp(argv[i], "--serve") == 0) {
              serve_mode = 1;
              json_mode  = 1;   /* serve mode implies JSON */
          } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
              serve_port = atoi(argv[++i]);
          }
      }

      if (!json_mode)
          printf("Leo Bodnar LBE-142x GPS locked clock source config\n\n");

      int fd;
      int res;
      u_int8_t buf[60];
      uint32_t current_f = 0;

      struct hidraw_devinfo info;

      /* Determine hidraw device path — auto-discover if not supplied */
      /* Use persistent buffers so pointers remain valid throughout main() */
      static char hidraw_path_buf[64];
      static char auto_serial_buf[64];
      const char *hidraw_path = NULL;
      const char *auto_serial = NULL;

      /* Check if first non-option arg looks like a device path */
      if (argc >= 2 && argv[1][0] == '/') {
          hidraw_path = argv[1];
      } else {
          /* Auto-discover */
          LbeDevice devs[MAX_DEVICES];
          int ndevs = find_lbe_devices(devs, MAX_DEVICES);

          if (ndevs == 0) {
              if (serve_mode) {
                  /* In serve mode, start the server anyway — the monitor
                   * thread will find the device when it appears. */
              } else if (json_mode) {
                  printf("{\"error\": \"No Leo Bodnar LBE-142x device found\"}\n");
                  return 1;
              } else {
                  printf("No Leo Bodnar LBE-142x device found.\n\n");
                  printf("Usage: lbe-142x [/dev/hidraw??] [--serial /dev/ttyACM??] [options]\n\n");
                  printf("        --f1:  frequency in Hz (1 to 1100000000), saved to flash\n");
                  printf(" --f1_nosave:  frequency in Hz, not saved\n");
                  printf("      --out1:  [0,1]  enable/disable output\n");
                  printf("     --blink1  blink output 1 LED for 3 seconds\n");
                  printf("    --serial:  serial port for NMEA GPS data\n\n");
                  return 1;
              }
          } else if (ndevs == 1) {
              if (!json_mode) {
                  printf("Auto-detected: %s  (%s)\n", devs[0].hidraw_path, devs[0].name);
                  if (devs[0].serial_path[0])
                      printf("  Serial port: %s\n", devs[0].serial_path);
                  printf("\n");
              }
              strncpy(hidraw_path_buf, devs[0].hidraw_path, sizeof(hidraw_path_buf) - 1);
              hidraw_path = hidraw_path_buf;
              if (devs[0].serial_path[0]) {
                  strncpy(auto_serial_buf, devs[0].serial_path, sizeof(auto_serial_buf) - 1);
                  auto_serial = auto_serial_buf;
              }
          } else {
              if (json_mode) {
                  /* In JSON mode, just pick the first device */
                  strncpy(hidraw_path_buf, devs[0].hidraw_path, sizeof(hidraw_path_buf) - 1);
                  hidraw_path = hidraw_path_buf;
                  if (devs[0].serial_path[0]) {
                      strncpy(auto_serial_buf, devs[0].serial_path, sizeof(auto_serial_buf) - 1);
                      auto_serial = auto_serial_buf;
                  }
              } else {
                  printf("Multiple Leo Bodnar devices found:\n");
                  for (int i = 0; i < ndevs; i++)
                      printf("  [%d] %s  %s  serial: %s\n", i + 1,
                             devs[i].hidraw_path, devs[i].name,
                             devs[i].serial_path[0] ? devs[i].serial_path : "(none)");
                  printf("\nSelect device [1-%d]: ", ndevs);
                  fflush(stdout);
                  int choice = 0;
                  if (scanf("%d", &choice) != 1 || choice < 1 || choice > ndevs) {
                      printf("Invalid selection.\n");
                      return 1;
                  }
                  choice--;
                  strncpy(hidraw_path_buf, devs[choice].hidraw_path, sizeof(hidraw_path_buf) - 1);
                  hidraw_path = hidraw_path_buf;
                  if (devs[choice].serial_path[0]) {
                      strncpy(auto_serial_buf, devs[choice].serial_path, sizeof(auto_serial_buf) - 1);
                      auto_serial = auto_serial_buf;
                  }
                  printf("\n");
              }
          }
      }

      /* In serve mode with no device present, skip straight to the server.
       * The monitor thread inside serve_http() will open the device later. */
      if (serve_mode && !hidraw_path) {
          serve_http(serve_port);
          return 0;
      }

      if (!json_mode)
          printf("Opening device %s\n", hidraw_path);

      fd = open(hidraw_path, O_RDWR | O_NONBLOCK);

      if (fd < 0)
      {
            perror("    Unable to open device");
            return 1;
      }

      //Device connected, setup report structs
      memset(&info, 0x0, sizeof(info));

      // Get Raw Info
      res = ioctl(fd, HIDIOCGRAWINFO, &info);
      
      if (res < 0)
      {
            if (!json_mode) perror("HIDIOCGRAWINFO");
      }
      else
      {
            if (info.vendor != VID_LBE || (info.product != PID_LBE_1420 && info.product != PID_LBE_1421)) {
                if (json_mode) {
                    printf("{\"error\": \"Not a valid LBE-142x device\"}\n");
                } else {
                    printf("    Not a valid LBE-142x Device\n\n");
                    printf("        vendor: 0x%04hx\n", info.vendor);
                    printf("        product: 0x%04hx\n", info.product);
                }
                return -1;
            }
      }

      /* Get Raw Name */
      res = ioctl(fd, HIDIOCGRAWNAME(256), buf);

      if (res < 0) {
            if (!json_mode) perror("HIDIOCGRAWNAME");
      }
      else {
            if (!json_mode) printf("Connected To: %s\n\n", buf);
      }

      /* Get Feature — report 0x01 contains status bits + frequency */
      buf[0] = 0x1; /* Report Number */
      res = ioctl(fd, HIDIOCGFEATURE(256), buf);

      if (res < 0) {
            if (!json_mode) perror("HIDIOCGFEATURE");
      }

      /* Collect HID status fields */
      uint8_t hid_status  = 0;
      uint8_t hid_fll     = 0;
      uint8_t hid_out1low = 0;
      int     hid_valid   = 0;
      if (res >= 0 && buf[0] == 0x01) {
          hid_status  = buf[1];
          hid_fll     = buf[18];
          hid_out1low = buf[10]; /* out1_power_low at byte 10 */
          /* frequency is at bytes 6-9 little-endian */
          current_f   = ((uint32_t)buf[9] << 24) | ((uint32_t)buf[8] << 16) |
                        ((uint32_t)buf[7] <<  8) |  (uint32_t)buf[6];
          hid_valid   = 1;
      }

      /* Get Raw Name (reuse buf for CLI parsing below) */
      res = ioctl(fd, HIDIOCGRAWNAME(256), buf);

      if (res < 0) {
            if (!json_mode) perror("HIDIOCGRAWNAME");
      }
      {

 //Get CLI values as vars
 int blink = -1;
 int enable = -1;
 int save = -1;
 int new_f = 0xffffffff;
 int json_out_unused = 0; /* already captured in json_mode */
 int serve_unused = 0, port_unused = 0; /* already captured in pre-scan */
 char *serial_port = NULL;
 processCommandLineArguments(argc, argv, &new_f, &blink, &enable, &save, &serial_port, &json_out_unused, &serve_unused, &port_unused);

 /* Fall back to auto-detected serial port if --serial not given */
 if (!serial_port && auto_serial)
     serial_port = (char *)auto_serial;

 /* Collect NMEA data if serial port available */
 NmeaInfo nmea;
 int nmea_valid = 0;
 if (serial_port) {
     if (read_nmea(serial_port, &nmea) == 0)
         nmea_valid = 1;
 }

 if (json_mode) {
     /* ---- JSON output ---- */
     printf("{\n");
     printf("  \"device\": \"%s\",\n", hidraw_path);
     if (serial_port)
         printf("  \"serial\": \"%s\",\n", serial_port);
     if (hid_valid) {
         printf("  \"device_status\": {\n");
         printf("    \"gps_lock\":   %s,\n",  (hid_status & GPS_LOCK_BIT) ? "true" : "false");
         printf("    \"pll_lock\":   %s,\n",  (hid_status & PLL_LOCK_BIT) ? "true" : "false");
         printf("    \"antenna_ok\": %s,\n",  (hid_status & ANT_OK_BIT)   ? "true" : "false");
         printf("    \"mode\":       \"%s\",\n", hid_fll ? "FLL" : "PLL");
         printf("    \"output1_enabled\": %s,\n", (hid_status & OUT1_EN_BIT) ? "true" : "false");
         printf("    \"output1_pps\":     %s,\n", (hid_status & PPS1_BIT)    ? "true" : "false");
         printf("    \"output1_drive\":   \"%s\",\n", hid_out1low ? "low" : "normal");
         printf("    \"frequency_hz\":    %u\n", current_f);
         printf("  }");
     } else {
         printf("  \"device_status\": null");
     }
     if (nmea_valid) {
         const char *fix_str;
         switch (nmea.fix_quality) {
             case 0:  fix_str = "none";  break;
             case 1:  fix_str = "GPS";   break;
             case 2:  fix_str = "DGPS";  break;
             default: fix_str = "unknown"; break;
         }
         const char *fix_mode_str_j;
          switch (nmea.fix_mode) {
              case 2:  fix_mode_str_j = "2D"; break;
              case 3:  fix_mode_str_j = "3D"; break;
              default: fix_mode_str_j = "none"; break;
          }
          printf(",\n  \"gps\": {\n");
          if (nmea.iso_datetime[0]) printf("    \"datetime_utc\": \"%s\",\n", nmea.iso_datetime);
          printf("    \"fix\":          \"%s\",\n",  fix_str);
          printf("    \"fix_mode\":     \"%s\",\n",  fix_mode_str_j);
          printf("    \"sats_used\":    %d,\n",      nmea.sats_used);
          printf("    \"gps_in_view\":  %d,\n",      nmea.gps_sats_view);
          printf("    \"glo_in_view\":  %d,\n",      nmea.glo_sats_view);
          printf("    \"hdop\":         %.2f,\n",    nmea.hdop);
          printf("    \"vdop\":         %.2f,\n",    nmea.vdop);
          printf("    \"pdop\":         %.2f,\n",    nmea.pdop);
          printf("    \"altitude_m\":   %.1f",       nmea.altitude_m);
          if (nmea.has_speed)
              printf(",\n    \"speed_knots\":  %.3f", nmea.speed_knots);
          if (nmea.has_position) {
              printf(",\n    \"latitude\":     %.7f", nmea.lat_deg);
              printf(",\n    \"longitude\":    %.7f", nmea.lon_deg);
          }
          printf("\n  }");
     } else if (serial_port) {
         printf(",\n  \"gps\": null");
     }
     printf("\n}\n");
 } else {
     /* ---- Human-readable output ---- */
     if (hid_valid) {
         printf("  Device Status:\n");
         printf("    GPS lock:   %s\n", (hid_status & GPS_LOCK_BIT) ? "LOCKED"        : "NO LOCK");
         printf("    PLL lock:   %s\n", (hid_status & PLL_LOCK_BIT) ? "LOCKED"        : "unlocked");
         printf("    Antenna:    %s\n", (hid_status & ANT_OK_BIT)   ? "OK"            : "SHORT CIRCUIT");
         printf("    Mode:       %s\n", hid_fll                     ? "FLL"           : "PLL");
         printf("    Output 1:   %s",  (hid_status & OUT1_EN_BIT)   ? "enabled"       : "disabled");
         if (hid_status & PPS1_BIT) printf("  (1-PPS mode)");
         printf("\n");
         printf("    Out1 drive: %s\n", hid_out1low ? "low" : "normal");
         printf("    Frequency:  %u Hz\n", current_f);
         printf("\n");
     }
     if (serial_port) {
         printf("  GPS Info (from %s):\n", serial_port);
         if (nmea_valid) {
             const char *fix_str;
             switch (nmea.fix_quality) {
                 case 0:  fix_str = "No fix";   break;
                 case 1:  fix_str = "GPS fix";  break;
                 case 2:  fix_str = "DGPS fix"; break;
                 default: fix_str = "Unknown";  break;
             }
             if (nmea.utc_date[0]) printf("    Date:        %s\n",   nmea.utc_date);
             if (nmea.utc_time[0]) printf("    Time:        %s\n",   nmea.utc_time);
             const char *fix_mode_str_h;
             switch (nmea.fix_mode) {
                 case 2:  fix_mode_str_h = "2D"; break;
                 case 3:  fix_mode_str_h = "3D"; break;
                 default: fix_mode_str_h = "none"; break;
             }
             printf("    Fix:         %s (%s)\n", fix_str, fix_mode_str_h);
             printf("    Sats used:   %d\n",   nmea.sats_used);
             printf("    GPS in view: %d\n",   nmea.gps_sats_view);
             printf("    GLO in view: %d\n",   nmea.glo_sats_view);
             printf("    HDOP:        %.2f\n", nmea.hdop);
             printf("    VDOP:        %.2f\n", nmea.vdop);
             printf("    PDOP:        %.2f\n", nmea.pdop);
             printf("    Altitude:    %.1f m\n", nmea.altitude_m);
             if (nmea.has_speed)
                 printf("    Speed:       %.3f kn\n", nmea.speed_knots);
             if (nmea.has_position) {
                 double lat = nmea.lat_deg;
                 double lon = nmea.lon_deg;
                 char lat_hemi = (lat >= 0) ? 'N' : 'S';
                 char lon_hemi = (lon >= 0) ? 'E' : 'W';
                 if (lat < 0) lat = -lat;
                 if (lon < 0) lon = -lon;
                 int lat_d = (int)lat;
                 int lon_d = (int)lon;
                 double lat_m = (lat - lat_d) * 60.0;
                 double lon_m = (lon - lon_d) * 60.0;
                 printf("    Position:    %d°%07.4f'%c  %d°%07.4f'%c\n",
                        lat_d, lat_m, lat_hemi, lon_d, lon_m, lon_hemi);
             }
         } else {
             printf("    (no NMEA data received within %d seconds)\n", NMEA_TIMEOUT_SEC);
         }
         printf("\n");
     }
 }

 if (!json_mode) {
     printf("  Changes:\n");
 }
      	int changed = 0;
 if (new_f != 0xffffffff && new_f != current_f) {
     if (!json_mode) printf("    Setting Frequency: %d Hz\n", new_f);

     memset(buf, 0, sizeof(buf));
     buf[0] = (save == 1 ? LBE_1420_SET_F1 : LBE_1420_SET_F1_NO_SAVE);
     buf[1] = (new_f >>  0) & 0xff;
     buf[2] = (new_f >>  8) & 0xff;
     buf[3] = (new_f >> 16) & 0xff;
     buf[4] = (new_f >> 24) & 0xff;
     res = ioctl(fd, HIDIOCSFEATURE(60), buf);
     if (res < 0) perror("HIDIOCSFEATURE");
     changed = 1;
 }
 if (enable != -1) {
     memset(buf, 0, sizeof(buf));
     buf[0] = LBE_1420_EN_OUT1;
     buf[1] = enable & 0x01;
     if (!json_mode) printf("    Output 1 enable: %d\n", enable);
     res = ioctl(fd, HIDIOCSFEATURE(60), buf);
     if (res < 0) perror("HIDIOCSFEATURE");
     changed = 1;
 }
 if (blink != -1) {
     memset(buf, 0, sizeof(buf));
     buf[0] = LBE_1420_BLINK_OUT1;
     if (!json_mode) printf("    Blink LED\n");
     res = ioctl(fd, HIDIOCSFEATURE(60), buf);
     if (res < 0) perror("HIDIOCSFEATURE");
     changed = 1;
 }
 if (!changed && !json_mode) {
     printf("    No changes made\n");
 }

 /* Start web server if requested (blocks forever) */
 if (serve_mode) {
     serve_http(serve_port);
 }
      }
      close(fd);

      return 0;
}


int processCommandLineArguments(int argc, char **argv, int *freq, int *blink, int *enable, int *save, char **serial_port, int *json_out, int *serve, int *port)
{
    int c;
    
    while (1)
    {
        static struct option long_options[] =
        {
                /* These options set a flag. */
                {"blink1",    no_argument,       0, 0},
                /* These options don't set a flag.
                    We distinguish them by their indices. */
                {"f1",        required_argument, 0, 'a'},
                {"f1_nosave", required_argument, 0, 'b'},
                {"out1",      required_argument, 0, 'c'},
                {"serial",    required_argument, 0, 's'},
                {"json",      no_argument,       0, 'j'},
                {"serve",     no_argument,       0, 'S'},
                {"port",      required_argument, 0, 'P'},
                {0, 0, 0, 0}
        };
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long(argc, argv, "abc:s:jSP:",
                    long_options, &option_index);

        /* Detect the end of the options. */
        if (c == -1)
        break;

        switch (c)
        {
            case 0:
            	*blink = 1;
                /* If this option set a flag, do nothing else now. */
                if (long_options[option_index].flag != 0)
                        break;
                break;
            
            case 'a'://f1
                *freq = atoi(optarg);
                *save = 1;
                break;

            case 'b'://f1_nosave
                *freq = atoi(optarg);
                *save = 0;
                break;

            case 'c'://out1
                *enable = atoi(optarg);
                break;

            case 's'://serial port
                *serial_port = optarg;
                break;

            case 'j'://json output
                *json_out = 1;
                break;

            case 'S'://serve
                *serve = 1;
                break;

            case 'P'://port
                *port = atoi(optarg);
                break;

            case '?':
                /* getopt_long already printed an error message. */
                break;

            default:
                abort();
        }
    }
    return 0;
}
