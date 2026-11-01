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

//Leo LBE-142x defines, don't need anything external this time
#define VID_LBE		0x1dd2

#define PID_LBE_1420	0x2443
#define PID_LBE_1421	0x2444


#define GPS_LOCK_BIT	0x01
#define PLL_LOCK_BIT	0x02
#define ANT_OK_BIT		0x04
#define LED1_BIT		0x08
#define LED2_BIT		0x10  // This is new for LBE-1421.
#define OUT1_EN_BIT		0x20  // Changed from 0x10 to 0x20.
#define OUT2_EN_BIT		0x40  // Changed from 0x20 to 0x40.
#define PPS1_BIT		0x80  // Changed from 0x40 to 0x80.  Only supported on 1421?

/* Common commands */
#define LBE_142X_EN        0x01
#define LBE_142X_BLINK	   0x02

// LBE-1420 Specific Commands
#define LBE_1420_SET_F1_NO_SAVE	0x03
#define LBE_1420_SET_F1		    0x04
#define LBE_1420_SET_PWR1	    0x07

// LBE-1421 Specific Commands (These are all new)
#define LBE_1421_SET_F1_NO_SAVE 0x05
#define LBE_1421_SET_F1         0x06
#define LBE_1421_SET_F2_NO_SAVE 0x09
#define LBE_1421_SET_F2         0x0A
#define LBE_142X_SET_PLL        0x0B
#define LBE_1421_SET_PPS        0x0C
#define LBE_1421_SET_PWR1       0x0D
#define LBE_1421_SET_PWR2       0x0E

#define LBE_1420_MAX_FREQ       1600000000UL
#define LBE_1421_MAX_FREQ       1400000000UL
#define LBE_REPORT_SIZE          60

static uint16_t selected_pid = PID_LBE_1420; /* Default fallback */

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 */
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* The frequency payload is the only command whose layout differs between
 * the two models: 1420 uses bytes 1..4, while 1421 uses bytes 5..8.  Keep
 * that detail here so every caller (CLI and HTTP) emits the same report. */
static int build_frequency_command(uint16_t pid, int output, int save,
                                   uint32_t hz, uint8_t buf[LBE_REPORT_SIZE])
{
    uint8_t opcode;
    size_t offset;
    uint32_t max_freq;

    memset(buf, 0, LBE_REPORT_SIZE);
    if (pid == PID_LBE_1421) {
        if (output == 1)
            opcode = save ? LBE_1421_SET_F1 : LBE_1421_SET_F1_NO_SAVE;
        else if (output == 2)
            opcode = save ? LBE_1421_SET_F2 : LBE_1421_SET_F2_NO_SAVE;
        else
            return -1;
        offset = 5;
        max_freq = LBE_1421_MAX_FREQ;
    } else if (pid == PID_LBE_1420) {
        if (output != 1)
            return -1;
        opcode = save ? LBE_1420_SET_F1 : LBE_1420_SET_F1_NO_SAVE;
        offset = 1;
        max_freq = LBE_1420_MAX_FREQ;
    } else {
        return -1;
    }

    if (hz == 0 || hz > max_freq)
        return -1;

    buf[0] = opcode;
    buf[offset + 0] = (uint8_t)(hz >> 0);
    buf[offset + 1] = (uint8_t)(hz >> 8);
    buf[offset + 2] = (uint8_t)(hz >> 16);
    buf[offset + 3] = (uint8_t)(hz >> 24);
    return 0;
}

static int build_power_command(uint16_t pid, int output, int low,
                               uint8_t buf[LBE_REPORT_SIZE])
{
    memset(buf, 0, LBE_REPORT_SIZE);
    if (pid == PID_LBE_1421) {
        if (output == 1)
            buf[0] = LBE_1421_SET_PWR1;
        else if (output == 2)
            buf[0] = LBE_1421_SET_PWR2;
        else
            return -1;
    } else if (pid == PID_LBE_1420 && output == 1) {
        buf[0] = LBE_1420_SET_PWR1;
    } else {
        return -1;
    }
    buf[1] = low ? 0x01 : 0x00;
    return 0;
}

static void build_outputs_enable_command(uint16_t pid, int enable,
                                         uint8_t buf[LBE_REPORT_SIZE])
{
    memset(buf, 0, LBE_REPORT_SIZE);
    buf[0] = LBE_142X_EN;
    /* The 1421 command controls its two outputs together. */
    buf[1] = enable ? (pid == PID_LBE_1421 ? 0x03 : 0x01) : 0x00;
}

static void build_pll_command(int fll_mode, uint8_t buf[LBE_REPORT_SIZE])
{
    memset(buf, 0, LBE_REPORT_SIZE);
    buf[0] = LBE_142X_SET_PLL;
    buf[1] = fll_mode ? 0x01 : 0x00;
}

static int build_pps_command(uint16_t pid, int enable,
                             uint8_t buf[LBE_REPORT_SIZE])
{
    if (pid != PID_LBE_1421)
        return -1;
    memset(buf, 0, LBE_REPORT_SIZE);
    buf[0] = LBE_1421_SET_PPS;
    buf[1] = enable ? 0x01 : 0x00;
    return 0;
}

typedef struct {
    uint8_t status;
    uint8_t fll;
    uint8_t out1_low;
    uint8_t out2_low;
    uint32_t frequency1_hz;
    uint32_t frequency2_hz;
    int valid;
} HidStatus;

static void decode_hid_status(uint16_t pid, const uint8_t buf[LBE_REPORT_SIZE],
                              HidStatus *status)
{
    memset(status, 0, sizeof(*status));
    status->status = buf[1];
    status->frequency1_hz = ((uint32_t)buf[9] << 24) |
                            ((uint32_t)buf[8] << 16) |
                            ((uint32_t)buf[7] << 8) |
                            (uint32_t)buf[6];
    status->fll = buf[18];
    if (pid == PID_LBE_1421) {
        status->frequency2_hz = ((uint32_t)buf[17] << 24) |
                                ((uint32_t)buf[16] << 16) |
                                ((uint32_t)buf[15] << 8) |
                                (uint32_t)buf[14];
        status->out1_low = buf[19];
        status->out2_low = buf[20];
    } else {
        status->out1_low = buf[10];
    }
    status->valid = 1;
}

static int read_hid_status(const char *hidraw_path, uint16_t pid,
                           HidStatus *status)
{
    uint8_t buf[LBE_REPORT_SIZE] = {0};
    if (!hidraw_path || !hidraw_path[0])
        return -1;

    int fd = open(hidraw_path, O_RDWR | O_NONBLOCK);
    if (fd < 0)
        return -1;
    buf[0] = 0x01;
    int rc = ioctl(fd, HIDIOCGFEATURE(256), buf);
    close(fd);
    if (rc < 0 || buf[0] != 0x01)
        return -1;
    decode_hid_status(pid, buf, status);
    return 0;
}

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
    cfmakeraw(&tty);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CREAD | CLOCAL);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag  = IGNBRK;
    tty.c_lflag  = 0;
    tty.c_oflag  = 0;
    tty.c_cflag &= ~HUPCL;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    tcsetattr(sfd, TCSANOW, &tty);
    tcflush(sfd, TCIFLUSH);

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
    uint16_t product;
} LbeDevice;

/*
 * Given a hidraw node name (e.g. "hidraw0"), try to find the paired
 * ttyACM device by walking sysfs:
 *   /sys/class/hidraw/hidrawN/device -> USB interface
 *   go up two levels to USB device, then look for tty in all interfaces
 */
static void find_paired_serial(const char *hidraw_name, char *out, size_t outsz)
{
    if (outsz == 0)
        return;
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

    /* Resolve to an absolute path.  Passing a fixed buffer smaller than
     * PATH_MAX makes glibc's fortified realpath abort at runtime. */
    char *usbdev = realpath(usbdev_rel, NULL);
    if (!usbdev) return;

    /* Scan the USB device directory for interface dirs containing tty/ */
    DIR *d = opendir(usbdev);
    if (!d) {
        free(usbdev);
        return;
    }

    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') continue;

        size_t ttydir_len = strlen(usbdev) + 1 + strlen(ent->d_name) +
                            sizeof("/tty");
        char *ttydir = malloc(ttydir_len);
        if (!ttydir) continue;
        snprintf(ttydir, ttydir_len, "%s/%s/tty", usbdev, ent->d_name);

        DIR *td = opendir(ttydir);
        if (!td) {
            free(ttydir);
            continue;
        }

        struct dirent *tent;
        while ((tent = readdir(td)) != NULL) {
            if (tent->d_name[0] == '.') continue;
            /* Found a tty entry, e.g. "ttyACM0" */
            const size_t prefix_len = sizeof("/dev/") - 1;
            if (outsz <= prefix_len) continue;
            size_t name_len = strnlen(tent->d_name, outsz - prefix_len);
            if (name_len == outsz - prefix_len) continue;
            memcpy(out, "/dev/", prefix_len);
            memcpy(out + prefix_len, tent->d_name, name_len + 1);
            closedir(td);
            free(ttydir);
            closedir(d);
            free(usbdev);
            return;
        }
        closedir(td);
        free(ttydir);
    }
    closedir(d);
    free(usbdev);
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

            memset(&devs[count], 0, sizeof(devs[count]));
            devs[count].product = info.product;

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

/* Global read-only flag — set once at startup, never modified afterwards.
 * When non-zero, all HID config writes and HTTP configuration endpoints
 * are blocked. */
static int g_readonly = 0;

typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t  new_data;  /* broadcast by NMEA thread on each $GNGGA */

    /* Device paths — updated by monitor thread under lock */
    char hidraw_path[64];
    char serial_port[64];
    int  serial_changed;   /* set by monitor, cleared by NMEA thread */

    /* HID state — updated by the NMEA and heartbeat threads. */
    uint16_t  pid;
    HidStatus hid;

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
            if (strcmp(st->hidraw_path, devs[0].hidraw_path) != 0) {
                strncpy(st->hidraw_path, devs[0].hidraw_path,
                        sizeof(st->hidraw_path) - 1);
                st->hid.valid = 0;
            }
            st->pid = devs[0].product;
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
            st->pid = 0;
            st->hid.valid = 0;
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
    /* Always re-read the current port from shared state before opening.
     * The monitor thread may have updated it (e.g. ttyACM0 → ttyACM1)
     * while we were looping on a dead fd. */
    {
        pthread_mutex_lock(&st->lock);
        strncpy(cur_port, st->serial_port, sizeof(cur_port) - 1);
        st->serial_changed = 0;
        pthread_mutex_unlock(&st->lock);
    }
    if (!cur_port[0]) goto wait_for_port;

    int sfd = open(cur_port, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (sfd < 0) {
        sleep(2);
        goto retry;
    }

    /* Configure serial port */
    int flags = fcntl(sfd, F_GETFL, 0);
    if (flags >= 0) fcntl(sfd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty;
    if (tcgetattr(sfd, &tty) != 0) {
        perror("    tcgetattr");
        close(sfd);
        return NULL;
    }
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    cfmakeraw(&tty);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CREAD | CLOCAL);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag  = IGNBRK;
    tty.c_lflag  = 0;
    tty.c_oflag  = 0;
    tty.c_cflag &= ~HUPCL;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    tcsetattr(sfd, TCSANOW, &tty);
    tcflush(sfd, TCIFLUSH);

    if (flags >= 0) fcntl(sfd, F_SETFL, flags | O_NONBLOCK);

    char linebuf[NMEA_LINE_MAX];
    int  linelen = 0;

    while (1) {
        /* Check if monitor has signalled a port change before blocking */
        pthread_mutex_lock(&st->lock);
        int chg = st->serial_changed;
        pthread_mutex_unlock(&st->lock);
        if (chg) { close(sfd); goto retry; }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sfd, &rfds);
        struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
        int sel = select(sfd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0) break;   /* fd error — reopen */
        if (sel == 0) {
            /* Timeout: check for port change (device unplugged/replugged) */
            pthread_mutex_lock(&st->lock);
            chg = st->serial_changed;
            pthread_mutex_unlock(&st->lock);
            if (chg) { close(sfd); goto retry; }
            continue;
        }

        char ch;
        ssize_t n;
        while ((n = read(sfd, &ch, 1)) == 1) {
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
                    uint16_t pid_snap;
                    strncpy(hid_snap, st->hidraw_path, sizeof(hid_snap) - 1);
                    hid_snap[sizeof(hid_snap)-1] = '\0';
                    pid_snap = st->pid;
                    pthread_mutex_unlock(&st->lock);

                    /* Read HID outside the lock — no blocking with mutex held */
                    HidStatus hid = {0};
                    (void)read_hid_status(hid_snap, pid_snap, &hid);

                    /* Re-lock, store HID results, broadcast to SSE clients */
                    pthread_mutex_lock(&st->lock);
                    st->hid = hid;
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
        /* read() returned 0 (EOF) or an error other than EAGAIN/EWOULDBLOCK.
         * EOF means the USB serial device was cleanly removed; EIO means it
         * was yanked mid-read.  Either way, close and reopen. */
        if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
            break;
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
    uint16_t  pid        = st->pid;
    HidStatus hid        = st->hid;
    NmeaInfo nmea        = st->nmea;
    int      nmea_valid  = st->nmea_valid;
    pthread_mutex_unlock(&st->lock);

    *device_present = hid.valid;

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
    if (pid == PID_LBE_1421)
        JCAT("  \"model\": \"LBE-1421\",\n");
    else if (pid == PID_LBE_1420)
        JCAT("  \"model\": \"LBE-1420\",\n");
    else
        JCAT("  \"model\": null,\n");

    if (hid.valid) {
        JCAT("  \"device_status\": {\n");
        JCAT("    \"gps_lock\":        %s,\n", (hid.status & GPS_LOCK_BIT) ? "true" : "false");
        JCAT("    \"pll_lock\":        %s,\n", (hid.status & PLL_LOCK_BIT) ? "true" : "false");
        JCAT("    \"antenna_ok\":      %s,\n", (hid.status & ANT_OK_BIT)   ? "true" : "false");
        JCAT("    \"mode\":            \"%s\",\n", hid.fll ? "FLL" : "PLL");
        JCAT("    \"output1_enabled\": %s,\n", (hid.status & OUT1_EN_BIT) ? "true" : "false");
        JCAT("    \"output1_pps\":     %s,\n", (pid == PID_LBE_1421 && (hid.status & PPS1_BIT)) ? "true" : "false");
        JCAT("    \"output1_drive\":   \"%s\",\n", hid.out1_low ? "low" : "normal");
        JCAT("    \"frequency_hz\":    %u,\n", hid.frequency1_hz);
        JCAT("    \"frequency1_hz\":   %u,\n", hid.frequency1_hz);
        JCAT("    \"output2_enabled\": %s,\n", (pid == PID_LBE_1421 && (hid.status & OUT2_EN_BIT)) ? "true" : "false");
        JCAT("    \"output2_drive\":   \"%s\",\n", hid.out2_low ? "low" : "normal");
        JCAT("    \"frequency2_hz\":   %u\n", hid.frequency2_hz);
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
                        uint8_t buf[LBE_REPORT_SIZE],
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
    int res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
    close(fd);
    if (res < 0) {
        snprintf(errbuf, errbuf_sz, "ioctl: %s", strerror(errno));
        return -2;
    }
    return 0;
}

/* Write a complete HTTP response segment. write(2) may complete only part
 * of a buffer, and checking the result avoids ignored-result warnings. */
static int write_all(int fd, const void *data, size_t len)
{
    const char *p = data;
    while (len > 0) {
        ssize_t written = write(fd, p, len);
        if (written < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (written == 0)
            return -1;
        p += written;
        len -= (size_t)written;
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
    if (hlen > 0 && write_all(cfd, header, (size_t)hlen) == 0)
        (void)write_all(cfd, body, (size_t)blen);
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
/* ---- Heartbeat thread: polls HID and broadcasts every 5s regardless of NMEA.
 * This ensures SSE clients receive device status updates even when no GPS
 * signal is present and no NMEA sentences are flowing. */
static void *heartbeat_thread(void *arg)
{
    ServerState *st = (ServerState *)arg;
    while (1) {
        sleep(5);

        /* Snapshot HID path under lock */
        char hid_snap[64] = {0};
        uint16_t pid_snap = 0;
        pthread_mutex_lock(&st->lock);
        strncpy(hid_snap, st->hidraw_path, sizeof(hid_snap) - 1);
        pid_snap = st->pid;
        pthread_mutex_unlock(&st->lock);

        /* Read HID outside the lock */
        HidStatus hid = {0};
        (void)read_hid_status(hid_snap, pid_snap, &hid);

        /* Store results and broadcast to SSE clients */
        pthread_mutex_lock(&st->lock);
        if (hid.valid) {
            st->hid = hid;
        } else if (hid_snap[0]) {
            /* Device was present but read failed — mark invalid */
            st->hid.valid = 0;
        }
        pthread_cond_broadcast(&st->new_data);
        pthread_mutex_unlock(&st->lock);
    }
    return NULL;
}

/* Match exactly one request path.  A prefix-only match would make
 * /config/frequency2 accidentally invoke the legacy /config/frequency
 * handler. */
static int request_path_is(const char *request, size_t offset,
                           const char *path)
{
    size_t path_len = strlen(path);
    return strncmp(request + offset, path, path_len) == 0 &&
           (request[offset + path_len] == ' ' ||
            request[offset + path_len] == '?');
}

static void serve_http(int port, int readonly)
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
    pthread_t monitor_tid, nmea_tid, heartbeat_tid;
    pthread_create(&monitor_tid,   NULL, device_monitor_thread, &st);
    pthread_create(&nmea_tid,      NULL, nmea_stream_thread,    &st);
    pthread_create(&heartbeat_tid, NULL, heartbeat_thread,      &st);

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

    fprintf(stderr, "Serving on http://0.0.0.0:%d/%s\n", port,
            readonly ? "  [READ-ONLY mode]" : "");
    fprintf(stderr, "  GET  /                   -> HTML dashboard\n");
    fprintf(stderr, "  GET  /json               -> JSON snapshot\n");
    fprintf(stderr, "  GET  /events             -> SSE stream\n");
    if (!readonly) {
        fprintf(stderr, "  POST /config/frequency   -> {\"frequency_hz\":N[,\"save\":true]}\n");
        fprintf(stderr, "  POST /config/frequency2  -> {\"frequency_hz\":N[,\"save\":true]} (1421)\n");
        fprintf(stderr, "  POST /config/output1     -> {\"enabled\":true|false}\n");
        fprintf(stderr, "  POST /config/power1      -> {\"low\":true|false}\n");
        fprintf(stderr, "  POST /config/power2      -> {\"low\":true|false} (1421)\n");
        fprintf(stderr, "  POST /config/pll         -> {\"fll\":true|false}\n");
        fprintf(stderr, "  POST /config/pps         -> {\"enabled\":true|false} (1421)\n");
        fprintf(stderr, "  POST /config/blink       -> {}\n");
    } else {
        fprintf(stderr, "  POST /config/*           -> 403 Forbidden (read-only mode)\n");
    }

    /* HTML Variables */
    const char *model_name = "LBE-142x";

    /* Embedded HTML dashboard template.
     * snprintf substitutions (4 total, in order):
     *   %s  → Model name
     *   %s  → Model name
     *   %s  → "true" or "false"  (READONLY JS variable)
     *   %s  → prefix             (all HTTP paths)
     * prefix comes from X-Forwarded-Prefix header (e.g. "/gpsdo"), default "".
     */
    static const char html_template[] =
"<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>Leo Bodnar %s</title>"
"<link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\"/>"
"<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>"
"<style>"
"*{box-sizing:border-box}"
"body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#f5f6fa;color:#2d3748;margin:0;padding:1.2em;font-size:14px}"
"h1{color:#1a202c;font-size:1.3em;margin:0 0 .6em;font-weight:600;letter-spacing:.01em}"
".card{background:#fff;border:1px solid #e2e8f0;border-radius:6px;padding:1em;margin:.6em 0;box-shadow:0 1px 3px rgba(0,0,0,.06)}"
".label{color:#718096;font-size:.8em;text-transform:uppercase;letter-spacing:.04em;margin-bottom:.15em}"
".val{color:#1a202c;font-size:.95em;font-weight:500}"
".ok{color:#276749}.warn{color:#b7791f}.err{color:#c53030}"
".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:.6em}"
"#status{color:#718096;font-size:.8em;margin-bottom:.6em}"
"#map{height:300px;margin:.6em 0;border:1px solid #e2e8f0;border-radius:6px;display:none}"
".ctrl-row{display:flex;align-items:center;gap:.8em;flex-wrap:wrap;margin:.4em 0}"
".ctrl-row label{color:#4a5568;font-size:.85em;min-width:7em;font-weight:500}"
"input[type=number]{background:#fff;color:#1a202c;border:1px solid #cbd5e0;border-radius:4px;padding:.3em .5em;font-family:inherit;width:10em;font-size:.9em}"
"input[type=number]:focus{outline:none;border-color:#4299e1;box-shadow:0 0 0 2px rgba(66,153,225,.2)}"
"input[type=checkbox]{accent-color:#4299e1;width:1.1em;height:1.1em;cursor:pointer}"
"button{background:#4299e1;color:#fff;border:none;border-radius:4px;padding:.3em .9em;font-family:inherit;font-size:.85em;font-weight:500;cursor:pointer;transition:background .15s}"
"button:hover{background:#3182ce}"
"button:active{background:#2b6cb0}"
"select{background:#fff;color:#1a202c;border:1px solid #cbd5e0;border-radius:4px;padding:.3em .5em;font-family:inherit;font-size:.9em;cursor:pointer}"
"select:focus{outline:none;border-color:#4299e1;box-shadow:0 0 0 2px rgba(66,153,225,.2)}"
"#ctrl_status{color:#718096;font-size:.8em;margin-top:.5em}"
"#readonly_banner{background:#fffbeb;color:#92400e;border:1px solid #fcd34d;border-radius:4px;padding:.4em .7em;font-size:.8em;margin-bottom:.6em;display:none}"
".dual{display:none}"
"</style></head><body>"
"<h1>&#x1F4E1; Leo Bodnar %s</h1>"
"<div id=\"readonly_banner\">&#x1F512; Read-only mode &mdash; configuration changes are disabled</div>"
"<div id=\"status\">Connecting...</div>"
"<div class=\"card\">"
"  <div class=\"label\">Device</div>"
"  <div class=\"grid\">"
"    <div><div class=\"label\">Model</div><div class=\"val\" id=\"model\">-</div></div>"
"    <div><div class=\"label\">HID path</div><div class=\"val\" id=\"device\">-</div></div>"
"    <div><div class=\"label\">Serial</div><div class=\"val\" id=\"serial\">-</div></div>"
"    <div><div class=\"label\">GPS lock</div><div class=\"val\" id=\"gps_lock\">-</div></div>"
"    <div><div class=\"label\">PLL lock</div><div class=\"val\" id=\"pll_lock\">-</div></div>"
"    <div><div class=\"label\">Antenna</div><div class=\"val\" id=\"antenna\">-</div></div>"
"    <div><div class=\"label\">Mode</div><div class=\"val\" id=\"mode\">-</div></div>"
"    <div><div class=\"label\">Output 1</div><div class=\"val\" id=\"out1\">-</div></div>"
"    <div><div class=\"label\">Output 1 drive</div><div class=\"val\" id=\"drive1\">-</div></div>"
"    <div><div class=\"label\">Output 1 frequency</div><div class=\"val\" id=\"freq1\">-</div></div>"
"    <div class=\"dual\"><div class=\"label\">Output 2</div><div class=\"val\" id=\"out2\">-</div></div>"
"    <div class=\"dual\"><div class=\"label\">Output 2 drive</div><div class=\"val\" id=\"drive2\">-</div></div>"
"    <div class=\"dual\"><div class=\"label\">Output 2 frequency</div><div class=\"val\" id=\"freq2\">-</div></div>"
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
"    <label>Output(s)</label>"
"    <input type=\"checkbox\" id=\"ctrl_out1\" onchange=\"setOutput1(this.checked)\">"
"    <span class=\"val\" id=\"ctrl_out1_lbl\">-</span>"
"  </div>"
"  <div class=\"ctrl-row\">"
"    <label>Output 1 Frequency (Hz)</label>"
"    <input type=\"number\" id=\"ctrl_freq1\" min=\"1\" max=\"1600000000\" step=\"1\" placeholder=\"Hz\" oninput=\"this.value=this.value.replace(/[^0-9]/g,'')\">"
"    <button onclick=\"setFreq1()\">Set</button>"
"    <button onclick=\"setFreq1Val(27000000)\">27 MHz</button>"
"    <input type=\"checkbox\" id=\"ctrl_save1\" checked>"
"    <span class=\"label\">save to flash</span>"
"  </div>"
"  <div class=\"ctrl-row\">"
"    <label>Output 1 Drive</label>"
"    <select id=\"ctrl_pwr1\" onchange=\"setPower1(this.value==='low')\">"
"      <option value=\"normal\">Normal</option>"
"      <option value=\"low\">Low power</option>"
"    </select>"
"  </div>"
"  <div class=\"ctrl-row\">"
"    <label>Discipline mode</label>"
"    <select id=\"ctrl_mode\" onchange=\"setPllMode(this.value==='fll')\">"
"      <option value=\"pll\">PLL</option>"
"      <option value=\"fll\">FLL</option>"
"    </select>"
"  </div>"
"  <div id=\"dual_controls\" style=\"display:none\">"
"    <div class=\"ctrl-row\">"
"      <label>Output 2 Frequency (Hz)</label>"
"      <input type=\"number\" id=\"ctrl_freq2\" min=\"1\" max=\"1400000000\" step=\"1\" placeholder=\"Hz\" oninput=\"this.value=this.value.replace(/[^0-9]/g,'')\">"
"      <button onclick=\"setFreq2()\">Set</button>"
"      <button onclick=\"setFreq2Val(27000000)\">27 MHz</button>"
"      <input type=\"checkbox\" id=\"ctrl_save2\" checked>"
"      <span class=\"label\">save to flash</span>"
"    </div>"
"    <div class=\"ctrl-row\">"
"      <label>Output 2 Drive</label>"
"      <select id=\"ctrl_pwr2\" onchange=\"setPower2(this.value==='low')\">"
"        <option value=\"normal\">Normal</option>"
"        <option value=\"low\">Low power</option>"
"      </select>"
"    </div>"
"    <div class=\"ctrl-row\">"
"      <label>1-PPS on output 1</label>"
"      <input type=\"checkbox\" id=\"ctrl_pps\" onchange=\"setPps(this.checked)\">"
"    </div>"
"  </div>"
"  <div class=\"ctrl-row\">"
"    <label>LED</label>"
"    <button onclick=\"doBlink()\">Blink LED(s)</button>"
"  </div>"
"  <div id=\"ctrl_status\"></div>"
"</div>"
"<script>"
"var READONLY=%s;"  /* substituted: "true" or "false" */
"var PREFIX='%s';"
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
/* Fix Leaflet default marker icon paths broken when loaded from CDN */
"var _iconBase='https://unpkg.com/leaflet@1.9.4/dist/images/';"
"L.Icon.Default.prototype.options.iconUrl=_iconBase+'marker-icon.png';"
"L.Icon.Default.prototype.options.iconRetinaUrl=_iconBase+'marker-icon-2x.png';"
"L.Icon.Default.prototype.options.shadowUrl=_iconBase+'marker-shadow.png';"
"L.Icon.Default.imagePath='';"
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
"  set('model', d.model||'-');"
"  set('device', d.device||'none');"
"  set('serial', d.serial||'none');"
"  var dual=d.model==='LBE-1421';"
"  Array.prototype.forEach.call(document.querySelectorAll('.dual'),function(e){e.style.display=dual?'block':'none';});"
"  var dualControls=document.getElementById('dual_controls');"
"  if(dualControls)dualControls.style.display=dual?'block':'none';"
"  var s=d.device_status;"
"  if(s){"
"    set('gps_lock', s.gps_lock?'LOCKED':'NO LOCK', s.gps_lock?'ok':'err');"
"    set('pll_lock', s.pll_lock?'LOCKED':'unlocked', s.pll_lock?'ok':'warn');"
"    set('antenna',  s.antenna_ok?'OK':'SHORT CIRCUIT', s.antenna_ok?'ok':'err');"
"    set('mode',     s.mode);"
"    var o1=s.output1_enabled?(s.output1_pps?'enabled (1-PPS)':'enabled'):'disabled';"
"    set('out1', o1, s.output1_enabled?'ok':'warn');"
"    set('drive1',   s.output1_drive);"
"    set('freq1',    fmtFreq(s.frequency1_hz));"
"    if(dual){"
"      set('out2',s.output2_enabled?'enabled':'disabled',s.output2_enabled?'ok':'warn');"
"      set('drive2',s.output2_drive);"
"      set('freq2',fmtFreq(s.frequency2_hz));"
"    }"
"    /* Sync controls from live state (only if user isn't actively editing) */"
"    var cb=document.getElementById('ctrl_out1');"
"    if(cb&&document.activeElement!==cb){"
"      cb.checked=s.output1_enabled;"
"      document.getElementById('ctrl_out1_lbl').textContent=s.output1_enabled?'enabled':'disabled';"
"    }"
"    var fi=document.getElementById('ctrl_freq1');"
"    if(fi&&document.activeElement!==fi){"
"      fi.value=s.frequency1_hz;"
"    }"
"    var pw=document.getElementById('ctrl_pwr1');"
"    if(pw&&document.activeElement!==pw){"
"      pw.value=(s.output1_drive==='low')?'low':'normal';"
"    }"
"    var mode=document.getElementById('ctrl_mode');"
"    if(mode&&document.activeElement!==mode)mode.value=(s.mode||'PLL').toLowerCase();"
"    if(dual){"
"      var f2=document.getElementById('ctrl_freq2');"
"      if(f2&&document.activeElement!==f2)f2.value=s.frequency2_hz;"
"      var p2=document.getElementById('ctrl_pwr2');"
"      if(p2&&document.activeElement!==p2)p2.value=(s.output2_drive==='low')?'low':'normal';"
"      var pps=document.getElementById('ctrl_pps');"
"      if(pps&&document.activeElement!==pps)pps.checked=s.output1_pps;"
"    }"
"  } else {"
"    ['gps_lock','pll_lock','antenna','mode','out1','drive1','freq1','out2','drive2','freq2'].forEach(function(i){set(i,'-');});"
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
"  apiPost(PREFIX+'/config/output1',{enabled:enabled});"
"}"
"function setFreq1Val(hz){"
"  document.getElementById('ctrl_freq1').value=hz;"
"  var save=document.getElementById('ctrl_save1').checked;"
"  apiPost(PREFIX+'/config/frequency',{frequency_hz:hz,save:save});"
"}"
"function setFreq1(){"
"  var hz=parseInt(document.getElementById('ctrl_freq1').value,10);"
"  if(!hz||hz<1){document.getElementById('ctrl_status').textContent='Enter a valid frequency in Hz';return;}"
"  var save=document.getElementById('ctrl_save1').checked;"
"  apiPost(PREFIX+'/config/frequency',{frequency_hz:hz,save:save});"
"}"
"function setFreq2Val(hz){"
"  document.getElementById('ctrl_freq2').value=hz;"
"  var save=document.getElementById('ctrl_save2').checked;"
"  apiPost(PREFIX+'/config/frequency2',{frequency_hz:hz,save:save});"
"}"
"function setFreq2(){"
"  var hz=parseInt(document.getElementById('ctrl_freq2').value,10);"
"  if(!hz||hz<1){document.getElementById('ctrl_status').textContent='Enter a valid frequency in Hz';return;}"
"  var save=document.getElementById('ctrl_save2').checked;"
"  apiPost(PREFIX+'/config/frequency2',{frequency_hz:hz,save:save});"
"}"
"function setPower1(low){"
"  apiPost(PREFIX+'/config/power1',{low:low});"
"}"
"function setPower2(low){"
"  apiPost(PREFIX+'/config/power2',{low:low});"
"}"
"function setPllMode(fll){"
"  apiPost(PREFIX+'/config/pll',{fll:fll});"
"}"
"function setPps(enabled){"
"  apiPost(PREFIX+'/config/pps',{enabled:enabled});"
"}"
"function doBlink(){"
"  apiPost(PREFIX+'/config/blink',{});"
"}"
"var es=new EventSource(PREFIX+'/events');"
"es.onopen=function(){document.getElementById('status').textContent='Connected - live updates via SSE';};"
"es.onerror=function(){document.getElementById('status').textContent='SSE disconnected - retrying...';};"
"es.onmessage=function(e){"
"  evtCount++;"
"  document.getElementById('status').textContent='Connected - events received: '+evtCount;"
"  try{update(JSON.parse(e.data));}catch(ex){"
"    document.getElementById('status').textContent='JSON parse error: '+ex.message;"
"  }"
"};"
"if(READONLY){"
"  document.getElementById('ctrl_card').style.display='none';"
"  document.getElementById('readonly_banner').style.display='block';"
"}"
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
        int is_sse  = is_get  && request_path_is(req, 4, "/events");
        int is_json = is_get  && request_path_is(req, 4, "/json");
        int is_post_freq  = is_post && request_path_is(req, 5, "/config/frequency");
        int is_post_freq2 = is_post && request_path_is(req, 5, "/config/frequency2");
        int is_post_out1  = is_post && request_path_is(req, 5, "/config/output1");
        int is_post_pwr1  = is_post && request_path_is(req, 5, "/config/power1");
        int is_post_pwr2  = is_post && request_path_is(req, 5, "/config/power2");
        int is_post_pll   = is_post && request_path_is(req, 5, "/config/pll");
        int is_post_pps   = is_post && request_path_is(req, 5, "/config/pps");
        int is_post_blink = is_post && request_path_is(req, 5, "/config/blink");

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
            if (hlen > 0 && write_all(cfd, header, (size_t)hlen) == 0)
                (void)write_all(cfd, body, body_len);
            if (json) free(json);
            close(cfd);
        } else if (is_post_freq || is_post_freq2 || is_post_out1 ||
                   is_post_pwr1 || is_post_pwr2 || is_post_pll ||
                   is_post_pps || is_post_blink) {
            /* --- Config POST endpoints ---------------------------------- */

            /* Reject writes in read-only mode */
            if (readonly) {
                static const char ro_body[] = "{\"ok\":false,\"error\":\"read-only mode\"}\n";
                char ro_hdr[256];
                int rhl = snprintf(ro_hdr, sizeof(ro_hdr),
                    "HTTP/1.1 403 Forbidden\r\n"
                    "Content-Type: application/json\r\n"
                    "Content-Length: %zu\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Connection: close\r\n"
                    "\r\n",
                    sizeof(ro_body) - 1);
                if (rhl > 0 && write_all(cfd, ro_hdr, (size_t)rhl) == 0)
                    (void)write_all(cfd, ro_body, sizeof(ro_body) - 1);
                close(cfd);
                continue;
            }

            /* Extract body: find \r\n\r\n separator */
            const char *body_start = strstr(req, "\r\n\r\n");
            if (body_start) body_start += 4;
            else body_start = "";

            /* Snapshot HID path under lock */
            char hid_snap[64] = {0};
            uint16_t pid_snap = 0;
            pthread_mutex_lock(&st.lock);
            strncpy(hid_snap, st.hidraw_path, sizeof(hid_snap) - 1);
            pid_snap = st.pid;
            pthread_mutex_unlock(&st.lock);

            uint8_t hbuf[LBE_REPORT_SIZE];
            memset(hbuf, 0, sizeof(hbuf));
            char errbuf[128] = {0};
            int rc;

            if (is_post_freq || is_post_freq2) {
                long freq_hz = 0;
                if (!json_get_long(body_start, "frequency_hz", &freq_hz) ||
                    freq_hz <= 0 || (unsigned long)freq_hz > UINT32_MAX) {
                    send_api_response(cfd, 0, 0, "missing or invalid frequency_hz");
                    close(cfd); continue;
                }
                int save_flag = 1;  /* default: save to flash */
                int save_val  = 1;
                if (json_get_bool(body_start, "save", &save_val))
                    save_flag = save_val;

                if (build_frequency_command(pid_snap, is_post_freq2 ? 2 : 1,
                                            save_flag, (uint32_t)freq_hz, hbuf) < 0) {
                    send_api_response(cfd, 0, 0,
                        is_post_freq2 ? "frequency2 is unsupported or out of range" :
                                        "frequency is out of range");
                    close(cfd); continue;
                }
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));

            } else if (is_post_out1) {
                int enabled = -1;
                if (!json_get_bool(body_start, "enabled", &enabled)) {
                    send_api_response(cfd, 0, 0, "missing or invalid enabled");
                    close(cfd); continue;
                }
                build_outputs_enable_command(pid_snap, enabled, hbuf);
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));

            } else if (is_post_pwr1 || is_post_pwr2) {
                int low = -1;
                if (!json_get_bool(body_start, "low", &low)) {
                    send_api_response(cfd, 0, 0, "missing or invalid low");
                    close(cfd); continue;
                }
                if (build_power_command(pid_snap, is_post_pwr2 ? 2 : 1,
                                        low, hbuf) < 0) {
                    send_api_response(cfd, 0, 0,
                        "power2 is only supported by LBE-1421");
                    close(cfd); continue;
                }
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));

            } else if (is_post_pll) {
                int fll = -1;
                if (!json_get_bool(body_start, "fll", &fll)) {
                    send_api_response(cfd, 0, 0, "missing or invalid fll");
                    close(cfd); continue;
                }
                build_pll_command(fll, hbuf);
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));

            } else if (is_post_pps) {
                int enabled = -1;
                if (!json_get_bool(body_start, "enabled", &enabled)) {
                    send_api_response(cfd, 0, 0, "missing or invalid enabled");
                    close(cfd); continue;
                }
                if (build_pps_command(pid_snap, enabled, hbuf) < 0) {
                    send_api_response(cfd, 0, 0,
                        "1PPS control is only supported by LBE-1421");
                    close(cfd); continue;
                }
                rc = hid_write_op(hid_snap, hbuf, errbuf, sizeof(errbuf));

            } else {  /* is_post_blink */
                hbuf[0] = LBE_142X_BLINK;
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
            /* HTML dashboard — substitute READONLY and PREFIX into the template.
             * PREFIX comes from the X-Forwarded-Prefix request header (set by a
             * reverse proxy such as UberSDR's /gpsdo/ route).  When accessed
             * directly the header is absent and prefix defaults to "", so all
             * paths remain absolute (e.g. /events, /config/output1). */
            char prefix[128] = "";
            const char *pfx_hdr = strcasestr(req, "X-Forwarded-Prefix:");
            if (pfx_hdr) {
                pfx_hdr += 19; /* skip "X-Forwarded-Prefix:" */
                while (*pfx_hdr == ' ') pfx_hdr++;
                const char *pfx_end = strpbrk(pfx_hdr, "\r\n");
                size_t pfx_len = pfx_end ? (size_t)(pfx_end - pfx_hdr) : strlen(pfx_hdr);
                if (pfx_len > 0 && pfx_len < sizeof(prefix))
                    memcpy(prefix, pfx_hdr, pfx_len);
            }

            /* Compute body length first (snprintf returns the would-be length) */
            const char *ro_str = readonly ? "true" : "false";
            int body_len = snprintf(NULL, 0, html_template,
                model_name, model_name, ro_str, prefix);
            if (body_len < 0) { close(cfd); continue; }

            char *body = malloc((size_t)body_len + 1);
            if (!body) { close(cfd); continue; }
            snprintf(body, (size_t)body_len + 1, html_template,
                model_name, model_name, ro_str, prefix);

            char header[128];
            int hlen = snprintf(header, sizeof(header),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html; charset=utf-8\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n",
                body_len);
            if (hlen > 0 && write_all(cfd, header, (size_t)hlen) == 0)
                (void)write_all(cfd, body, (size_t)body_len);
            free(body);
            close(cfd);
        }
    }
    close(srv);
}

int processCommandLineArguments(int argc, char **argv,
                                int *freq1, int *freq2, int *blink, int *enable,
                                int *save1, int *save2, int *power1, int *power2,
                                int *fll_mode, int *pps, char **serial_port,
                                int *json_out, int *serve, int *port, int *readonly);

int main(int argc, char **argv)
{
      /* Quick pre-scan for --json / -j, --serve, --readonly before any output.
       * Environment variables are checked first so CLI args can override. */
      int json_mode = 0;
      int serve_mode = 0;
      int serve_port = HTTP_DEFAULT_PORT;
      int readonly_mode = 0;

      /* Env var pre-population (lowest priority — CLI args override below) */
      {
          const char *ev;
          if ((ev = getenv("LBE_SERVE")) != NULL && atoi(ev)) {
              serve_mode = 1;
              json_mode  = 1;
          }
          if ((ev = getenv("LBE_JSON")) != NULL && atoi(ev))
              json_mode = 1;
          if ((ev = getenv("LBE_PORT")) != NULL && atoi(ev) > 0)
              serve_port = atoi(ev);
          if ((ev = getenv("LBE_READONLY")) != NULL && atoi(ev))
              readonly_mode = 1;
      }

      for (int i = 1; i < argc; i++) {
          if (strcmp(argv[i], "--json") == 0 || strcmp(argv[i], "-j") == 0) {
              json_mode = 1;
          } else if (strcmp(argv[i], "--serve") == 0) {
              serve_mode = 1;
              json_mode  = 1;   /* serve mode implies JSON */
          } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
              serve_port = atoi(argv[++i]);
          } else if (strcmp(argv[i], "--readonly") == 0) {
              readonly_mode = 1;
          }
      }

      /* Propagate to global so serve_http() can read it */
      g_readonly = readonly_mode;

      if (!json_mode)
          printf("Leo Bodnar LBE-142x GPS locked clock source config\n\n");

      int fd;
      int res;
      u_int8_t buf[60];
      uint32_t current_f = 0;
      uint32_t current_f2 = 0;

      struct hidraw_devinfo info;

      /* Determine hidraw device path — auto-discover if not supplied */
      /* Use persistent buffers so pointers remain valid throughout main() */
      static char hidraw_path_buf[64];
      static char auto_serial_buf[64];
      const char *hidraw_path = NULL;
      const char *auto_serial = NULL;

      /* Check if first non-option arg looks like a device path,
       * or fall back to LBE_HIDRAW environment variable. */
      const char *env_hidraw = getenv("LBE_HIDRAW");
      if (argc >= 2 && argv[1][0] == '/') {
          hidraw_path = argv[1];
      } else if (env_hidraw && env_hidraw[0] == '/') {
          strncpy(hidraw_path_buf, env_hidraw, sizeof(hidraw_path_buf) - 1);
          hidraw_path = hidraw_path_buf;
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
                  printf("     --blink1  blink output LED(s) for 3 seconds\n");
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
              selected_pid = devs[0].product;
          } else {
              if (json_mode) {
                  /* In JSON mode, just pick the first device */
                  strncpy(hidraw_path_buf, devs[0].hidraw_path, sizeof(hidraw_path_buf) - 1);
                  hidraw_path = hidraw_path_buf;
                  if (devs[0].serial_path[0]) {
                      strncpy(auto_serial_buf, devs[0].serial_path, sizeof(auto_serial_buf) - 1);
                      auto_serial = auto_serial_buf;
                  }
                  selected_pid = devs[0].product;
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
                  selected_pid = devs[choice].product;
                  printf("\n");
              }
          }
      }

      /* In serve mode with no device present, skip straight to the server.
       * The monitor thread inside serve_http() will open the device later. */
      if (serve_mode && !hidraw_path) {
          serve_http(serve_port, readonly_mode);
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
            selected_pid = info.product;
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
      HidStatus hid = {0};
      if (res >= 0 && buf[0] == 0x01) {
          decode_hid_status(selected_pid, buf, &hid);
          current_f = hid.frequency1_hz;
          current_f2 = hid.frequency2_hz;
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
 int save1 = -1;
 int save2 = -1;
 int new_f = 0xffffffff;
 int new_f2 = 0xffffffff;
 int power1 = -1;
 int power2 = -1;
 int fll_mode = -1;
 int pps = -1;
 int json_out_unused = 0; /* already captured in json_mode */
 int serve_unused = 0, port_unused = 0; /* already captured in pre-scan */
 int readonly_unused = 0; /* already captured in readonly_mode */
 char *serial_port = NULL;
 processCommandLineArguments(argc, argv, &new_f, &new_f2, &blink, &enable,
                             &save1, &save2, &power1, &power2, &fll_mode, &pps,
                             &serial_port, &json_out_unused, &serve_unused,
                             &port_unused, &readonly_unused);

 /* Apply environment variable fallbacks for any option not set by CLI args.
  * CLI args always take priority over env vars. */
 {
     const char *ev;
     if (!serial_port && (ev = getenv("LBE_SERIAL")) != NULL && ev[0])
         serial_port = (char *)ev;
     if (new_f == (int)0xffffffff) {
         if ((ev = getenv("LBE_F1")) != NULL && atoi(ev) > 0) {
             new_f = atoi(ev);
             save1 = 1;
         } else if ((ev = getenv("LBE_F1_NOSAVE")) != NULL && atoi(ev) > 0) {
             new_f = atoi(ev);
             save1 = 0;
         }
     }
     if (new_f2 == (int)0xffffffff) {
         if ((ev = getenv("LBE_F2")) != NULL && atoi(ev) > 0) {
             new_f2 = atoi(ev);
             save2 = 1;
         } else if ((ev = getenv("LBE_F2_NOSAVE")) != NULL && atoi(ev) > 0) {
             new_f2 = atoi(ev);
             save2 = 0;
         }
     }
     if (enable == -1 && (ev = getenv("LBE_OUT1")) != NULL)
         enable = atoi(ev);
     if (blink == -1 && (ev = getenv("LBE_BLINK1")) != NULL && atoi(ev))
         blink = 1;
     if (power1 == -1 && (ev = getenv("LBE_PWR1")) != NULL)
         power1 = atoi(ev);
     if (power2 == -1 && (ev = getenv("LBE_PWR2")) != NULL)
         power2 = atoi(ev);
     if (fll_mode == -1 && (ev = getenv("LBE_FLL")) != NULL && atoi(ev))
         fll_mode = 1;
     if (fll_mode == -1 && (ev = getenv("LBE_PLL")) != NULL && atoi(ev))
         fll_mode = 0;
     if (pps == -1 && (ev = getenv("LBE_PPS")) != NULL)
         pps = atoi(ev);
 }

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
     printf("  \"model\": \"%s\",\n",
            selected_pid == PID_LBE_1421 ? "LBE-1421" : "LBE-1420");
     if (serial_port)
         printf("  \"serial\": \"%s\",\n", serial_port);
     if (hid.valid) {
         printf("  \"device_status\": {\n");
         printf("    \"gps_lock\":        %s,\n",  (hid.status & GPS_LOCK_BIT) ? "true" : "false");
         printf("    \"pll_lock\":        %s,\n",  (hid.status & PLL_LOCK_BIT) ? "true" : "false");
         printf("    \"antenna_ok\":      %s,\n",  (hid.status & ANT_OK_BIT)   ? "true" : "false");
         printf("    \"mode\":            \"%s\",\n", hid.fll ? "FLL" : "PLL");
         printf("    \"output1_enabled\": %s,\n", (hid.status & OUT1_EN_BIT) ? "true" : "false");
         printf("    \"output1_pps\":     %s,\n", (selected_pid == PID_LBE_1421 && (hid.status & PPS1_BIT)) ? "true" : "false");
         printf("    \"output1_drive\":   \"%s\",\n", hid.out1_low ? "low" : "normal");
         printf("    \"frequency_hz\":    %u,\n", current_f);
         printf("    \"frequency1_hz\":   %u,\n", current_f);
         printf("    \"output2_enabled\": %s,\n", (selected_pid == PID_LBE_1421 && (hid.status & OUT2_EN_BIT)) ? "true" : "false");
         printf("    \"output2_drive\":   \"%s\",\n", hid.out2_low ? "low" : "normal");
         printf("    \"frequency2_hz\":   %u\n", current_f2);
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
     if (hid.valid) {
         printf("  Device Status:\n");
         printf("    GPS lock:   %s\n", (hid.status & GPS_LOCK_BIT) ? "LOCKED"        : "NO LOCK");
         printf("    PLL lock:   %s\n", (hid.status & PLL_LOCK_BIT) ? "LOCKED"        : "unlocked");
         printf("    Antenna:    %s\n", (hid.status & ANT_OK_BIT)   ? "OK"            : "SHORT CIRCUIT");
         printf("    Mode:       %s\n", hid.fll                     ? "FLL"           : "PLL");
         printf("    Output 1:   %s",  (hid.status & OUT1_EN_BIT)   ? "enabled"       : "disabled");
         if (selected_pid == PID_LBE_1421 && (hid.status & PPS1_BIT)) printf("  (1-PPS mode)");
         printf("\n");
         printf("    Out1 drive: %s\n", hid.out1_low ? "low" : "normal");
         printf("    Frequency 1: %u Hz\n", current_f);
         if (selected_pid == PID_LBE_1421) {
             printf("    Output 2:   %s\n", (hid.status & OUT2_EN_BIT) ? "enabled" : "disabled");
             printf("    Out2 drive: %s\n", hid.out2_low ? "low" : "normal");
             printf("    Frequency 2: %u Hz\n", current_f2);
         }
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

 if (selected_pid != PID_LBE_1421 &&
     (new_f2 != (int)0xffffffff || power2 != -1 || pps != -1)) {
     fprintf(stderr, "Output 2 and 1PPS controls require an LBE-1421.\n");
     close(fd);
     return 2;
 }
 if ((enable != -1 && enable != 0 && enable != 1) ||
     (power1 != -1 && power1 != 0 && power1 != 1) ||
     (power2 != -1 && power2 != 0 && power2 != 1) ||
     (pps != -1 && pps != 0 && pps != 1)) {
     fprintf(stderr, "Boolean options must be 0 or 1.\n");
     close(fd);
     return 2;
 }

 if (!json_mode)
     printf("  Changes:\n");
 int changed = 0;
 if (readonly_mode) {
     if (!json_mode) printf("    Read-only mode — no changes applied\n");
 } else {
     if (new_f != (int)0xffffffff && (uint32_t)new_f != current_f) {
         if (build_frequency_command(selected_pid, 1, save1, (uint32_t)new_f,
                                     buf) < 0) {
             fprintf(stderr, "Output 1 frequency must be between 1 and %lu Hz.\n",
                     (unsigned long)(selected_pid == PID_LBE_1421 ? LBE_1421_MAX_FREQ : LBE_1420_MAX_FREQ));
         } else {
             if (!json_mode) printf("    Setting output 1 frequency: %d Hz\n", new_f);
             res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
             if (res < 0) perror("HIDIOCSFEATURE");
             else changed = 1;
         }
     }
     if (new_f2 != (int)0xffffffff && (uint32_t)new_f2 != current_f2) {
         if (build_frequency_command(selected_pid, 2, save2, (uint32_t)new_f2,
                                     buf) < 0) {
             fprintf(stderr, "Output 2 frequency must be between 1 and %lu Hz.\n",
                     (unsigned long)LBE_1421_MAX_FREQ);
         } else {
             if (!json_mode) printf("    Setting output 2 frequency: %d Hz\n", new_f2);
             res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
             if (res < 0) perror("HIDIOCSFEATURE");
             else changed = 1;
         }
     }
     if (enable != -1) {
         build_outputs_enable_command(selected_pid, enable, buf);
         if (!json_mode) printf("    Output%s enable: %d\n",
                                selected_pid == PID_LBE_1421 ? "s" : " 1", enable);
         res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
         if (res < 0) perror("HIDIOCSFEATURE");
         else changed = 1;
     }
     if (power1 != -1) {
         (void)build_power_command(selected_pid, 1, power1, buf);
         if (!json_mode) printf("    Setting output 1 drive: %s\n",
                                power1 ? "low" : "normal");
         res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
         if (res < 0) perror("HIDIOCSFEATURE");
         else changed = 1;
     }
     if (power2 != -1) {
         (void)build_power_command(selected_pid, 2, power2, buf);
         if (!json_mode) printf("    Setting output 2 drive: %s\n",
                                power2 ? "low" : "normal");
         res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
         if (res < 0) perror("HIDIOCSFEATURE");
         else changed = 1;
     }
     if (fll_mode != -1) {
         build_pll_command(fll_mode, buf);
         if (!json_mode) printf("    Setting discipline mode: %s\n",
                                fll_mode ? "FLL" : "PLL");
         res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
         if (res < 0) perror("HIDIOCSFEATURE");
         else changed = 1;
     }
     if (pps != -1) {
         (void)build_pps_command(selected_pid, pps, buf);
         if (!json_mode) printf("    1-PPS on output 1: %s\n", pps ? "enabled" : "disabled");
         res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
         if (res < 0) perror("HIDIOCSFEATURE");
         else changed = 1;
     }
     if (blink != -1) {
         memset(buf, 0, sizeof(buf));
         buf[0] = LBE_142X_BLINK;
         if (!json_mode) printf("    Blink LED(s)\n");
         res = ioctl(fd, HIDIOCSFEATURE(LBE_REPORT_SIZE), buf);
         if (res < 0) perror("HIDIOCSFEATURE");
         else changed = 1;
     }
     if (!changed && !json_mode)
         printf("    No changes made\n");
 } /* end !readonly_mode */

 /* Start web server if requested (blocks forever) */
 if (serve_mode) {
     serve_http(serve_port, readonly_mode);
 }
      }
      close(fd);

      return 0;
}


int processCommandLineArguments(int argc, char **argv,
                                int *freq1, int *freq2, int *blink, int *enable,
                                int *save1, int *save2, int *power1, int *power2,
                                int *fll_mode, int *pps, char **serial_port,
                                int *json_out, int *serve, int *port, int *readonly)
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
                {"f2",        required_argument, 0, 'd'},
                {"f2_nosave", required_argument, 0, 'e'},
                {"fll",       no_argument,       0, 'f'},
                {"pll",       no_argument,       0, 'l'},
                {"pps",       required_argument, 0, 'q'},
                {"power1",    required_argument, 0, 'm'},
                {"pwr1",      required_argument, 0, 'm'},
                {"power2",    required_argument, 0, 'n'},
                {"pwr2",      required_argument, 0, 'n'},
                {"serial",    required_argument, 0, 's'},
                {"json",      no_argument,       0, 'j'},
                {"serve",     no_argument,       0, 'S'},
                {"port",      required_argument, 0, 'P'},
                {"readonly",  no_argument,       0, 'R'},
                {0, 0, 0, 0}
        };
        /* getopt_long stores the option index here. */
        int option_index = 0;

        c = getopt_long(argc, argv, "a:b:c:d:e:flq:m:n:s:jSP:R",
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
                *freq1 = atoi(optarg);
                *save1 = 1;
                break;

            case 'b'://f1_nosave
                *freq1 = atoi(optarg);
                *save1 = 0;
                break;

            case 'c'://out1
                *enable = atoi(optarg);
                break;

            case 'd'://f2
                *freq2 = atoi(optarg);
                *save2 = 1;
                break;

            case 'e'://f2_nosave
                *freq2 = atoi(optarg);
                *save2 = 0;
                break;

            case 'f'://fll
                *fll_mode = 1;
                break;

            case 'l'://pll
                *fll_mode = 0;
                break;

            case 'q'://pps
                *pps = atoi(optarg);
                break;

            case 'm'://power1
                *power1 = atoi(optarg);
                break;

            case 'n'://power2
                *power2 = atoi(optarg);
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

            case 'R'://readonly
                *readonly = 1;
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
