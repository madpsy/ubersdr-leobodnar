# lbe-142x — Leo Bodnar LBE-1420 GPS-Locked Clock Source Utility

A Linux command-line utility and web dashboard for configuring and monitoring the
[Leo Bodnar LBE-1420](http://www.leobodnar.com/shop/index.php?main_page=product_info&cPath=107&products_id=301)
GPS-locked clock source via its USB HID interface.

## Intended Use

This utility is designed for use with **[UberSDR](https://ubersdr.org/)** running an
**RX888 Mk II** SDR receiver. The RX888 Mk II accepts an external 27 MHz
reference clock input; the LBE-1420 provides a GPS-disciplined, ultra-stable clock
signal at the required frequency. The `--serve` mode allows the LBE-1420 to be
monitored and reconfigured remotely over the network without interrupting the SDR
session.

Typical setup:

```
LBE-1420 (USB HID + serial NMEA) ──► Linux host running lbe-142x --serve
                                                │
                                    clock output (SMA) ──► RX888 Mk II REF IN
```

## Prerequisites

- Linux with `hidraw` support (kernel ≥ 3.6)
- GCC and `pthread`
- Read/write access to `/dev/hidraw*` (add your user to the `plugdev` group, or run as root)

```
sudo usermod -aG plugdev $USER
# then log out and back in, or:
sudo udevadm control --reload-rules
```

A udev rule to grant group access:

```
# /etc/udev/rules.d/99-leobodnar.rules
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="1dd2", ATTRS{idProduct}=="2443", GROUP="plugdev", MODE="0660"
```

## Build

```
make
```

Or manually:

```
gcc -g -o lbe-142x lbe-142x.c -I. -lpthread
```

## Usage

```
lbe-142x [/dev/hidrawN] [options]
```

If no device path is given, the first LBE-1420 found is used automatically.
If multiple devices are present you will be prompted to choose.

### Options

| Option | Argument | Description |
|--------|----------|-------------|
| `--f1` | `<Hz>` | Set output frequency in Hz (1–1,600,000,000). Saved to flash. |
| `--f1_nosave` | `<Hz>` | Set output frequency in Hz. **Not** saved to flash (temporary). |
| `--out1` | `0` or `1` | Disable (`0`) or enable (`1`) output 1. |
| `--blink1` | — | Blink the output 1 LED for ~3 seconds. |
| `--serial` | `<port>` | Serial port for NMEA GPS data (e.g. `/dev/ttyACM0`). |
| `--json` | — | Output a single JSON snapshot to stdout and exit. |
| `--serve` | — | Start the HTTP web server (default port 5123). Blocks forever. |
| `--port` | `<N>` | HTTP port for `--serve` mode (default: 5123). |

### Examples

```bash
# Show device status (human-readable)
sudo ./lbe-142x

# Set frequency to 27 MHz, saved to flash
sudo ./lbe-142x --f1 27000000

# Set frequency to 10 MHz, not saved
sudo ./lbe-142x --f1_nosave 10000000

# Enable output 1
sudo ./lbe-142x --out1 1

# Blink LED
sudo ./lbe-142x --blink1

# JSON snapshot with NMEA data
sudo ./lbe-142x --json --serial /dev/ttyACM0

# Start web server on default port
sudo ./lbe-142x --serve --serial /dev/ttyACM0

# Start web server on port 8080
sudo ./lbe-142x --serve --port 8080 --serial /dev/ttyACM0
```

## Web Server (`--serve`)

When started with `--serve`, the binary listens on TCP port 5123 (or `--port N`)
and serves a live dashboard. The device and serial port are monitored continuously;
the server starts even if no device is plugged in at startup.

Open `http://<host>:5123/` in a browser.

### HTTP Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | HTML dashboard with live SSE updates and controls |
| `GET` | `/json` | One-shot JSON snapshot of current device state |
| `GET` | `/events` | Server-Sent Events stream (one JSON object per GGA sentence) |
| `POST` | `/config/frequency` | Set output frequency |
| `POST` | `/config/output1` | Enable or disable output 1 |
| `POST` | `/config/power1` | Set output 1 drive level |
| `POST` | `/config/blink` | Blink output 1 LED |

### POST Request Bodies

All POST endpoints accept and return `application/json`.

#### `POST /config/frequency`

```json
{ "frequency_hz": 27000000, "save": true }
```

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `frequency_hz` | integer | yes | — | Frequency in Hz (1–1,600,000,000) |
| `save` | boolean | no | `true` | Persist to flash if `true`; temporary if `false` |

#### `POST /config/output1`

```json
{ "enabled": true }
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `enabled` | boolean | yes | `true` to enable output, `false` to disable |

#### `POST /config/power1`

```json
{ "low": false }
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `low` | boolean | yes | `true` for low-power drive, `false` for normal drive |

#### `POST /config/blink`

```json
{}
```

No fields required. Triggers a ~3-second LED blink on output 1.

### POST Response

All POST endpoints return:

```json
{ "ok": true }
```

or on error:

```json
{ "ok": false, "error": "description" }
```

### SSE Event Format

Each event on `/events` is a JSON object:

```json
{
  "device": "/dev/hidraw0",
  "serial": "/dev/ttyACM0",
  "device_status": {
    "gps_lock":        true,
    "pll_lock":        true,
    "antenna_ok":      true,
    "mode":            "PLL",
    "output1_enabled": true,
    "output1_pps":     false,
    "output1_drive":   "normal",
    "frequency_hz":    27000000
  },
  "gps": {
    "datetime_utc": "2025-01-15T12:34:56Z",
    "fix":          "GPS",
    "fix_mode":     "3D",
    "sats_used":    8,
    "gps_in_view":  10,
    "glo_in_view":  6,
    "hdop":         0.90,
    "vdop":         1.20,
    "pdop":         1.50,
    "altitude_m":   42.1,
    "speed_knots":  0.012,
    "latitude":     51.500000,
    "longitude":    -0.120000
  }
}
```

`device_status` is `null` if the HID device is not available.
`gps` is `null` if no serial port is configured or no NMEA data has been received.

## Dashboard Features

- **Live status** — GPS lock, PLL lock, antenna, mode, output state, frequency, NMEA fix
- **Leaflet map** — shown automatically once a position fix is received; marker and tooltip update in real time
- **Controls card**:
  - Output 1 enable/disable checkbox (pre-populated from device state)
  - Frequency input (Hz) with "27 MHz" quick-set button and "save to flash" option
  - Output 1 drive level selector (Normal / Low power)
  - Blink LED button

## Device Notes

- Vendor ID: `0x1dd2`, Product ID: `0x2443`
- HID feature report `0x01` is used for status reads
- Frequency range: 1 Hz – 1.6 GHz
- The device retains settings in flash across power cycles when saved
