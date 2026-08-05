# CAGE

A ground-based radar and lidar rig built on a 1:1 skeletal mockup of the Overseer UAV
(820 mm nose-to-tail, 760 mm wing-to-wing).

It does two jobs. The first is to demonstrate a layered detection stack of the kind
that would fly on the real aircraft. The second is to measure how well different RF
shielding materials (RAM foam, copper mesh, ferrite sheet) block radar detection.

It only receives. There is no jamming or active interference involved.

## Detection layers

| Layer | Sensor | Range | Role |
|---|---|---|---|
| Early warning | HLK-LD2451 x1 | 10-100 m | Long-range forward cone, +/-15 deg |
| Close tracking | HLK-LD2450 x3 | 0-6 m | 360 deg multi-target tracking, +/-60 deg each |
| Fine spatial | LDROBOT LD06 | 0.02-12 m | 360 deg 2D lidar sweep, ~0.8 deg resolution |

The three LD2450s sit 120 degrees apart in an equilateral triangle. Each one covers
+/-60 degrees, so between them they tile a full circle with no gap.

## Architecture

```
LD2450 x3 --UART--> ESP32-S3 #1 --+-- WiFi/UDP --> laptop (Python scope)
                                  +-- HTTP ------> phone  (browser scope)

LD2451 + LD06 --UART--> ESP32-S3 #2 -- WiFi/UDP --> laptop
```

Each board reads its sensors over hardware UART and tags every detection with the
sensor ID and that sensor's mounting angle. The viewers use the angle to rotate each
sensor's detections into a shared airframe frame, then merge them into one picture.

## What's here

| Path | What it is |
|---|---|
| `board1_web_radar/` | Main firmware. Three radars, self-hosted web scope, UDP output |
| `board1_final_3radar/` | Same three radars, UDP only |
| `board1_udp_radar/` | Single radar over UDP, used during bring-up |
| `board1_phase2_ld2450_single/` | Single radar, serial output only |
| `laptop/radar_plot_360.py` | Merged 360 degree scope |
| `CAGE code v1 (redundant).py` | Early single-sensor scope. Superseded, kept for reference |

## Wiring, board #1

The data lines cross over: the LD2450's TX goes to the pin the sketch calls RX.

| Sensor | Mounting | LD2450 TX to | LD2450 RX to | UART |
|---|---|---|---|---|
| A | 0 deg (forward) | GPIO 18 | GPIO 17 | UART1 |
| B | +120 deg | GPIO 16 | GPIO 15 | UART2 |
| C | -120 deg | GPIO 5 | GPIO 4 | UART0 |

All three share 5V and ground. Total draw is around 1 A. Use a 5V/2A supply, not a
PC USB port. A PC port can't feed three radars plus the ESP32, and the first thing
that fails is WiFi association, which looks like a network problem rather than a
power one.

Set `USB CDC On Boot` to `Enabled` in the IDE. Sensor C sits on UART0, which is
normally the serial console, so the console has to move to the S3's native USB. That
means you upload on one COM port and read the serial monitor on a different one.

## The library needs patching

This uses [rbegamer/HLK-LD2450](https://github.com/rbegamer/HLK-LD2450), which decodes
coordinates wrongly out of the box. It reads X, Y and speed as two's complement, but
the sensor sends them as sign-magnitude: bit 15 is the sign, bits 0-14 the magnitude.
Left alone, a target at +300 mm comes out as -32468 mm.

In `LD2450.cpp`, in `ProcessSerialDataIntoRadarData()`:

```cpp
uint16_t rawX = rec_buf[index] | (rec_buf[index + 1] << 8);
target.x = (rawX & 0x8000) ? (int16_t)(rawX & 0x7FFF) : -(int16_t)(rawX & 0x7FFF);
// same for y and speed
```

`distance` is calculated from x and y, so it comes right on its own once those are
fixed. Reinstalling the library silently undoes this, so check it if readings suddenly
go strange.

Two smaller things about the same library. Its `valid` flag is set from
`resolution != 0` and can't be relied on, so filter on whether the distance is
physically plausible instead. And `RadarTarget` is declared inside the class, so it's
`LD2450::RadarTarget`, not `RadarTarget`.

## Running it

Firmware:

1. Copy `secrets.example.h` to `secrets.h` in the sketch folder and put your WiFi
   details in it. `secrets.h` is gitignored.
2. Board `ESP32S3 Dev Module`, `USB CDC On Boot` enabled, 8 MB flash.
3. Install `HLK-LD2450` from Library Manager and patch it as above.
4. Upload.

For the phone view, open `http://<board-ip>/` in a browser. The ESP32 serves the page
itself so there's nothing to install. On iOS `http://cage.local/` also works; Android
is unreliable with `.local` so use the IP.

For the laptop view:

```bash
pip install matplotlib numpy
python laptop/radar_plot_360.py
```

## Filtering

Raw LD2450 output is messy. One person shows up as several targets because arms, legs
and torso all reflect separately, and anywhere two sensors overlap they both report
the same person independently. Both viewers run the same three steps before drawing
anything:

1. Detections within 0.6 m of each other are merged into one.
2. What's left is matched against existing tracks frame to frame and smoothed.
3. A track has to survive three frames before it gets drawn, which kills most ghosts.

The constants are at the top of each viewer if they need retuning.

## Progress

- [x] WiFi and UDP working
- [x] One LD2450 reading correctly (this is where the decode bug turned up)
- [x] Three LD2450s merged into one 360 degree view, plus the phone viewer
- [ ] LD2451 and LD06 on board #2
- [ ] Print the airframe, mount everything, run the shielding tests

## Radome note

At 24 GHz the wavelength inside PETG is about 7.2 mm. A wall in front of a radar
wants to be either much thinner than that (1 mm or less) or a multiple of a half
wave, so around 3.6 mm or 7.2 mm. Odd quarter-wave thicknesses near 1.8 mm and 5.4 mm
reflect the most and are worth avoiding. Print radar windows solid, since sparse
infill leaves air gaps that scatter unpredictably.
