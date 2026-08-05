"""
CAGE, merged 360 degree radar scope for the 3x LD2450 triangular array.

Run:  python radar_plot_360.py
Listens on UDP :5005 for packets from board #1.

Packet format, one per sensor:  "<id>,<mountAngleDeg>:<x>,<y>,<dist>,<speed>;..."
e.g.  "A,0:252,335,419,0"   or   "B,120:-"  when that sensor sees nothing.

Raw LD2450 output is noisy and reports one person as several targets, since body
parts reflect separately and overlapping sensors each see the same person. So this
does three things before drawing:
  1. CLUSTER  detections closer than CLUSTER_RADIUS_M merge into one target
  2. TRACK    clusters are matched to persistent tracks and position-smoothed
  3. CONFIRM  a track must be seen MIN_HITS frames before it is drawn
Tune the constants below if it feels too twitchy or too sluggish.
"""

import socket
import math
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Helvetica", "Arial", "DejaVu Sans"]
plt.rcParams["font.weight"] = "bold"

UDP_PORT = 5005
MAX_RANGE_M = 6.0
DEBUG = False            # True prints every packet to the console

# filtering / tracking behaviour
CLUSTER_RADIUS_M = 0.60  # detections within this distance are the same target
MATCH_RADIUS_M   = 0.90  # how far a track may jump between frames
SMOOTH           = 0.35  # 0..1 position smoothing, lower is smoother and laggier
MIN_HITS         = 3     # frames a track must persist before it is shown
MAX_MISSES       = 10    # frames unseen before a track is dropped
TRAIL            = 8     # length of each track's fading tail
SHOW_RAW         = False # True also draws the unfiltered detections, dim

BG = "#001a00"
GREEN = "#00ff41"
DIM_GREEN = "#0a5c1e"
BLIP = (0.0, 1.0, 0.25)
RAW = (0.35, 0.55, 0.35)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.setblocking(False)

# full-circle polar scope
fig = plt.figure(figsize=(8, 8), facecolor="black")
ax = fig.add_subplot(111, projection="polar", facecolor=BG)
ax.set_theta_zero_location("N")   # 0 deg (forward) at top
ax.set_theta_direction(-1)        # clockwise positive, to the right
# set_ylim, not set_rmax: with an initially empty scatter, autoscale otherwise
# collapses the radial axis to a few cm and every blip falls outside the view.
ax.set_ylim(0, MAX_RANGE_M)
ax.set_rticks([1, 2, 3, 4, 5, 6])
ax.set_rlabel_position(15)
ax.set_thetagrids(range(0, 360, 30))

ax.grid(color=DIM_GREEN, alpha=0.6, linewidth=0.8)
ax.spines["polar"].set_color(GREEN)
ax.tick_params(colors=GREEN)
for lbl in ax.get_xticklabels() + ax.get_yticklabels():
    lbl.set_color(GREEN)

ax.set_title("CAGE - merged 360 scope (range in metres)", pad=22, color=GREEN)
ax.text(0.5, 1.005, "FWD", transform=ax.transAxes, ha="center", va="bottom",
        fontsize=10, color=GREEN, alpha=0.8)
ax.plot(0, 0, marker="^", markersize=12, color=GREEN, zorder=5)

raw_scatter = ax.scatter([], [], s=25, zorder=3)
scatter = ax.scatter([], [], s=110, zorder=4)
status = ax.text(0.02, 0.02, "", transform=ax.transAxes, color=GREEN,
                 fontsize=9, alpha=0.8)

# latest[sensor_id] = list of (X, Y) in metres, airframe frame.
# X = right, Y = forward.
latest = {}


class Track:
    """A smoothed, persistent target."""
    __slots__ = ("x", "y", "hits", "misses", "tail")

    def __init__(self, x, y):
        self.x, self.y = x, y
        self.hits = 1
        self.misses = 0
        self.tail = deque(maxlen=TRAIL)
        self.tail.append((x, y))

    def update(self, x, y):
        self.x += SMOOTH * (x - self.x)
        self.y += SMOOTH * (y - self.y)
        self.hits += 1
        self.misses = 0
        self.tail.append((self.x, self.y))


tracks = []


def poll_udp():
    """Read all waiting packets, keep each sensor's most recent frame."""
    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except BlockingIOError:
            break
        msg = data.decode(errors="ignore").strip()
        if DEBUG:
            print("RX:", repr(msg))

        head, sep, body = msg.partition(":")
        if not sep:
            continue
        head_parts = head.split(",")
        if len(head_parts) != 2:
            continue
        sensor_id = head_parts[0]
        try:
            mount_deg = float(head_parts[1])
        except ValueError:
            continue

        pts = []
        if body and body != "-":
            for chunk in body.split(";"):
                parts = chunk.split(",")
                if len(parts) != 4:
                    continue
                try:
                    x, y, dist, speed = (int(p) for p in parts)
                except ValueError:
                    continue
                # rotate into the shared airframe frame
                theta = math.radians(mount_deg) + math.atan2(x, y)
                r = dist / 1000.0
                if r <= 0 or r > MAX_RANGE_M:
                    continue
                pts.append((r * math.sin(theta), r * math.cos(theta)))
        latest[sensor_id] = pts


def cluster(points):
    """Greedily merge points within CLUSTER_RADIUS_M into single centroids."""
    groups = []
    for x, y in points:
        for g in groups:
            if math.hypot(x - g["x"], y - g["y"]) <= CLUSTER_RADIUS_M:
                g["pts"].append((x, y))
                n = len(g["pts"])
                g["x"] = sum(p[0] for p in g["pts"]) / n
                g["y"] = sum(p[1] for p in g["pts"]) / n
                break
        else:
            groups.append({"x": x, "y": y, "pts": [(x, y)]})
    return [(g["x"], g["y"]) for g in groups]


def step_tracks(centroids):
    """Match centroids to existing tracks, age out the unmatched."""
    unmatched = list(centroids)
    for t in tracks:
        best, best_d = None, MATCH_RADIUS_M
        for c in unmatched:
            d = math.hypot(c[0] - t.x, c[1] - t.y)
            if d < best_d:
                best, best_d = c, d
        if best is not None:
            t.update(*best)
            unmatched.remove(best)
        else:
            t.misses += 1
    for c in unmatched:
        tracks.append(Track(*c))
    tracks[:] = [t for t in tracks if t.misses <= MAX_MISSES]


def update(_frame):
    poll_udp()

    detections = [p for pts in latest.values() for p in pts]
    step_tracks(cluster(detections))

    offsets, colors = [], []
    shown = 0
    for t in tracks:
        if t.hits < MIN_HITS:
            continue        # not yet confirmed, suppresses flicker and ghosts
        shown += 1
        n = len(t.tail)
        for age, (x, y) in enumerate(t.tail):
            alpha = 0.15 + 0.85 * ((age + 1) / n)
            offsets.append([math.atan2(x, y), math.hypot(x, y)])
            colors.append((*BLIP, alpha))

    if offsets:
        scatter.set_offsets(np.array(offsets))
        scatter.set_facecolors(np.array(colors))
    else:
        scatter.set_offsets(np.empty((0, 2)))

    if SHOW_RAW and detections:
        raw_scatter.set_offsets(
            np.array([[math.atan2(x, y), math.hypot(x, y)] for x, y in detections]))
        raw_scatter.set_facecolors(np.array([(*RAW, 0.5)] * len(detections)))
    else:
        raw_scatter.set_offsets(np.empty((0, 2)))

    status.set_text(f"targets: {shown}    raw returns: {len(detections)}")
    return scatter, raw_scatter, status


ani = animation.FuncAnimation(fig, update, interval=50, blit=False,
                              cache_frame_data=False)
print(f"Listening on UDP :{UDP_PORT} ... (close the plot window to stop)")
plt.show()
