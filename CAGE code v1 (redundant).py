import socket
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

UDP_PORT = 5005
MAX_RANGE_M = 6.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0",UDP_PORT))
sock.setblocking(False)

fig = plt.figure(figsize=(7,7))
ax = fig.add_subplot(111, projection = "polar")
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_thetamin(-90)
ax.set_thetamax(90)
ax.set_rmax(MAX_RANGE_M)
ax.set_rlabel_position(90)
ax.set_title("CAGE radar - sensor view (m)", pad=20)

scatter = ax.scatter([], [], s=120, c="red", edgecolors="black", zorder=3)

latest = {"targets": []}

def poll_udp():
    """Drain all waiting packets, keep only the newest frame."""
    newest = None
    while True:
        try:
            data, _ = sock.recvfrom(1024)
        except BlockingIOError:
            break
        newest = data.decode(errors="ignore").strip()
    if newest is None:
        return 

    targets = []
    if newest and newest != "-":
        for chunk in newest.split(";"):
            parts = chunk.split(",")
            if len(parts) != 4:
                continue
            try:
                x, y, dist, speed = (int(p) for p in parts)
            except ValueError:
                continue
            angle = math.atan2(x, y)        
            r = dist / 1000.0               
            targets.append((angle, r, speed))
    latest["targets"] = targets


def update(_frame):
    poll_udp()
    targets = latest["targets"]
    if targets:
        thetas = [t[0] for t in targets]
        rs = [t[1] for t in targets]
        scatter.set_offsets([[th, r] for th, r in zip(thetas, rs)])
    else:
        scatter.set_offsets([[0, 0]][:0])  # empty
    return (scatter,)


ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
print(f"Listening on UDP :{UDP_PORT} ... (close the plot window to stop)")
plt.show()