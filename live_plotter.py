import sys
import serial
import signal
import matplotlib.pyplot as plt
import matplotlib.animation as animate
import math
import time


base = (0, 0, 63)
lower_length = 367
upper_length = 363


def interrupt_handler(signal, frame):
    print(" [interrupt] ")
    sys.exit(0)


signal.signal(signal.SIGINT, interrupt_handler)


ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
print("using port", ser.name, "@", ser.baudrate)

yaw0 = 0
yawPI = 0
pitch0_0 = 0
pitch0_PI_2 = 0
pitch1_0 = 0
pitch1_PI_2 = 0

past_axes = []
past_values = [[], [], []]

traces = [{"x": [], "y": [], "z": []}]

num_combined = 6

fig = plt.figure()
ax3d = fig.add_subplot(num_combined + len(past_values),
                       1, (1, num_combined), projection='3d')

for i in range(len(past_values)):
    past_axes.append(fig.add_subplot(
        num_combined + len(past_values), 1, num_combined+1+i))


def calibrate(ser, start, prompt):
    input(prompt)
    ser.reset_input_buffer()
    time.sleep(0.1)
    str_in = ser.read_all().decode()
    return int(str_in.split(start)[-1].split()[0])


def setup_stops(ser, delay=0):
    global yaw0, yawPI, pitch0_0, pitch0_PI_2, pitch1_0, pitch1_PI_2

    yaw0 = calibrate(ser, "BASE", "rotate to yaw= 0")
    yawPI = calibrate(ser, "BASE", "rotate to yaw= PI")
    pitch0_0 = calibrate(ser, "LOWER", "rotate to pitch0= 0")
    pitch0_PI_2 = calibrate(ser, "LOWER", "rotate to pitch0= PI/2")
    pitch1_0 = calibrate(ser, "UPPER", "rotate to pitch1= 0")
    pitch1_PI_2 = calibrate(ser, "UPPER", "rotate to pitch1= PI/2")

    print("   yaw (0->  pi):", yaw0, yawPI)
    print("pitch0 (0->pi/2):", pitch0_0, pitch0_PI_2)
    print("pitch1 (0->pi/2):", pitch1_0, pitch1_PI_2)


def extend_from_point(point, length, yaw, pitch):
    distance = length * math.cos(pitch)
    return [point[0] + distance * math.cos(yaw),
            point[1] + distance * math.sin(yaw),
            point[2] + length * math.sin(pitch)]


def live_show(i, ser):
    global yaw0, yawPI, pitch0_0, pitch0_PI_2, pitch1_0, pitch1_PI_2
    global traces

    yaw = 0
    lower_pitch = 0
    upper_pitch = 0

    read = [False, False, False]

    while not read[0] or not read[1] or not read[2]:
        data = ser.readline().decode().strip()
        if not data:
            break
        # print("[rec]", data)

        if data.startswith("BASE"):
            raw = int(data[4:])
            past_values[0].append(raw)
            yaw = math.pi * (raw - yaw0) / (yawPI - yaw0)
            read[0] = True

        if data.startswith("LOWER"):
            raw = int(data[5:])
            past_values[1].append(raw)
            lower_pitch = 0.5 * math.pi * \
                (raw - pitch0_0) / (pitch0_PI_2 - pitch0_0)
            read[1] = True

        if data.startswith("UPPER"):
            raw = int(data[5:])
            past_values[2].append(raw)
            upper_pitch = 0.5 * math.pi * \
                (raw - pitch1_0) / (pitch1_PI_2 - pitch1_0)
            read[2] = True

    ser.reset_input_buffer()

    elbow = extend_from_point(base, lower_length, yaw, lower_pitch)
    tip = extend_from_point(elbow, upper_length, yaw,
                            upper_pitch + lower_pitch + math.radians(183))

    if i % 10 == 0:
        print(f"{tip[0]:.2f} {tip[1]:.2f} {tip[2]:.2f}")

    if tip[2] > 20:
        if len(traces[-1]["x"]) > 0:
            traces.append({"x": [], "y": [], "z": []})
    else:
        traces[-1]["x"].append(tip[0])
        traces[-1]["y"].append(tip[1])
        traces[-1]["z"].append(0)

    xs = [base[0], elbow[0], tip[0]]
    ys = [base[1], elbow[1], tip[1]]
    zs = [base[2], elbow[2], tip[2]]

    ax3d.clear()
    ax3d.plot3D(xs, ys, zs)

    for i in range(len(past_values)):
        past_values[i] = past_values[i][-50:]
        past_axes[i].clear()
        past_axes[i].plot(past_values[i])
        try:
            past_axes[i].set_ybound(0, 1.1 * max(past_values[i]))
        except:
            pass

    for trace in traces:
        if len(trace) > 0:
            ax3d.plot3D(trace["x"], trace["y"], trace["z"])

    ax3d.set_xlim3d(-500, 500)
    ax3d.set_ylim3d(0, 500)
    ax3d.set_zlim3d(0, 300)


setup_stops(ser)

ani = animate.FuncAnimation(fig, live_show, fargs=(
    ser,), interval=10, cache_frame_data=False)
plt.show()
