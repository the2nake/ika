import sys
import serial
import signal
import matplotlib.pyplot as plt
import matplotlib.animation as animate
import math
import time


def interrupt_handler(signal, frame):
    print(" [interrupt] ")
    sys.exit(0)


signal.signal(signal.SIGINT, interrupt_handler)


ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
print("using port", ser.name, "@", ser.baudrate)

yaw0 = 0
yawPI = 0
pitch0 = 0
pitchPI_2 = 0

past_axes = []
past_values = [[], [], []]

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
    global yaw0, yawPI, pitch0, pitchPI_2

    yaw0 = calibrate(ser, "BASE", "rotate to yaw= 0")
    yawPI = calibrate(ser, "BASE", "rotate to yaw= PI")
    pitch0 = calibrate(ser, "LOWER", "rotate to pitch= 0")
    pitchPI_2 = calibrate(ser, "LOWER", "rotate to pitch= PI/2")

    print("  yaw (0->  pi):", yaw0, yawPI)
    print("pitch (0->pi/2):", pitch0, pitchPI_2)


def live_show(i, ser):
    global yaw0, yawPI, pitch0, pitchPI_2

    base = (0, 0, 0)
    lower_length = 10
    yaw = 0
    lower_pitch = 0

    read = [False, False]

    while not read[0] or not read[1]:
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
                (raw - pitch0) / (pitchPI_2 - pitch0)
            read[1] = True

    ser.reset_input_buffer()

    distance = lower_length * math.cos(lower_pitch)
    elbow = (base[0] + distance * math.cos(yaw), base[1] + distance *
             math.sin(yaw), base[2] + lower_length * math.sin(lower_pitch))

    if i % 10 == 0:
        print(f"(t0, t1): {math.degrees(lower_pitch)} {math.degrees(yaw)}")

    xs = [base[0], elbow[0]]
    ys = [base[1], elbow[1]]
    zs = [base[2], elbow[2]]

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

    ax3d.set_xlim3d(-20, 20)
    ax3d.set_ylim3d(-20, 20)
    ax3d.set_zlim3d(0, 30)


setup_stops(ser)

ani = animate.FuncAnimation(fig, live_show, fargs=(
    ser,), interval=10, cache_frame_data=False)
plt.show()
