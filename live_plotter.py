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

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')

yaw0 = 0
yawPI = 0
pitch0 = 0
pitchPI_2 = 0


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
            yaw = math.pi * (int(data[4:]) - yaw0) / (yawPI - yaw0)
            read[0] = True

        if data.startswith("LOWER"):
            lower_pitch = 0.5 * math.pi * \
                (int(data[5:]) - pitch0) / (pitchPI_2 - pitch0)
            read[1] = True

    ser.read_all()

    distance = lower_length * math.cos(lower_pitch)
    elbow = (base[0] + distance * math.cos(yaw), base[1] + distance *
             math.sin(yaw), base[2] + lower_length * math.sin(lower_pitch))
    
    print("pitch", math.degrees(lower_pitch))
    print("yaw", math.degrees(yaw))
    
    xs = [base[0], elbow[0]]
    ys = [base[1], elbow[1]]
    zs = [base[2], elbow[2]]

    ax.clear()
    ax.plot3D(xs, ys, zs)

    ax.set_xlim3d(-20, 20)
    ax.set_ylim3d(-20, 20)
    ax.set_zlim3d(0, 30)


setup_stops(ser)

ani = animate.FuncAnimation(fig, live_show, fargs=(
    ser,), interval=10, cache_frame_data=False)
plt.show()
