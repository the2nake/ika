import serial
import signal
import sys
import time
import ik


def interrupt_handler(signal, frame):
    print(" [interrupt] ")
    sys.exit(0)


def lerp(a: list[int], b: list[int], t: float):
    return [i + (j - i) * t for i, j in zip(a, b)]


if __name__ == "__main__":
    signal.signal(signal.SIGINT, interrupt_handler)

    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    print("using port", ser.name, "@", ser.baudrate)

    j1, j2, j3, j4, j5 = 90, 90, 180, 0, 0

    seq_i = 0
    seq = [
        [-100, 100, 50],
        [-100, 100, 0],
        [-100, 100, 50],
        [50, 100, 50],
        [50, 100, 00],
        [50, 100, 50],
        [200, 100, 50],
        [200, 100, 0],
        [200, 100, 50]
    ]
    dt = 1
    last = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now - last >= dt:
            seq_i = (seq_i + 1) % len(seq)
            last = time.perf_counter()
        a = seq[seq_i]
        b = seq[(seq_i + 1) % len(seq)]
        j1, j2, j3, j4, j5 = ik.get_joints_to(*lerp(a, b, (now - last) / dt))
        ser.write(f"{j1},{j2},{j3},{j4},{j5}%".encode())
        print(f"[sent]{j1},{j2},{j3},{j4},{j5}%")
        # TODO: use writemicroseconds, see if you can get it to go 750 and 2250
        # TODO: modulate the microseconds values
        ser.flush()
        time.sleep(1/100)
        str = ser.read_all()
        if str:
            print(" [rec]", str.decode())
