import serial
import time
import math


def read_value(string: str, keyword: str):
    start = string.rfind(keyword) + len(keyword)
    return int(string[start:].split(None, 1)[0])


def calibrate(ser: serial.Serial, prefix: str, rad: float, sample_ms=50) -> int:
    input(f"rotate to {prefix} = {round(math.degrees(rad))} deg")
    ser.reset_input_buffer()
    time.sleep(0.001 * sample_ms)
    str_in = ser.read_all()
    if str_in is None:
        return 0
    return read_value(str_in.decode(), prefix)


tup_int_6 = tuple[int, int, int, int, int, int]
tup_float_6 = tuple[float, float, float, float, float, float]


def setup(ser: serial.Serial, rads: tup_float_6) -> tup_int_6:
    azim0 = calibrate(ser, "BASE",  rads[0])
    azim1 = calibrate(ser, "BASE",  rads[1])
    elev0 = calibrate(ser, "LOWER", rads[2])
    elev1 = calibrate(ser, "LOWER", rads[3])
    elbow0 = calibrate(ser, "UPPER", rads[4])
    elbow1 = calibrate(ser, "UPPER", rads[5])
    return azim0, azim1, elev0, elev1, elbow0, elbow1


def get_raws(ser: serial.Serial, sample_ms=50) -> tuple[int, int, int]:
    ser.reset_input_buffer()
    time.sleep(0.001 * sample_ms)
    str_in = ser.read_all()
    if str_in is None:
        return (0, 0, 0)

    str_in = str_in.decode()
    azim = read_value(str_in, "BASE")
    elev = read_value(str_in, "LOWER")
    elbow = read_value(str_in, "UPPER")

    return azim, elev, elbow
