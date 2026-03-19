from angle_tune import setup, get_raws

import serial
import math

import numpy as np


def raw_to_rad(raw: int, raw_ends: tuple[int, int], rad_ends: tuple[float, float]):
    innov = (raw - raw_ends[0]) / (raw_ends[1] - raw_ends[0])
    return rad_ends[0] + innov * (rad_ends[1] - rad_ends[0])


def get_eq(raw_ends: tuple[int, int], rad_ends: tuple[float, float]):
    """returns `k`, `b` such that `rad = k * raw + b`"""
    k = (rad_ends[1] - rad_ends[0]) / (raw_ends[1] - raw_ends[0])
    b = rad_ends[0] - k * raw_ends[0]
    return k, b


def calibrate_angles(ser: serial.Serial, silent=False):
    stops = (5944, 594, 6143, 4736, 3703, 2355)
    rads = (0, math.pi, math.pi/4, math.pi/2, math.pi/4, math.pi/2)

    calibrate_str = "y" if silent else input("run angle calibration? (y/N): ")
    if calibrate_str.startswith("y"):
        stops = setup(ser, rads)

    k_azim, b_azim = get_eq(stops[0:2], rads[0:2])
    k_elev, b_elev = get_eq(stops[2:4], rads[2:4])
    k_elbow, b_elbow = get_eq(stops[4:6], rads[4:6])

    if not silent:
        print(f"raw stops: {stops}")
        print(f"eqs:  base: ({k_azim:.5e}) * raw + ({b_azim:.5e})")
        print(f"      elev: ({k_elev:.5e}) * raw + ({b_elev:.5e})")
        print(f"     elbow: ({k_elbow:.5e}) * raw + ({b_elbow:.5e})")

    return k_azim, b_azim, k_elev, b_elev, k_elbow, b_elbow


def compute_vec3(yaw: float, pitch: float):
    """computes normalised 3d vector specified by angles in radians"""
    return (math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw),
            math.sin(pitch))


tup_float_6 = tuple[float, float, float, float, float, float]


def calibrate_lengths(ser: serial.Serial, coeffs: tup_float_6,
                      points: list[tuple[float, float, float]], silent=False):
    elbow_offset = math.radians(183)

    min_points = 2
    if len(points) < min_points:
        print(f"calibration impossible, at least {min_points} points needed")
        return None

    A = np.zeros((3 * len(points), 5))
    b = np.zeros((3 * len(points), 1))

    for i, point in enumerate(points):
        input(f"move the tip to position {point}")
        azim, elev, elbow = (coeffs[2*j] * raw + coeffs[2*j+1]
                             for j, raw in enumerate(get_raws(ser)))
        nx1, ny1, nz1 = compute_vec3(azim, elev)
        nx2, ny2, nz2 = compute_vec3(azim, elev + elbow + elbow_offset)

        A[3*i] = [nx1, nx2, 1, 0, 0]
        A[3*i+1] = [ny1, ny2, 0, 1, 0]
        A[3*i+2] = [nz1, nz2, 0, 0, 1]
        b[3*i] = point[0]
        b[3*i+1] = point[1]
        b[3*i+2] = point[2]

    res = np.linalg.lstsq(A, b)[0].transpose()[0]

    # ! problem: base needs to be aligned with world axes

    if not silent:
        # ref = A @ np.array([[367.0], [340.0], [0.0], [-170.0], [20.0]]) 
        # calib = A @ res
        # print("  reference", ref)
        # print("calibration", calib)
        # print(np.concat((ref, calib), axis=1))
        print("joint lengths", res[:2])
        print("base position", res[2:])
    return res


if __name__ == "__main__":
    calibration_points = [(0.0, 0.0, 0.0),
                          (0.0, 100.0, 0.0),
                          (100.0, 100.0, 0.0),
                          (100.0, 0.0, 0.0)]

    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    print("using port", ser.name, "@", ser.baudrate)

    coeffs = calibrate_angles(ser)
    geom = calibrate_lengths(ser, coeffs, calibration_points)
