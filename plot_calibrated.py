from auto_calibrate import calibrate_angles

from typing import cast
import serial
import math

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt



if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    print("using port", ser.name, "@", ser.baudrate)

    coeffs = calibrate_angles(ser)

    base = (0, 0, 63)
    lower_length = 367
    upper_length = 363 + 20

    fig, axs = plt.subplot_mosaic([['3d']])
    axs['3d'] = cast(Axes3D, plt.subplot(projection='3d'))
    
    plt.show()
