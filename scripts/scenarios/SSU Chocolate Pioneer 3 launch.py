import math
import time
import krpc
from utils.launch_into_orbit import launch_into_orbit

TARGET_ALTITUDE = 120000
TARGET_INCLINATION = 0

def main():
    conn = krpc.connect(name='Launch into orbit')

    launch_into_orbit(conn, TARGET_ALTITUDE, TARGET_INCLINATION, auto_stage=False)

if __name__ == "__main__":
    main()
