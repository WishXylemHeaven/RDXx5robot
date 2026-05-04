
#!/usr/bin/env python3

import serial

import time



PORT = "/dev/ttyS1"

BAUD = 115200



CMDS = {

    "version_0xf1": bytes.fromhex("5a 06 01 f1 00 d7"),

    "sn_0xf3":      bytes.fromhex("5a 06 01 f3 00 46"),

    "info_0x21":    bytes.fromhex("5a 06 01 21 00 8f"),

    "battery_0x07": bytes.fromhex("5a 06 01 07 00 e4"),

    "odom_0x11":    bytes.fromhex("5a 06 01 11 00 a2"),

    "imu_0x13":     bytes.fromhex("5a 06 01 13 00 33"),

}



def read_some(ser, duration=0.4):

    end = time.time() + duration

    data = bytearray()

    while time.time() < end:

        n = ser.in_waiting

        if n:

            data += ser.read(n)

        time.sleep(0.01)

    return bytes(data)



def main():

    print(f"Open {PORT} @ {BAUD}")

    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:

        ser.reset_input_buffer()

        ser.reset_output_buffer()



        for name, cmd in CMDS.items():

            print(f"\nTX {name}: {cmd.hex(' ')}")

            ser.write(cmd)

            ser.flush()

            rx = read_some(ser, 0.5)

            if rx:

                print(f"RX: {rx.hex(' ')}")

            else:

                print("RX: <no data>")



    print("\nDone.")



if __name__ == "__main__":

    main()

