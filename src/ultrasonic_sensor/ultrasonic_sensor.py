import serial
import time
import struct

SERIAL_PORT = '/dev/ttyTHS2'
BAUD_RATE = 115200

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) as ser:
            print("Sending trigger to sensor...")

            while True:
                # Step 1: Send trigger 'a'
                ser.write(b'a')
                print("Triggered measurement")

                # Step 2: Wait for sensor response
                time.sleep(0.02)  # 20ms delay

                # Step 3: Try reading 4 bytes
                response = ser.read(4)
                if len(response) == 4:
                    header, high, low, checksum = struct.unpack('BBBB', response)

                    if header == 0xFF:
                        calculated_sum = header + high + low
                        if calculated_sum & 0xFF == checksum:
                            distance = (high << 8) | low
                            print(f"{distance} mm")
                        else:
                            print("Checksum mismatch:", response)
                    else:
                        print("Invalid header:", response)
                else:
                    print("Incomplete packet or no response")

                # Step 4: Wait 500 ms
                time.sleep(0.5)

    except serial.SerialException as e:
        print("Serial error:", e)

if __name__ == '__main__':
    main()
