#!/usr/bin/env python3
"""엔코더 테스트 - 단순 버전"""

import serial
import time

PORT = '/dev/ttyACM0'
BAUD = 115200

def main():
    print(f"Connecting to {PORT}...")

    ser = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(2)

    # 부트 메시지 읽기
    while ser.in_waiting:
        print("Boot:", ser.readline().decode().strip())

    print("\nType 'E' and press Enter (Q to quit):\n")

    try:
        while True:
            cmd = input("> ").strip()
            if cmd.upper() == 'Q':
                break

            # 한 글자씩 보내기
            for c in cmd:
                ser.write(c.encode())
                time.sleep(0.01)

            time.sleep(0.2)

            # 응답 읽기
            response = b''
            while ser.in_waiting:
                response += ser.read(ser.in_waiting)
                time.sleep(0.05)

            if response:
                print(f"Response: {response.decode().strip()}")
            else:
                print("(no response)")

    except KeyboardInterrupt:
        pass

    ser.close()

if __name__ == '__main__':
    main()
