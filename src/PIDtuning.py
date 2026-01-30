import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Arduino 리셋 대기

# 상태 확인
ser.write(b'?\n')
time.sleep(0.1)
print(ser.read(200).decode())

# 모터 테스트
ser.write(b'D 0.5 0 0\n')
time.sleep(0.1)
print(ser.read(100).decode())

# 3초 후 정지
time.sleep(3)
ser.write(b'S\n')
print(ser.read(100).decode())

ser.close()