# 통신 프로토콜

## Arduino 모터 프로토콜 (Arduino_MoebiusTech_v2.ino)

### 시리얼 설정
- Port: `/dev/ttyUSB0`
- Baudrate: 115200
- Watchdog: 200ms

### 명령어
| 명령 | 형식 | 설명 |
|------|------|------|
| V | `V vx vy wz` | 속도 명령 (m/s, rad/s) |
| D | `D vx vy wz` | 정규화 명령 (-1~1) |
| S/Z | `S` | 정지 |
| P | `P value` | MAX_PWM 설정 (300~4000) |
| C | `C idx scale` | 모터 보정 |

### 속도 변환
```
MAX_VEL_LINEAR = 0.05 m/s → normalized 1.0
MAX_VEL_ANGULAR = 0.5 rad/s → normalized 1.0
```

### 메카넘 휠 믹싱
```
FL = vx + vy + wz
FR = vx - vy - wz
RL = vx - vy + wz
RR = vx + vy - wz
```

---

## Loader Arduino 프로토콜 (carrying.ino)

### 시리얼 설정
- Port: `/dev/ttyACM0`
- Baudrate: 9600

### 하드웨어
- DC 모터: ENA(6), IN1(5), IN2(4)
- 서보: 핀 11 (0°=언로드, 90°=중립, 180°=로드)
- 리밋 스위치: FWD(9), REV(8)

### 명령어
| 명령 | 설명 |
|------|------|
| `1` | Load - 전진→리밋→서보180°→후진→리밋 |
| `2` | Unload - 전진→리밋→서보0°→후진→리밋 |

### 통신 흐름 (serial_bridge 모드)
```
mission_manager → /loader/command
       ↓
loader_driver → /motor_cmd ("1" or "2")
       ↓
serial_bridge.py → Arduino
       ↓
Arduino → "LOAD_COMPLETE"
       ↓
serial_bridge.py → /loading_status ("completed")
       ↓
loader_driver → /loader/status (DONE)
```

---

## MQTT 프로토콜 (balemale backend)

### 토픽 구조
```
balemale/robot/{robotId}/cmd          # Subscribe: 서버 명령
balemale/robot/{robotId}/map          # Subscribe: 맵 정보
balemale/robot/{robotId}/request/dispatch  # Publish: 배차 요청
balemale/robot/{robotId}/event        # Publish: 상태 이벤트
balemale/robot/{robotId}/heartbeat    # Publish: 하트비트
```

### 배차 요청
```json
{"reqId": "...", "plate": "12가3456", "nowNodeId": 0, "ts": "..."}
```

### 서버 명령 (type 1: 배차)
```json
{
  "type": "1",
  "reqId": "...",
  "cmdId": "...",
  "payload": {"vehicleId": 1, "targetNodeId": 17, "path": [0,1,5,17]}
}
```

### eventType
| 값 | 설명 |
|----|------|
| WAITING | 대기 중 |
| LOADING | 적재/하역 중 |
| DRIVING_DESTINATION | 목적지 이동 |
| DRIVING_HOME | 홈 복귀 |
| PARKING | 주차 중 |
| ESTOP | 오류/정지 |
