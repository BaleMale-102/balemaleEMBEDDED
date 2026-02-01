# MotorCommand.msg
# 4개 메카넘 휠 PWM 명령

std_msgs/Header header

int32 front_left                        # PWM 값 (300~2000, 1150=정지)
int32 front_right
int32 rear_left
int32 rear_right

bool enabled                            # 모터 활성화 여부
