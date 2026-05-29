# untouchable-areas — 하드웨어 상수 동결

명시적 지시가 없는 한 아래 영역은 **변경 금지**한다.

- `include/AutoDrive/Modules/Basic.h`의 `GPIO_PIN` 핀맵 값
- `include/AutoDrive/Modules/RobotHat.h`의 I2C 레지스터 오프셋(`*_REG_OFFSET`) 및 `CAR_I2C_CHANNEL` 채널 번호
- `AutoDrive.ini` `[calibration]` 상수(`steer_angle_offset`, `camera_pitch_angle_offset`, `camera_yaw_angle_offset`)
- `AutoDrive.ini`의 `[protocal]` 섹션명(오타지만 의도적 — 외부 클라이언트 호환)

**Why:** 이 값들은 실제 PiCar-X 하드웨어 배선·I2C 칩 사양·외부 클라이언트 프로토콜에 1:1로 묶여 있다. 코드만 보고 "정리"하면 빌드는 통과해도 하드웨어가 오동작하거나 원격 클라이언트와 호환이 깨진다. 자동 검증 수단(테스트/CI)이 없어 회귀를 잡을 안전망도 없다.

**How to apply:** 위 상수를 바꿔야 할 것 같으면 멈추고 사용자에게 확인한다(AskUserQuestion). 기능 변경은 이 상수를 건드리지 않고 상위 로직에서 처리하는 길을 우선 찾는다. 관련 사실: [[project-intent]].
