# AutoDrive

Raspberry Pi 기반의 소형 자율주행 실험 플랫폼입니다. PiCar‑X 하드웨어를 제어하고, ZeroMQ Pub/Sub로 상태 스트림과 제어 명령을 네트워크에 노출합니다. 듀얼 카메라(libcamera) 프레임을 OpenCV로 처리/전송하며, LD06 라이다, 초음파, 바닥선 센서, SSD1306 LCD를 지원합니다.

## Architecture
- Orchestrator: `PiCar`가 하드웨어 모듈과 ZeroMQ XPUB/XSUB 프록시를 구성합니다.
- IPC/TCP 브리지: `Protocol::PubSubServer`가 `inproc://SERVER_XPUB/XSUB`와 TCP (`AutoDrive.ini`)를 바인딩합니다.
- 모듈 스레드: 각 모듈은 `PubSubClient`를 소유하고 업데이트 스레드 + 퍼블리셔 스레드(+ 구독 스레드)를 실행합니다.
- 실행 모드(인자): `1=Direct`, `2=Remote`, `3=Camera`.
  - Direct: 로컬 카메라 UI 표시 + 키보드 입력 처리.
  - Remote/Camera: pub/sub 루프만 실행(화면/키보드 없음).

주요 모듈(헤더는 `include/AutoDrive/Modules/*.h`):
- `MoveMotor`(후륜 PWM, 조향 서보), `CameraMotor`(피치/요 서보), `Sensors`(HC‑SR04 + 3x ADC 바닥선), `LcdDisplay`(I2C SSD1306), `LidarSensor`(LD06 `/dev/ttyUSB0`), `CameraSensor`(듀얼 카메라 → OpenCV Mat, JPEG 퍼블리시).

## Build
필수: CMake ≥ 3.16, C++20, Raspberry Pi 환경에서 `wiringPi`, `libcamera`, OpenCV, ZeroMQ 개발 패키지.

```bash
cmake -S . -B build
cmake --build build --parallel
```

VS Code 작업으로도 가능: Task "CMake Build".

## Run
빌드 결과는 `install/AutoDrive`로 배치됩니다.

```bash
./install/AutoDrive 1   # Direct 모드 (카메라 UI + 키 입력)
./install/AutoDrive 2   # Remote 모드 (네트워크만)
./install/AutoDrive 3   # Camera 모드 (카메라/센서 퍼블리시)
```

하드웨어 권한/연결: GPIO/I2C/시리얼 사용이 가능해야 하며, LD06은 `/dev/ttyUSB0`(230400 baud)로 연결됩니다.

## Configuration
실행 디렉터리의 `./AutoDrive.ini`를 로드합니다. 템플릿은 `doc/AutoDrive.ini`, 예시는 `install/AutoDrive.ini`.

섹션과 키:
- `[protocal]`(오타 그대로 사용): `publish_ip` 기본 `tcp://*:45000`, `subscribe_ip` 기본 `tcp://*:45001`.
- `[calibration]`: `steer_angle_offset`, `camera_pitch_angle_offset`, `camera_yaw_angle_offset`(deg). 서보 매핑에 오프셋 적용.
- `[camera]`: `width`, `height`, `buffer_length`, `frame_rate`, `is_record`(GStreamer로 파일 기록).

## Messaging (ZeroMQ multipart)
퍼블리셔는 `ChangePubTopic("STATE_*")`, 커맨드 수신자는 `AddSubTopic("COMMAND_*")`를 사용합니다. 공통 규약: [topic, command, type, payloads...]. 자세한 구현은 `src/Modules/Hardware.cpp` 참고.

- MoveMotor 명령(`COMMAND_MOVE_MOTOR`):
  - `(topic, "REAR_MOTOR", "VALUE", int)` PWM 설정
  - `(topic, "REAR_MOTOR", "SPEED", int)` 가속 램프(단위/초)
  - `(topic, "REAR_MOTOR", "STOP")` 즉시 정지
  - `(topic, "STEER_MOTOR", "VALUE", float)` 조향 각도 설정(도)
  - `(topic, "STEER_MOTOR", "SPEED", float)` 조향 속도(도/초)
  - 상태 `STATE_MOVE_MOTOR`: `curRear, tarRear, dRear, curSteer, tarSteer, dSteer`

- CameraMotor 명령(`COMMAND_CAMERA_MOTOR`):
  - `(topic, "PITCH_MOTOR"|"YAW_MOTOR", "VALUE"|"SPEED", float)`
  - 상태 `STATE_CAMERA_MOTOR`: `curPitch, tarPitch, dPitch, curYaw, tarYaw, dYaw`
  - 주: 현재 `PiCar::initRobotHat()`에서 초기화가 비활성화(주석)되어 있을 수 있습니다.

- 센서 상태 `STATE_SENSOR`: `sonic(mm: double), floorLeft(int), floorCenter(int), floorRight(int)`
- 라이다 상태 `STATE_LIDAR_SENSOR`: `count(int)` 이후 반복 프레임 `degree(float), distance(mm: float), intensity(float)`
- 카메라 상태 `STATE_CAMERA_SENSOR`: 좌/우 JPEG 버퍼 2개(바이너리 프레임)
- PiCar 하트비트 `STATE_PICAR`: `"RUNNING"|"STOP"`, 명령 `COMMAND_PICAR`: `"TURN_OFF"`, `"UPDATE_CONNECTION"`

## Motion Smoothing & Timing
모듈 스레드는 고정 주기 틱으로 동작합니다: 모션 `16ms`, 라이다 `33ms`, LCD `1000ms`, 카메라 `1000/fps` ms. 목표값+델타(속도) 모델로 각 틱마다 타깃으로 점진 수렴합니다(`updateThreadFunc()`). 스레드 이름은 `pthread_setname_np`로 지정됩니다.

## Direct 모드 키 입력
`PiCar::executeKeyInput()` 기준(대소문자 구분 없음):
- 종료: `Q`
- 후륜: `W`(+100), `S`(−100), `X`(0으로 설정), `Z`(즉시 정지)
- 조향: `A`(−5°), `D`(+5°), `F`(0°)
- 카메라 피치: `O`(+10°), `L`(−10°), `.` 또는 `>`(0°)
- 카메라 요: `K`(−10°), `;` 또는 `:`(+10°), `J`(0°)

참고: 카메라 서보가 비활성화된 구성에서는 피치/요 키가 효과가 없을 수 있습니다.

## Dependencies
`CMakeLists.txt` 링크 대상 기준:
- ZeroMQ(`zmq` + `cppzmq`), WiringPi(`wiringPi`, `wiringPiI2C`, `wiringSerial`)
- libcamera(`camera`, `camera-base`), OpenCV(`core`, `imgproc`, `highgui`, `imgcodecs`, `videoio`), pthread, m, atomic
- 녹화: OpenCV VideoWriter의 GStreamer 파이프라인 사용(x264)

## Key Files
- 엔트리: `src/AutoDrive.cpp`, 오케스트레이터: `src/PiCar.cpp`
- 메시징: `include/AutoDrive/Modules/Protocol.h`, `src/Modules/Protocol.cpp`
- 하드웨어: `src/Modules/{Hardware,Basic,RobotHat,LD06,EP0152,Camera}.cpp`

## Notes
- 카메라 전송은 JPEG로 퍼블리시합니다(네트워크 대역폭 최적화). Raw/PNG 경로는 헬퍼가 있으나 기본 퍼블리셔는 JPEG입니다.
- 원격 클라이언트는 TCP 소켓(`publish_ip`/`subscribe_ip`)로 상태 구독과 명령 발행이 가능합니다.


