# project-intent

updated: 2026-05-29

## 프로젝트 한 줄 정의
라즈베리파이(PiCar-X 하드웨어) 기반 소형 자율주행 실험 플랫폼 — C++20 실행 파일로 하드웨어를 제어하고 ZeroMQ Pub/Sub로 상태/명령을 네트워크에 노출한다.

## 타깃 유형
- 임베디드 단일 실행 파일(서비스성). 라이브러리 아님.
- 실행 모드 인자: `1=Direct`(로컬 카메라 UI + 키 입력), `2=Remote`(네트워크만), `3=Camera`(카메라/센서 퍼블리시).

## 디렉토리 용도
| 경로 | 용도 |
|------|------|
| `src/AutoDrive.cpp` | 엔트리포인트(main, 모드 인자 파싱) |
| `src/PiCar.cpp` · `include/AutoDrive/PiCar.h` | 오케스트레이터 — 하드웨어 모듈 + ZeroMQ XPUB/XSUB 프록시 구성, 모드별 run 루프 |
| `src/Modules/` · `include/AutoDrive/Modules/` | 모듈 구현/헤더 (아래 모듈 표 참조) |
| `doc/AutoDrive.ini` | 설정 템플릿(`{port}` 플레이스홀더) |
| `install/` | 빌드 산출물 배치 위치(`install/AutoDrive`), 런타임 `AutoDrive.ini`, `record_data/`(녹화 mp4) |
| `build/` | CMake/Ninja 빌드 트리(compile_commands.json 포함) |
| `.vscode/` | VS Code 빌드/디버그 태스크 |
| `.claude/` · `.codex/` · `.workflow/` · `temp/` | 멀티에이전트 하네스(프로젝트 코드 아님 — 손대지 않음) |

## 모듈 구조 (계층)
저수준 하드웨어 드라이버 + 고수준 `Hardware` 파사드(스레드 구동) 2계층:

| 모듈 | 계층 | 역할 |
|------|------|------|
| `Basic` | 저수준 | wiringPi GPIO 래퍼 + 핀맵(후륜 방향/라이다/초음파/스위치/LED) |
| `RobotHat` | 저수준 | SunFounder Robot HAT I2C PWM/Servo/ADC 드라이버(CAR_I2C) — 후륜 PWM, 조향/카메라 서보, 바닥선 ADC |
| `EP0152` | 저수준 | I2C OLED LCD(SSD1306 계열, `LCD_I2C`) |
| `LD06` | 저수준 | LD06 라이다(시리얼 `/dev/ttyUSB0`, 230400 baud) |
| `Camera` | 저수준 | libcamera 듀얼 카메라 → OpenCV Mat, JPEG 퍼블리시 |
| `Protocol` | 미들웨어 | ZeroMQ `PubSubServer`/`PubSubClient` (inproc XPUB/XSUB + TCP 브리지) |
| `Hardware` | 고수준 파사드 | `MoveMotor`/`CameraMotor`/`Sensors`/`LcdDisplay`/`LidarSensor`/`CameraSensor` — 저수준 드라이버를 감싸 업데이트/퍼블리시 스레드 구동 |
| `INIParser` | 유틸 | `AutoDrive.ini` 파서 |

## 외부 시스템·의존성
- ZeroMQ(`zmq` + `cppzmq`/`zmq_addon`) — Pub/Sub 메시징(TCP 브리지로 원격 클라이언트 노출).
- WiringPi(`wiringPi`/`wiringPiI2C`/`wiringSerial`) — GPIO/I2C/시리얼. (gpiod에서 롤백됨, [[legacy-code-handling]] 참조)
- libcamera(`camera`, `camera-base`) — 듀얼 카메라 캡처.
- OpenCV(`core`/`imgproc`/`highgui`/`imgcodecs`/`videoio`) — 프레임 처리, JPEG 인코딩, GStreamer(x264) 녹화.
- pthread / atomic / m. C++ 표준은 C++20.
- 하드웨어 연결: GPIO/I2C/시리얼 권한 필요. LD06은 `/dev/ttyUSB0`.

## 실행/테스트 방법
- **작업 환경 = 타깃 하드웨어**: 지금 Claude가 도는 이 호스트가 곧 라즈베리파이 본체다 (`raspi5`, Raspberry Pi 5 Model B, aarch64). 별도 개발/배포 머신 없음 — 빌드·실행이 모두 여기서 직접 일어난다. 하드웨어 장치도 연결돼 있다: LD06 `/dev/ttyUSB0`, I2C `/dev/i2c-*`.
- 빌드: `cmake -S . -B build && cmake --build build --parallel` (또는 VS Code Task "CMake Build"). 빌드 타입은 Debug(`-O0 -g`), 산출물은 `install/AutoDrive`. — 에이전트가 이 환경에서 직접 수행 가능.
- 실행: `./install/AutoDrive 1|2|3` (Direct/Remote/Camera). 실행 디렉터리의 `./AutoDrive.ini` 로드. 실제 모터/서보를 구동하므로 로봇이 물리적으로 움직인다.
- **검증 방법**: 테스트 프레임워크·CI 없음. 변경 검증 = **빌드 성공(에이전트가 여기서 직접) + 검증용 짧은 실행/주행(에이전트가 직접 가능)**. 모터 구동 실행은 로봇이 물리적으로 움직이지만, 변경 확인에 필요한 **짧은 실행·짧은 주행 정도는 에이전트가 직접 해도 된다**(장시간 자율 주행이 아니라 최소 실행으로 한정). (에이전트 행동 지침: [[test-strategy]])

## 설정 (AutoDrive.ini)
- `[protocal]`(오타 그대로 — 외부 클라이언트 호환 때문에 동결, [[untouchable-areas]]): `publish_ip`(기본 `tcp://*:45000`), `subscribe_ip`(기본 `tcp://*:45001`).
- `[calibration]`: `steer_angle_offset`, `camera_pitch_angle_offset`, `camera_yaw_angle_offset` (deg) — 서보 매핑 오프셋. (동결, [[untouchable-areas]])
- `[camera]`: `width`/`height`/`buffer_length`/`frame_rate`/`is_record`(GStreamer 파일 기록).

## 알려진 비활성/롤백 영역
- `PiCar::initRobotHat()`의 CameraMotor(피치/요 서보) 초기화가 주석 비활성화돼 있을 수 있음 → Direct 모드 카메라 서보 키 무반응 가능.
- 최근 커밋에서 gpiod → wiringPi 롤백("gpiod는 문제가 있어서 다시 wiringPi로 롤백함").
- `Basic.h`: `GPIO_PIN_LCD_LED_BACK_LEFT = 21`이 `GPIO_PIN_MCU_RESET = 21`과 핀 번호 중복(코드 주석에 명시).
- 취급 정책: 정리 대상으로 보되 명시 지시 없이 삭제하지 않음 — [[legacy-code-handling]].

## Unresolved
(이번 회차에서 critical 4문항 모두 해소됨 — 남은 항목 없음)
