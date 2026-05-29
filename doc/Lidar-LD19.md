# LiDAR Sensor LD19 기술 요약 (LDROBOT)

> 출처: `doc/Lidar-LD19.pdf`(LD19 Development Manual V2.3, 2022-03-15, 30p)의 기술 내용 + 공식 데이터시트(LD19 Datasheet V2.6) 스펙 보강. 이미지/스크린샷 설명은 제외하고 텍스트·표·코드만 추출.

LD19는 DTOF(Direct Time-of-Flight) 방식의 360° 2D 스캐닝 LiDAR. 적외선 레이저를 전방으로 쏘고 단일광자 수신부로 반사광을 받아, 발사–수신 시간차(비행시간)에 광속을 곱해 거리를 측정한다. 측정한 거리에 각도 측정부의 각도값을 결합해 **포인트 클라우드**를 만들고 UART로 외부에 단방향 전송한다.

- **구성**: 레이저 거리측정 코어 + 무선 전력전송부 + 무선 통신부 + 각도 측정부 + 모터 드라이브부 + 기계 하우징
- **측정 속도**: 초당 4,500회(4.5kHz, 고정)
- **회전 속도 제어**: 내부 속도 제어 기본(전원 인가 후 3초 내 10±0.1Hz 안정화). PWM 입력으로 외부 속도 제어도 가능

---

## 1. 주요 사양 (Datasheet V2.6)

### 전기·기계 파라미터

| 항목 | 최소 | 표준 | 최대 | 비고 |
|------|------|------|------|------|
| **입력 전압** | 4.5V | 5V | 5.5V | |
| **PWM 제어 주파수** | 20kHz | 30kHz | 50kHz | 사각파, 권장 30kHz |
| **PWM High 레벨** | 3.0V | 3.3V | 3.5V | |
| **PWM Low 레벨** | 0V | 0V | 0.5V | |
| **PWM 듀티비** | 0% | 40% | 100% | 듀티 40% → 스캔 10Hz |
| **기동 전류** | - | 300mA | - | |
| **동작 전류** | - | 180mA | - | |
| **기계 치수** | - | 54 × 46.29 × 34.80 mm | - | L×W×H |
| **무게** | - | 47g | - | 연결 케이블 제외 |
| **통신 인터페이스** | - | UART @ 230400 | - | |
| **구동 모터** | - | BLDC | - | 브러시리스 |
| **동작 온도** | -10℃ | 25℃ | 40℃ | |
| **보관 온도** | -30℃ | 25℃ | 70℃ | |

### 광학 파라미터

| 항목 | 값 | 비고 |
|------|------|------|
| **레이저 파장** | 895 / **905** / 915 nm | 적외선 대역 |
| **레이저 안전 등급** | IEC-60825 **Class 1** | 인체 안전 |
| **피치 각도** | 0 / 0.5 / 2° | |

### 성능 파라미터

| 항목 | 값 | 비고 |
|------|------|------|
| **측정 범위** | 0.02m ~ **12m** | 반사율 70% 타깃 기준 |
| **스캔 주파수** | 5 / **10** / 13 Hz | 외부 PWM 속도 제어 시 |
| **측정 주파수** | 4500 Hz (고정) | |
| **평균 거리 정확도** | ±45 mm | 0.3~12m, 70% 확산 반사면, 100회 평균 |
| **거리 표준편차** | 10 mm | 0.3~12m |
| **측정 분해능** | 15 mm | |
| **각도 오차** | ±2° | |
| **각도 분해능** | 0.8° | 기본 10Hz 스캔 기준 |
| **배경광 내성** | 30 KLux | |
| **소음** | ≤45dB @ 30cm | |
| **수명** | 10,000 시간 | |

---

## 2. 통신 인터페이스

외부 시스템과 **ZH1.5T-4P (1.5mm) 커넥터**로 연결(전원 공급 + 데이터 수신).

| 핀 | 신호 | 타입 | 설명 | 최소 | 표준 | 최대 |
|----|------|------|------|------|------|------|
| 1 | **Tx** | 출력 | LiDAR 데이터 출력 | 0V | 3.3V | 3.5V |
| 2 | **PWM** | 입력 | 모터 제어 | 0V | - | 3.3V |
| 3 | **GND** | 전원(-) | 접지 | - | 0V | - |
| 4 | **P5V** | 전원(+) | 전원 공급 | 4.5V | 5V | 5.5V |

**UART 전송 파라미터**

| baud rate | data | stop bit | parity | flow control |
|-----------|------|----------|--------|--------------|
| 230400 bit/s | 8 bits | 1 | none | none |

**속도 제어**
- **내부 제어(기본)**: PWM 핀을 GND에 연결 → 10±0.1Hz로 고정. 외부 속도 제어를 안 쓸 때는 **반드시 PWM 핀을 접지**해야 함.
- **외부 제어**: PWM 핀에 사각파 입력. 트리거 조건 = ① PWM 주파수 20~50kHz(권장 30kHz), ② 듀티비 (45%, 55%) 구간(45%·55% 제외), ③ 최소 100ms 연속 입력. 한 번 외부 제어로 진입하면 전원 재투입 전까지 유지. 모터 개체차가 있어 정확 제어하려면 수신 데이터의 속도값으로 PID 폐루프 제어 필요.

---

## 3. 데이터 프로토콜

LD19는 단방향 통신. 안정 동작 후 명령 없이 측정 데이터 패킷을 계속 송신한다.

### 3.1 패킷 포맷

| 필드 | 길이 | 설명 |
|------|------|------|
| **Header** | 1 byte | 고정값 `0x54` (패킷 시작) |
| **VerLen** | 1 byte | 상위 3bit=패킷 타입(고정 1), 하위 5bit=측정점 수(고정 12) → 고정값 `0x2C` |
| **Speed** | 2 byte | 회전 속도, 단위 °/s |
| **Start angle** | 2 byte | 시작 각도, 단위 0.01° |
| **Data** | 3 byte × 12 | 측정 데이터(아래 참조) |
| **End angle** | 2 byte | 끝 각도, 단위 0.01° |
| **Timestamp** | 2 byte | 타임스탬프, 단위 ms, 최대 30000(도달 시 0부터 재시작) |
| **CRC check** | 1 byte | 자신을 제외한 앞 데이터 전체의 CRC8 |

**측정 데이터 1점 = 3 byte**: distance(2 byte, LSB先, 단위 mm) + intensity(1 byte, 신호 강도). 6m 이내 흰색 물체의 강도 표준값은 약 200.

**C 구조체 정의**
```c
#define POINT_PER_PACK 12
#define HEADER 0x54

typedef struct __attribute__((packed)) {
    uint16_t distance;   // mm
    uint8_t  intensity;  // 신호 강도
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
    uint8_t              header;       // 0x54
    uint8_t              ver_len;      // 0x2C
    uint16_t             speed;        // °/s
    uint16_t             start_angle;  // 0.01°
    LidarPointStructDef  point[POINT_PER_PACK];
    uint16_t             end_angle;    // 0.01°
    uint16_t             timestamp;    // ms
    uint8_t              crc8;
} LiDARFrameTypeDef;
```

### 3.2 각도 계산 (선형 보간)

각 점의 각도는 시작/끝 각도의 선형 보간으로 구한다.
```c
step  = (end_angle - start_angle) / (len - 1);
angle = start_angle + step * i;   // i ∈ [0, len), len = 패킷 내 측정점 수(12)
```

### 3.3 CRC8 체크

256엔트리 룩업 테이블 기반. (테이블 전체는 원문 매뉴얼 §3.1 참조)
```c
uint8_t CalCRC8(uint8_t *p, uint8_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}
```

### 3.4 파싱 예제

수신 데이터:
```
54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 CD
00 E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50
```

| 필드 | 원시값 | 해석 |
|------|--------|------|
| Speed | `0x0868` | 2152 °/s |
| Start angle | `0x7EAB` = 32427 | 324.27° |
| End angle | `0x82BE` = 33470 | 334.70° |
| 점1 distance | `0x00E0` | 224 mm |
| 점1 intensity | `0xE4` | 228 |
| 점2 distance | `0x00DC` | 220 mm |
| … | … | … |
| 점12 distance | `0x00B0` | 176 mm |
| 점12 intensity | `0xEA` | 234 |

> 원본 PDF는 점2 distance를 "00DCH = 200mm", 점2 intensity를 "00B0H = 176mm"로 표기하나, `0x00DC = 220`이 맞고 intensity 자리에 distance 단위(mm) 값이 잘못 들어가 있다(원문 오타). 위 표는 올바른 값으로 보정했다.

---

## 4. 좌표계

- **왼손 좌표계** 사용.
- 회전 중심 = 좌표 원점.
- 센서 정면 = **0도** 방향.
- 회전 각도는 **시계 방향**으로 증가.

> 주의: ROS/Rviz는 오른손 좌표계(반시계)를 쓰므로, ROS 패키지의 `toLaserScan` 함수에서 좌표 변환을 거쳐야 정상 표시된다.

---

## 5. 개발 킷 / SDK / ROS

### USB 어댑터 (Windows)
- USB 변환 칩: **CP2102**. Silicon Labs VCP 드라이버 필요.
  - 드라이버: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- 포인트 클라우드 시각화 도구 `ld_desktop` 제공.
  - 다운로드: https://github.com/ldrobotSensorTeam/ld_desktop_tool/releases

### Linux 디바이스 권한
```bash
ls /dev/ttyUSB*                 # 연결 확인
sudo chmod 777 /dev/ttyUSB*     # 권한 부여
```
launch 파일의 `port_name`을 실제 포트(예: `/dev/ttyUSB0`)로 수정.

### ROS / ROS2 / SDK 저장소

| 종류 | 지원 환경 | 저장소 |
|------|-----------|--------|
| **ROS** | Kinetic(16.04) / Melodic(18.04) / Noetic(20.04) | [ldlidar_stl_ros](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros) |
| **ROS2** | Foxy(20.04) | [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) |
| **Linux SDK** | C++11 / C99, CMake+GCC | [ldlidar_stl_sdk](https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk) |

**SDK 빌드·실행 (참고 — 본 프로젝트의 검증 흐름과 별개)**
```bash
cd ~/ldlidar_ws/ldlidar_stl_sdk
mkdir build && cd build
cmake ../ && make
./ldlidar_stl /dev/ttyUSB0
```

> Raspberry Pi(SBC)에서의 ROS 사용은 별도 매뉴얼 《LDRobot_LD06 Raspberry Pi Raspbian User manual》 참조(LD06·LD19 공용).

---

## 6. 3D CAD 모델

LDROBOT 공식 GitHub에서 LD19 STEP 모델을 다운로드해 저장했다.

- **저장 위치**: [doc/LD19_3D_model/00_ld19-tof_20220315.stp](LD19_3D_model/00_ld19-tof_20220315.stp) (STEP 포맷, 2022-03-15, 약 9.3MB)
- **출처**: LDROBOT 공식 3D 모델 저장소 [ldrobotSensorTeam/Product_3D_model](https://github.com/ldrobotSensorTeam/Product_3D_model) — release `v1.0` "Publish LDROBOT LiDAR 3D model file(LD19/LD14)"
- **직접 링크**: https://github.com/ldrobotSensorTeam/Product_3D_model/releases/download/v1.0/LDROBOT_LiDAR_LD19_3D_STP_V1.0.zip
- Fusion 360 / SolidWorks 등에서 열람 가능(STEP은 범용 CAD 교환 포맷).

> 참고 외부 CAD 라이브러리: [GrabCAD - Lidar LD19](https://grabcad.com/library/lidar-ld19-1) (커뮤니티 업로드본), [Cults3D - LD19D300](https://cults3d.com/en/3d-model/gadget/high-precision-lidar-sensor-ld19d300-accurate-cad)

---

## 7. 개정 이력 (매뉴얼)

| 버전 | 날짜 | 내용 |
|------|------|------|
| 1.0 | 2020-09-01 | 최초 작성 |
| 1.1 | 2021-01-15 | Transform() 함수 제거 |
| 2.0 | 2022-02-27 | 개발 킷 설명 추가 |
| 2.1 | 2022-03-06 | 문서 그래픽 디자인·포맷 정비 |
| 2.2 | 2022-03-09 | 표지 제목·일부 내용 수정 |
| 2.3 | 2022-03-15 | 문서 내 문제 문장 수정 |
</content>
</invoke>
