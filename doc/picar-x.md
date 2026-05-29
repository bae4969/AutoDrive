# PiCar-X 기술 매뉴얼 (SunFounder)

> picar-x.pdf(2023-02-21, 153p)에서 기술적 내용만 추출. 자율주행 역사·감사글·저작권 등 비기술 내용은 제외.

PiCar-X는 Raspberry Pi 기반 AI 자율주행 로봇카. 2축 카메라 모듈, 초음파 모듈, 라인트래킹(그레이스케일) 모듈로 색상/얼굴/교통표지 인식, 장애물 회피, 라인 트래킹 등을 수행. 프로그래밍은 **Python**과 **EzBlock(Blockly)** 두 가지 지원.

- 소프트웨어 스택: Python, OpenCV(컴퓨터 비전), Google TensorFlow(딥러닝), eSpeak(TTS)
- 제어 보드: SunFounder Robot HAT (좌/우 구동모터, 조향·팬틸트 서보, ADC/PWM/I2C 확장, 스피커, 블루투스 내장)

---

## 1. Robot HAT 하드웨어 사양

| 항목 | 사양 |
|------|------|
| **모터 포트 (좌/우)** | 2채널 XH2.54. 좌측=GPIO 4, 우측=GPIO 5 |
| **I2C 핀** | Raspberry Pi에서 나오는 2채널 I2C |
| **PWM 핀** | 12채널, P0–P12 |
| **ADC 핀** | 4채널, A0–A3 |
| **디지털 핀** | 4채널, D0–D3 |
| **전원 포트** | 7–12V PH2.0 2핀 입력. Raspberry Pi와 Robot HAT 동시 급전 |

**배터리 인디케이터 LED**
- 7.8V 초과: LED 2개 점등
- 6.7V ~ 7.8V: LED 1개 점등
- 6.7V 미만: 모두 소등 (충전 필요)

**버튼/스위치**
- **USR LED**: 프로그램으로 제어 (출력 1=ON, 0=OFF)
- **RST 버튼**: 짧게 누르면 프로그램 리셋. LED 켜질 때까지 길게 누르면 블루투스 연결 해제
- **USR 버튼**: 프로그램으로 기능 설정 (누름=입력 "0", 뗌=입력 "1")
- **전원 스위치**: Robot HAT 전원 ON/OFF. 전원 포트 연결 시 Raspberry Pi는 부팅되지만, Robot HAT을 쓰려면 스위치를 ON 해야 함

---

## 2. 조립 시 주의사항

- 조립 전 **18650 배터리 2개**를 구매해 완충할 것. Robot HAT은 배터리를 충전하지 못하므로 **별도 충전기** 필요.
- 조립 후 Robot HAT으로 서보에 전원이 들어온 상태에서는 **서보(스티어링 기어)를 손으로 강제로 돌리지 말 것** — 서보 손상.

---

## 3. Python 환경 구축

### 3.1 필요 부품
- **필수**: Raspberry Pi, 2.5A 이상 전원 어댑터(micro USB), 8GB 이상 micro SD 카드
- **선택**: 스크린(HDMI), 마우스·키보드(USB), HDMI 케이블, 케이스, 3.5mm 사운드/이어폰

### 3.2 OS 설치 (Raspberry Pi Imager)
1. https://www.raspberrypi.org/software/ 에서 OS에 맞는 Raspberry Pi Imager 다운로드·설치
2. SD 카드 삽입
3. **CHOOSE OS → Raspberry Pi OS (other)** 선택

> ⚠️ **Bullseye로 업그레이드 금지**. 일부 기능이 동작하지 않으므로 **Debian Buster** 유지 권장. **Raspberry Pi OS (Legacy)** (데스크톱 포함) 설치 권장.

4. 사용할 SD 카드 선택
5. 고급 옵션(설정 버튼 또는 `Ctrl+Shift+X`): **SSH 활성화**, username/hostname 설정
   - hostname 미설정 시 기본값 `raspberrypi` — 원격 접속에 이 hostname 사용
6. WiFi 설정 (wifi country는 ISO 3166-1 alpha-2 2자리 코드)
7. **WRITE** 클릭 → 쓰기 완료 대기

### 3.3 Raspberry Pi 원격 접속

#### 스크린이 있는 경우
SD 카드 삽입 → 마우스·키보드 연결 → HDMI로 스크린 연결(Pi 4는 **HDMI0**, 전원에 가까운 포트) → 전원 인가 → 데스크톱 표시

#### 스크린이 없는 경우 — SSH 접속

**공통 흐름**
```bash
# hostname으로 IP 확인 (Windows PowerShell)
ping -4 raspberrypi.local

# SSH 접속 (Mac/Linux/Windows 공통)
ssh pi@raspberrypi.local
```
- 최초 접속 시 fingerprint 확인 → `yes`
- 비밀번호 입력 (입력 문자는 화면에 표시되지 않음 — 정상)

**OS 업데이트**
```bash
sudo apt update
sudo apt upgrade
```

> Windows에서 `ssh`가 인식되지 않으면 시스템이 오래된 것 → OpenSSH 수동 설치(부록 참고) 또는 PuTTY 사용.

#### VNC 원격 데스크톱
SSH로는 카메라 영상을 볼 수 없으므로 컴퓨터 비전 프로젝트는 VNC/XRDP 또는 직접 스크린 연결 필요.

```bash
sudo raspi-config
# 3 Interface Options → P3 VNC → <Yes> → <OK> → <Finish>
```

Mac에서 VNC 접속 시 설정 (선택):
```bash
# 전 계정 공통 파라미터 지정 파일 생성
sudo nano /etc/vnc/config.d/common.custom
# 내용: Authentication=VncAuth  (Ctrl+X → Y → Enter)

# Mac 접속용 VNC 비밀번호 설정
sudo vncpasswd -service

sudo reboot
```
- VNC Viewer 설치 후 hostname 또는 IP 입력 → 접속
- 접속 주소 형식: `vnc://<username>@<hostname>.local` 또는 `vnc://<username>@<IP address>`
- VNC 옵션: Encryption=Prefer off, Authentication=VNC password

### 3.4 모듈 설치

```bash
# (Lite 버전 OS인 경우 Python3 관련 패키지 필수)
sudo apt install git python3-pip python3-setuptools python3-smbus

# robot-hat
cd /home/pi/
git clone https://github.com/sunfounder/robot-hat.git
cd robot-hat
sudo python3 setup.py install

# vilib
cd /home/pi/
git clone https://github.com/sunfounder/vilib.git
cd vilib
sudo python3 install.py

# picar-x (v2.0 브랜치)
cd /home/pi/
git clone -b v2.0 https://github.com/sunfounder/picar-x.git
cd picar-x
sudo python3 setup.py install

# i2s 앰프 설치 (사운드 필수)
cd /home/pi/picar-x
sudo bash i2samp.sh
```
- `i2samp.sh` 실행 중 프롬프트는 모두 `y` → 재부팅. 재부팅 후 소리가 없으면 스크립트를 여러 번 실행.
- `setup.py` 실행 시 네트워크 문제로 다운로드 실패 가능 → 재시도(`y` 입력).

### 3.5 I2C·카메라 인터페이스 활성화
기본 비활성 상태이므로 활성화 필요.
```bash
sudo raspi-config
# 3 Interfacing Options → P5 I2C → <Yes> → <OK>
# 3 Interfacing Options → P1 Camera → <Yes> → <OK>
# <Finish> → 재부팅 <Yes>
```

### 3.6 서보 각도 영점 조정 (Servo Adjust)
서보 가동 범위는 **-90°~90°**이나 공장 출하 각도는 랜덤. 영점(0°)을 맞추지 않고 조립하면 코드 실행 시 오작동하거나 서보가 블록되어 소손될 수 있음.

```bash
cd /home/pi/picar-x/example
sudo python3 servo_zeroing.py
```
1. 서보 암을 축에 끼우고 임의 각도로 살짝 돌려둠
2. 배터리 케이블 연결, 전원 ON → 1~2분 후 부팅 완료음
3. 위 스크립트 실행
4. 서보 케이블을 **P11 포트**에 연결
5. 서보 암이 0° 위치로 회전. 안 되면 RST 버튼으로 Robot HAT 재시작

**주의**
- 서보 나사로 고정하기 전엔 케이블을 뽑지 말 것 (고정 후 분리 가능)
- 전원이 들어온 상태에서 서보를 돌리지 말 것 — 손상. 축 각도가 안 맞으면 뽑아서 재삽입
- 각 서보 조립 전 P11에 연결하고 전원 ON 하여 0°로 세팅

---

## 4. 캘리브레이션 (Calibration)

조립 편차나 서보 자체 한계로 각도가 틀어질 수 있어 보정. 보통 보정각은 **-5°~5°**. 편차가 너무 크면 Servo Adjust로 돌아가 0° 재설정 후 재조립.

```bash
cd /home/pi/picar-x/example/calibration
sudo python3 calibration.py
```
터미널 키 조작:
- `R`: 앞바퀴 방향 서보 동작 테스트
- `1` → `W/S`: 앞바퀴 서보 선택 후 좌우 치우침 없이 정면 정렬
- `2` → `W/S`: Pan 서보(팬틸트 좌우) 정렬
- `3` → `W/S`: Tilt 서보(팬틸트 상하) 정렬
- `E`: 전진 테스트 (모터 배선 역방향 확인)
- `4`,`5` → `Q`: 좌/우 모터 선택 후 회전 방향 보정
- `Spacebar`: 보정값 저장 (`y` 확인) → `Esc` 종료

---

## 5. Python 예제

> 모든 예제는 `cd /home/pi/picar-x/example` 후 실행. 소스 경로로 이동해야 수정·실행 가능.

### 5.1 기본 이동 (move.py)
```python
from picarx import Picarx
import time

if __name__ == "__main__":
    try:
        px = Picarx()
        px.forward(30)
        time.sleep(0.5)
        for angle in range(0,35):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        for angle in range(-35,0):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)
        px.forward(0)
        time.sleep(1)
        for angle in range(0,35):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)
        for angle in range(-35,0):
            px.set_camera_servo1_angle(angle)
            time.sleep(0.01)
        for angle in range(0,35):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)
        for angle in range(35,-35,-1):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)
        for angle in range(-35,0):
            px.set_camera_servo2_angle(angle)
            time.sleep(0.01)
    finally:
        px.forward(0)
```

**핵심 API (`picarx` 모듈)**
| 함수 | 기능 |
|------|------|
| `forward(speed)` | 주어진 속도로 전진 |
| `backward(speed)` | 후진 |
| `stop()` | 정지 |
| `set_dir_servo_angle(angle)` | 조향 서보 각도 설정 |
| `set_camera_servo1_angle(angle)` | 카메라 Pan(좌우) 서보 |
| `set_camera_servo2_angle(angle)` | 카메라 Tilt(상하) 서보 |

### 5.2 장애물 회피 (avoiding_obstacles.py)
초음파로 전방 거리 측정 → 25cm 미만이면 좌회전(-35°), 아니면 직진(0°).
```python
from picarx import Picarx

def main():
    try:
        px = Picarx()
        # px = Picarx(ultrasonic_pins=['D2','D3']) # trig, echo
        px.forward(30)
        while True:
            distance = px.ultrasonic.read()
            print("distance: ",distance)
            if distance > 0 and distance < 300:
                if distance < 25:
                    px.set_dir_servo_angle(-35)
                else:
                    px.set_dir_servo_angle(0)
    finally:
        px.forward(0)

if __name__ == "__main__":
    main()
```
- 초음파 모듈은 `picarx`에 포함 → `px.ultrasonic.read()`로 거리(cm) 획득

### 5.3 라인 트래킹 (minecart_plus.py)
그레이스케일 모듈로 검은 선을 따라 전진. 어두운 색 테이프로 가능한 직선 라인 제작.
```python
from picarx import Picarx

if __name__=='__main__':
    try:
        px = Picarx()
        # px = Picarx(grayscale_pins=['A0', 'A1', 'A2'])
        px_power = 10
        while True:
            gm_val_list = px.get_grayscale_data()
            print("gm_val_list:",gm_val_list)
            gm_status = px.get_line_status(gm_val_list)
            print("gm_status:",gm_status)
            if gm_status == 'forward':
                px.forward(px_power)
            elif gm_status == 'left':
                px.set_dir_servo_angle(12)
                px.forward(px_power)
            elif gm_status == 'right':
                px.set_dir_servo_angle(-12)
                px.forward(px_power)
            else:
                px.set_dir_servo_angle(0)
                px.stop()
    finally:
        px.stop()
```
**그레이스케일 API**
- `get_grayscale_data()`: 3개 센서값을 오른쪽→왼쪽 순으로 출력. 밝을수록 값이 큼
- `get_line_status(data)`: 센서값 기반으로 `forward`/`left`/`right`/`stop` 행동 반환
  - 3개 센서 모두 임계값 초과 = 흰색(라인 없음) → `stop`
  - 오른쪽(첫째) 센서가 검은 선 감지 → `right`
  - 중앙 센서 감지 → `forward`
  - 왼쪽 센서 감지 → `left`

### 5.4 Text to Speech (tts_example.py)
TTS는 **eSpeak** 소프트웨어로 구현. 사용 전 `i2samp.sh`로 스피커 활성화 필요.
```python
from robot_hat import TTS

if __name__ == "__main__":
    words = ["Hello", "Hi", "Good bye", "Nice to meet you"]
    tts_robot = TTS()
    for i in words:
        print(i)
        tts_robot.say(i)
```

### 5.5 컴퓨터 비전 (computer_vision.py)
> 카메라 영상 표시 필요 → 데스크톱(스크린 직결 또는 VNC/XRDP) 환경에서 실행. SSH로는 불가.

```python
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

with PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 24
    rawCapture = PiRGBArray(camera, size=camera.resolution)
    time.sleep(2)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        cv2.imshow("video", img)
        rawCapture.truncate(0)  # 캐시 해제
        k = cv2.waitKey(1) & 0xFF
        if k == 27:  # ESC
            break
print('quit ...')
cv2.destroyAllWindows()
camera.close()
```
- `picamera`로 영상 취득. OpenCV 이미지는 BGR 순서의 numpy 배열
- JPEG 인코딩/디코딩 손실을 피하려면 `picamera.array`의 `PiRGBArray` + `'bgr'` 포맷 사용(처리 속도도 향상). RGB와 BGR는 크기·구성은 같고 색상 평면만 반대

### 5.6 색상 검출 (color_detect.py)
HSV 색공간의 H 범위로 색상 판별. 빨강 검출 시 사각형으로 박스 표시.
```python
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time

# HSV 색공간 H값 범위
color_dict = {'red':[0,4],'orange':[5,18],'yellow':[22,37],'green':[42,85],
              'blue':[92,110],'purple':[115,165],'red_2':[165,180]}
kernel_5 = np.ones((5,5),np.uint8)  # 5x5 컨볼루션 커널 (모폴로지 연산용)

def color_detect(img,color_name):
    resize_img = cv2.resize(img, (160,120), interpolation=cv2.INTER_LINEAR)  # 연산량 감소
    hsv = cv2.cvtColor(resize_img, cv2.COLOR_BGR2HSV)  # BGR→HSV
    color_type = color_name
    mask = cv2.inRange(hsv, np.array([min(color_dict[color_type]), 60, 60]),
                            np.array([max(color_dict[color_type]), 255, 255]))
    if color_type == 'red':  # 빨강은 H 범위가 양 끝에 걸쳐 두 마스크 OR
        mask_2 = cv2.inRange(hsv, (color_dict['red_2'][0],0,0), (color_dict['red_2'][1],255,255))
        mask = cv2.bitwise_or(mask, mask_2)
    morphologyEx_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_5, iterations=1)  # 열림 연산
    _tuple = cv2.findContours(morphologyEx_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(_tuple) == 3:   # opencv3.x / 4.x 호환
        _, contours, hierarchy = _tuple
    else:
        contours, hierarchy = _tuple
    color_area_num = len(contours)
    if color_area_num > 0:
        for i in contours:
            x,y,w,h = cv2.boundingRect(i)
            if w >= 8 and h >= 8:  # 1/4 축소했으므로 원본 좌표 복원 위해 x4
                x = x * 4; y = y * 4; w = w * 4; h = h * 4
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(img,color_type,(x,y), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)
    return img,mask,morphologyEx_img

with PiCamera() as camera:
    camera.resolution = (640,480)
    camera.framerate = 24
    rawCapture = PiRGBArray(camera, size=camera.resolution)
    time.sleep(2)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        img,img_2,img_3 = color_detect(img,'red')
        cv2.imshow("video", img)
        cv2.imshow("mask", img_2)
        cv2.imshow("morphologyEx_img", img_3)
        rawCapture.truncate(0)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
cv2.destroyAllWindows()
camera.close()
```
**`color_detect()` 처리 4단계**: ① 대상 색을 이진 이미지로 추출 → ② 모폴로지 변환 → ③ 윤곽선(contour) 검출 → ④ 인식 객체에 프레임 표시. (H: 색상, S: 채도, V: 명도. 조명에 따라 범위 조정 필요)

### 5.7 얼굴 검출 (human_face_detect.py)
OpenCV Haar Cascade 분류기 사용. 모델 파일 `haarcascade_frontalface_default.xml`이 예제 폴더에 위치.
```python
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

def human_face_detect(img):
    resize_img = cv2.resize(img, (320,240), interpolation=cv2.INTER_LINEAR)  # 연산량 감소
    gray = cv2.cvtColor(resize_img, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
    faces = face_cascade.detectMultiScale(gray, 1.3, 2)  # 얼굴 검출
    face_num = len(faces)
    if face_num > 0:
        for (x,y,w,h) in faces:
            x = x*2; y = y*2; w = w*2; h = h*2  # 1/2 축소했으므로 x2 복원
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    return img

# 모델 로드
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

with PiCamera() as camera:
    camera.resolution = (640,480)
    camera.framerate = 24
    rawCapture = PiRGBArray(camera, size=camera.resolution)
    time.sleep(2)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = frame.array
        img = human_face_detect(img)
        cv2.imshow("video", img)
        rawCapture.truncate(0)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
cv2.destroyAllWindows()
camera.close()
```
- Haar feature 기반 Cascade Classifier (Viola-Jones, 2001). 양성/음성 이미지로 학습된 cascade 함수로 객체 검출
- **처리 3단계**: ① 그레이스케일 변환 → ② 얼굴 검출(bounding rect) → ③ 프레임 표시

### 5.8 비디오 카 (video_car.py)
1인칭 시점 주행. 키보드 제어 + 사진 촬영.
- `O`: 가속 / `P`: 감속 / `W`: 전진 / `S`: 후진 / `A`: 좌회전 / `D`: 우회전 / `F`: 정지 / `T`: 촬영 / `ESC`·`Ctrl+C`: 종료
```python
from utils import reset_mcu
reset_mcu()
from picarx import Picarx
from vilib import Vilib
from time import sleep, time, strftime, localtime
import readchar

px = Picarx()

def take_photo():
    _time = strftime('%Y-%m-%d-%H-%M-%S',localtime(time()))
    name = 'photo_%s'%_time
    path = "/home/pi/Pictures/picar-x/"
    Vilib.take_photo(name, path)
    print('\nphoto save as %s%s.jpg'%(path,name))

def move(operate, speed):
    if operate == 'stop':
        px.stop()
    else:
        if operate == 'forward':
            px.set_dir_servo_angle(0); px.forward(speed)
        elif operate == 'backward':
            px.set_dir_servo_angle(0); px.backward(speed)
        elif operate == 'turn left':
            px.set_dir_servo_angle(-30); px.forward(speed)
        elif operate == 'turn right':
            px.set_dir_servo_angle(30); px.forward(speed)

def main():
    speed = 0
    status = 'stop'
    Vilib.camera_start(vflip=False,hflip=False)
    Vilib.display(local=True,web=True)
    sleep(2)
    while True:
        print("\rstatus: %s , speed: %s "%(status, speed), end='', flush=True)
        key = readchar.readkey().lower()
        if key in ('wsadfop'):
            if key == 'o':
                if speed <=90: speed += 10
            elif key == 'p':
                if speed >=10: speed -= 10
                if speed == 0: status = 'stop'
            elif key in ('wsad'):
                if speed == 0: speed = 10
                if key == 'w':
                    if status != 'forward' and speed > 60: speed = 60  # 역방향 전환 시 순간 전류 제한
                    status = 'forward'
                elif key == 'a': status = 'turn left'
                elif key == 's':
                    if status != 'backward' and speed > 60: speed = 60
                    status = 'backward'
                elif key == 'd': status = 'turn right'
            elif key == 'f': status = 'stop'
            move(status, speed)
        elif key == 't':
            take_photo()
        elif key == readchar.key.CTRL_C or key in readchar.key.ESCAPE_SEQUENCES:
            px.stop(); Vilib.camera_close()
            break
        sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("error:%s"%e)
    finally:
        px.stop()
        Vilib.camera_close()
```
- `Vilib`(vilib 모듈): `camera_start(vflip, hflip)`, `display(local, web)`, `take_photo(name, path)`

### 5.9 앱 제어 (SunFounder Controller)
SunFounder Controller 앱으로 Raspberry Pi/Pico 기반 로봇 제어. Button/Switch/Joystick/D-pad/Slider/Throttle Slider 위젯, Digital Display/Ultrasonic Radar/Grayscale Detection/Speedometer 입력 위젯 제공. 17개 영역(A–Q)에 위젯 배치. 라이브 영상 스트리밍 지원.

```bash
# sunfounder-controller 설치 (robot-hat, vilib, picar-x 선행 설치 필요)
cd ~
git clone https://github.com/sunfounder/sunfounder-controller.git
cd ~/sunfounder-controller
sudo python3 setup.py install

# 실행
cd ~/sunfounder-controller/examples
sudo python3 picarx_control.py
```
- 앱(App Store/Google Play)에서 SunFounder Controller 설치 → `+`로 컨트롤러 생성 → 위젯 배치 → Connect(근처 로봇 자동 검색, 이름은 `picarx_control.py`에 정의됨이 상시 실행 중이어야 함)
- 모바일 기기와 PiCar-X가 **같은 LAN**에 있어야 함. 자동 검색 안 되면 IP 수동 입력
- **위젯 기능**: A=속도 표시, D=그레이스케일 3센서 상태(검은선/흰색/절벽), E=장애물 회피, I=라인 추종, J=음성인식(forward/backward/left/right), K=전후좌우 주행, Q=카메라 상하좌우, N=색상 인식, O=얼굴 인식, P=객체 인식(약 90종, [coco_labels.txt](https://github.com/sunfounder/vilib/blob/master/workspace/coco_labels.txt))

---

## 6. EzBlock (Blockly 그래픽 프로그래밍)

6~12세 또는 프로그래밍 경험이 없거나 빠른 테스트를 원할 때 적합. Mac/PC/Android 거의 모든 기기 지원. 그래픽·Python 두 환경 제공.

> EzBlock 사용 시 **EzBlock OS가 사전 설치된 전용 OS 이미지**로 SD 카드를 다시 구워야 함. 새/미사용 TF 카드 권장.

### 6.1 서보 영점 조정
Python판과 동일(범위 -90°~90°, P11 포트에 연결해 0° 세팅). 단,
> EzBlock 앱으로 프로그램을 다운로드하면 이 zeroing 기능은 비활성화됨.

### 6.2 EzBlock Studio 설치·연결
- EzBlock Studio 설치(앱 또는 웹 버전) → 제품과 EzBlock 연결(Wi-Fi, Bluetooth 설정) → 캘리브레이션 → 예제 실행
- 연결 후 캘리브레이션 단계: 좌측 포인트=Pan-Tilt(카메라) 보정, 우측 포인트=앞바퀴 방향 보정. 미세 조정이며, 한계까지 눌러도 안 맞으면 분해 후 재조립 권장
- 재캘리브레이션: connect 아이콘 → Settings → Calibrate

### 6.3 EzBlock 핵심 블록·개념

| 프로젝트 | 핵심 블록/개념 |
|----------|----------------|
| **Move** | forward(%) / backward(%) — 가용 출력 백분율. 조향 블록 범위 -45~45(음수=좌회전). delay(ms) 블록. stop 블록 |
| **Remote Control** | Remote Control 페이지에서 Joystick 드래그 → (X,Y) 좌표(-100~100). `map value` 블록으로 범위 재매핑 |
| **Test Ultrasonic** | `Ultrasonic get distance` 블록으로 전방 거리(cm) 읽기. Variable로 거리값 공유, Print로 디버그 |
| **Test Grayscale** | 그레이스케일 블록으로 센서값 읽기(A0=좌, A1=중앙, A2=우). 순흑=0. `create list with`로 3센서값 리스트화 |
| **Color Detection** | HSV 범위 지정 → OpenCV로 배경 노이즈 제거 → 박스 표시. 6색 모델(red/orange/yellow/green/blue/purple). 한 번에 한 색만. camera monitor on/off (off여도 객체검출은 동작) |
| **Face Detection** | `face detection` on 블록. 팬틸트 조향 블록(값 증가=우/상). `of detected face` 블록으로 좌표/크기/개수 읽기 |
| **Sound Effect** | `say` 블록(text/number). `repeat` 블록. 수학연산 블록. `play sound effects - with volume -%` 블록(사이렌·총소리 등, 볼륨 0~100) |
| **Background Music** | `play background music` 블록(Start 함수에 배치). `set background music volume`(0~100). Slider 위젯 + `slider [A] get value` |
| **Say Hello** | `if do` 블록 + 조건문 블록(=, >, < 등). 얼굴 인식 시 고개 끄덕이고 "Hello!" |
| **Music Car** | `if else do / else if do`로 다중 조건 판정. 초음파로 벽 회피 + 음악 재생 |
| **Cliff Detection** | 그레이스케일로 절벽 감지. 리스트 변수 반환 함수로 단순화 |
| **Minecart** | 그레이스케일 라인 추종. `set ref to ()` 블록으로 임계값 설정(흰/검 중간값). 좌/우 프로브가 검은 테이프 감지 시 해당 방향 조향 |
| **Minecart Plus** | Minecart에 탈선 복구 추가. 별도 `to do something` 함수로 후진·재정렬(반환값 없음) |
| **Bullfight** | 색상 검출(red)로 카메라를 빨간 천에 고정 후 차체 방향 추종. 화면 3x3 그리드로 (x,y) 좌표화, 다중 타겟 시 최대 타겟 크기 기록 |
| **Beware of Pedestrians** | 얼굴 검출 10회 중 검출 시 [count]+1, count>3이면 정지(주행 안전 시뮬레이션) |
| **Traffic Sign Detection** | 교통표지 4종 인식 + 라인 추종 결합. Stop 표지=정지, Forward 표지=전진. `set ref to ()` 임계값 |
| **Orienteering** | 원격제어 + 6색 카드 탐색 게임. TTS로 다음 색 안내, 3색을 랜덤 순서로 탐색 |

---

## 7. 부록

### 7.1 PuTTY (Windows SSH)
1. PuTTY 다운로드
2. Session → Host Name에 RPi IP 입력, Port=22
3. Open → 보안 경고 시 Yes
4. `login as:` → `pi`, password → `raspberry`(기본값)
   - 비밀번호 입력 문자는 표시되지 않음(정상). `inactive` 표시 시 연결 끊김 → 재접속

### 7.2 PowerShell로 OpenSSH 설치
`ssh`가 인식되지 않을 때(구형 시스템). PowerShell을 **관리자 권한**으로 실행:
```powershell
# OpenSSH.Client 설치
Add-WindowsCapability -Online -Name OpenSSH.Client~~~~0.0.1.0

# 설치 확인
Get-WindowsCapability -Online | Where-Object Name -like 'OpenSSH*'
# State : Installed 확인
```
- 위 프롬프트가 안 나오면 시스템이 너무 오래된 것 → PuTTY 등 서드파티 SSH 도구 사용 권장
- PowerShell 재시작(관리자) 후 `ssh` 사용 가능

### 7.3 배터리 사양
| 항목 | 값 |
|------|-----|
| 전압 | 3.7V |
| 규격 | 18650 |
| 종류 | 충전식 리튬이온 |
| 단자 | **버튼 탑(Button Top)** — 배터리 홀더 접촉 보장 |
| 보호회로 | **없음(No Protective Board)** — 보호회로 있으면 과전류 보호로 로봇 전원 차단·정지 가능 |
| 용량 | 3000mAh 이상 권장 |

- Robot HAT은 충전 불가 → 별도 충전기 필요
- Robot HAT 전원 인디케이터 2개가 모두 꺼지면 전압 부족 → 충전 필요

---

## 8. FAQ

**Q1. EzBlock OS 설치 후 서보가 0°로 안 돌아감**
1. 서보 케이블 연결·Robot HAT 전원 확인
2. Reset 버튼
3. EzBlock Studio에서 이미 프로그램을 실행했다면 P11용 커스텀 프로그램이 사라진 것 → EzBlock Studio에서 서보 각도 0° 설정 프로그램을 수동 작성

**Q2. VNC 사용 시 "desktop cannot be displayed"**
터미널에서 `sudo raspi-config`로 해상도 변경

**Q3. 서보가 가끔 이유 없이 중앙으로 복귀**
구조물 등에 막혀 목표 위치에 도달 못 하면 서보가 소손 방지를 위해 전원 차단 보호 모드 진입. 일정 시간 전원 차단 후 PWM 신호가 없으면 자동으로 원위치로 복귀.

---

## 9. 3D 모델 (CAD)

> **주의**: SunFounder는 PiCar-X의 **공식 CAD/STEP 파일을 공개하지 않는다**. 공식 [sunfounder/picar-x](https://github.com/sunfounder/picar-x) 저장소에도 CAD 디렉토리가 없다(코드·문서만 제공). 아래는 제조사 공식이 아닌 **서드파티/학술 3D 모델**이다.

PiCar-X 디지털트윈 학술 프로젝트에서 제작한 시뮬레이션용 3D 모델을 다운로드해 저장했다.

- **저장 위치**: [doc/picar-x_3D_model/](picar-x_3D_model/)
  - `meshes/Chassis.dae`, `WheelFront{Left,Right}.dae`, `WheelRear{Left,Right}.dae` — COLLADA 메시(섀시 + 바퀴 4개)
  - `urdf/picarx.urdf`, `urdf/model.sdf` — ROS/Gazebo용 로봇 기술 파일(링크·조인트·관성 정의)
- **출처**: [cau-se/ARCHES-PiCar-X](https://github.com/cau-se/ARCHES-PiCar-X) (`PiCar-X/simulation/picarx_description/`), 디지털트윈 연구용. **Apache-2.0** 라이선스(`meshes/LICENSE.txt` 동봉).
- **용도**: Gazebo/Ignition 등 **ROS 시뮬레이션**용 모델. STEP 같은 편집형 솔리드 CAD가 아니라 메시(.dae) + 기술서(URDF/SDF) 조합이다.

> 참고 커뮤니티 CAD/3D 프린트 모델:
> - [Theosakamg/PiCar_Hardware](https://github.com/Theosakamg/PiCar_Hardware) — FreeCAD + STL/DAE. 단, **PiCar-V/S(구형 3륜) 베이스**용이라 PiCar-X와 형상이 다름.
> - [Printables - PiCar-X 부품](https://www.printables.com/model/1288585-sunfounder-picar-x-front-bumper) 등 커뮤니티 3D 프린트 부품(프론트 범퍼 등 일부 부품 단위).
>
> 편집 가능한 정식 솔리드 CAD가 필요하면 SunFounder에 직접 문의가 가장 확실하다.
