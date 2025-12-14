# 🦆 Duckiebot ROS 2 Simulation in NVIDIA Isaac Sim

## 📌 Project Overview
ROS 2 (Robot Operating System 2) Framework와 NVIDIA Isaac Sim의 물리 시뮬레이션을 연동하여 Duckiebot의 자율 주행 및 원격 제어 시스템을 구현했습니다.

### ⚙️ Key Technologies
| Category | Technology | Description |
| :--- | :--- | :--- |
| **Simulation** | **NVIDIA Isaac Sim** | 옴니버스(Omniverse) 환경 기반의 고성능 물리 시뮬레이터 |
| **Control** | **ROS 2 (Humble/Iron)** | 로봇 제어 및 센서 데이터 처리 프레임워크 |
| **Vision** | **OpenCV** | 실시간 객체 추적(Red Object Tracking) 및 면적 기반 거리 제어 |
| **Mechanism** | **Action Graph, USD, Articulation** | Isaac Sim 내 ROS-Bridge 연동 및 로봇 구동 설정 |

---

## 🚀 Architecture and Project Structure

### 시스템 아키텍처
본 프로젝트는 **Isaac Sim의 ROS 2 Bridge**를 통해 ROS 노드와 시뮬레이션 환경이 양방향으로 데이터를 주고받도록 설계되었습니다. 

1.  **시뮬레이션 → ROS 2:** 카메라 이미지 토픽 (`/rgb`) 전송.
2.  **ROS 2 → 시뮬레이션:** 이동 명령 토픽 (`/cmd_vel`) 및 사용자 정의 서비스(`SetColor.srv`) 수신.

### 📁 Project Structure
본 저장소에는 Isaac Sim Scene 파일과 ROS 2 스크립트가 모두 포함되어 있습니다.

| File/Folder | Type | Description |
| :--- | :--- | :--- |
| `duckiebot_backup.usd` | USD Scene | Isaac Sim에서 설정된 최종 시뮬레이션 환경 파일 |
| `duckie_vision.py` | Python Script | **자율 주행 (객체 추적) 로직** 구현 (OpenCV) |
| `duckie_arrow.py` | Python Script | 키보드 기반 **원격 조종 (Teleoperation)** 로직 구현 |
| `led_service.py` | Python Script | ROS 2 사용자 정의 서비스 (`SetColor.srv`) 예제 |

---

## ⚙️ How to Run (빌드 및 실행 방법)

### 3.1. Prerequisites (필수 조건)
* NVIDIA Isaac Sim 설치 및 환경 설정 (ROS Bridge Extension 활성화 필수)
* ROS 2 (Humble 또는 Iron) 환경 설정
* Python 3 및 ROS 2 Python 라이브러리 (ex. `cv_bridge`, `opencv-python`) 설치 완료

### 3.2. Step-by-Step Execution

1.  **저장소 클론:**
    ```bash
    git clone [https://github.com/yoonhyungsik/isaac-sim.git](https://github.com/yoonhyungsik/isaac-sim.git)
    cd isaac-sim
    ```
2.  **Isaac Sim Scene 로드:**
    * Isaac Sim을 실행하고 **`duckiebot_backup.usd`** 파일을 Open합니다.
    * 시뮬레이션 **`Play`** 버튼을 클릭하여 물리 엔진을 시작합니다.
3.  **ROS 2 제어 노드 실행:**
    * **WSL/Linux 터미널**을 열고, 클론한 폴더(`isaac-sim`)로 이동합니다.
    * **자율 주행 (객체 추적):**
        ```bash
        python3 duckie_vision.py
        ```
    * **원격 조종 (Teleoperation):**
        ```bash
        python3 duckie_arrow.py
        ```
        (터미널 활성화 후 WASD 또는 방향키로 조작)

---

## 🎥 Demonstration Video

### 🎬 시연 영상
프로젝트의 최종 결과는 아래 영상을 통해 확인하실 수 있습니다.

| Feature | Description |
| :--- | :--- |
| **Object Tracking** | 빨간색 객체의 면적 변화(거리)와 중심점 오차에 따른 조향/속도 제어 시연 |
| **ROS Teleoperation** | 키보드 입력에 따른 즉각적인 로봇 반응 및 `cmd_vel` 메시지 통신 확인 |

