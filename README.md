# Robot_Project
두산로보틱스 프로젝트
# 🤖 Robotic Automation Portfolio

이건도 | Robot Software Engineer (ROS2 / Robotics / Automation)

본 저장소는 실제 로봇 제어 및 자동화 공정 프로젝트 경험을 기반으로 구성한 포트폴리오입니다.
ROS2 기반 로봇 암 제어, AMR(자율주행로봇) 시스템 구축, 스마트 팩토리 시뮬레이션 모델링 등 실제 산업 환경에서의 문제 해결 경험을 담았습니다.

---

## 📌 Projects Overview

| **Project 1. Triangular Cup Stacking Robot** | **Project 2. AMR 기반 자재 운반 자동화 공장** | **Project 3. Smart Factory Automated Simulation** |
|:--------------------------------------------:|:---------------------------------------------:|:------------------------------------------------:|
| <img width="345" height="444" alt="image" src="https://github.com/user-attachments/assets/56dc50b2-ea75-410c-8e3c-57e9f1356f6e" /> | <img width="345" height="444" alt="image" src="https://github.com/user-attachments/assets/c4d06500-04e0-4b02-ae1e-90fbc7d567bc" /> | <img width="345" height="444" alt="image" src="https://github.com/user-attachments/assets/a56f00ac-c1da-4c86-aa1f-feacb447417b" /> |
| ROS2 기반 로봇 암 + 그리퍼를 활용해 삼각뿔 컵 조형물을 자동으로 구축하는 프로젝트 | Vision + ROS2 + PyQt 기반, 박스 픽업 및 컨베이어 이동을 자동화한 AMR 시스템 구축 | Gazebo 시뮬레이션 환경 제작 및 다중 AMR 경로 최적화 기반 스마트 팩토리 설계 |


---

# 🦾 Project 1. Triangular Cup Stacking Robot (ROS2 Manipulation)

### 📎 기술 스택

* **환경**: Ubuntu 22.04 / ROS2 / Python
* **기술**: MoveIt, RVIZ2, Force Control, Bézier Curve Planning
* **역할 기여도**:

  * 패스트 스태킹 알고리즘 개발 **70%**
  * 싱글 스태킹 알고리즘 개발 **20%**

---

## 🎯 프로젝트 목표

* 입력된 **기준 좌표 + 층 수**만으로 반복 가능한 컵 조형물 구축
* 실 환경에서의 오차(중력/탄성/높이)를 고려한 정밀 제어
* 하드코딩 제거 → **알고리즘 기반 자동 배치**

---

## 🧠 핵심 알고리즘 (Fast vs Single Stacking)

<table>
  <tr>
    <th align="center">✔ Fast Stacking (속도 중심)</th>
    <th align="center">✔ Single Stacking (정확도 중심)</th>
  </tr>
  <tr>
    <td align="center">
      <img width="350" src="https://github.com/user-attachments/assets/81109e26-6403-45ed-bbb6-54225e42a4c0" />
    </td>
    <td align="center">
      <img width="350" src="https://github.com/user-attachments/assets/1733a068-e033-40a5-9949-c2a23aa0c922" />
    </td>
  </tr>
  <tr>
    <td valign="top">
      <ul>
        <li>여러 개의 컵을 동시에 잡기 위한 <strong>Wide-grip picking 로직</strong></li>
        <li>이동 중 컵을 지지하는 <strong>툴 각도 보정</strong></li>
        <li>삼각형 외심 좌표 기반 배치</li>
      </ul>
    </td>
    <td valign="top">
      <ul>
        <li>Force control 기반, 실제 컵 <strong>position 추정</strong></li>
        <li>Bézier 곡선 기반 <strong>부드러운 경로 생성</strong></li>
        <li>가장 먼 컵부터 배치하여 충돌 경로 방지</li>
        <li>뒤집힌 컵을 올리기 위한 <strong>Flip motion</strong> 구현</li>
      </ul>
    </td>
  </tr>
</table>

---

## 🔧 문제점 및 개선점

### ❗ 문제점 (Fast Stacking)
- 초기 컵 위치 오차 → 성공률 **40%**
- Release 시 아래층 컵 밀림 → 구조적 불안정성

### ✅ 개선점 (Single Stacking)
- Force Control 도입 → **정확한 위치 보정**
- Bézier 경로 생성 → **부드럽고 안전한 이동**
- 가장 먼 컵부터 배치 → **경로 간섭 최소화**
- Flip motion 구현 → **조형물 꼭대기 컵 뒤집기 가능**

---

## 💡 담당 업무 정리

* Force Control 적용, 실제 컵 위치 pose 읽어오는 로직 구현
* Bézier 기반 이동 경로 생성 및 MoveIt 동작 개선
* 기준 좌표–층수 기반 자동 배치 수학적 로직 구현
* Flip 동작(컵을 눕혀 뒤집는 동작) 알고리즘 설계 및 안전범위 튜닝

---

# 🚚 Project 2. AMR 기반 자재 운반 자동화 공장

### 📎 기술 스택

* **환경**: Ubuntu 22.04 / Python
* **기술**: ROS2, OpenCV(aruco), YOLOv8, PyQt, Conveyor Motor Control
* **역할 기여도**:

  * YOLO 학습 **30%**
  * Camera Calibration **50%**
  * 컨베이어 벨트 제어 **100%**

---

## 🎯 프로젝트 개요

1. GUI에서 특정 박스 운반 명령 전송
2. AMR이 아루코 마커 기반 위치 인식
3. 박스 픽업 → 필요 수량만큼 컨베이어로 이동
4. 바구니로 이동 후 하역
5. 지정 위치까지 최종 운반

---

## 🔍 주요 기능

### ✔ Vision (YOLOv8 + ArUco)

* "red", "blue", "mark", "aruco" 총 4종 객체 감지
* 픽업 위치 오차 감소를 위한 내부 파라미터 분석

  * 초점거리 약 **1381–1382 (거의 동일)**
  * 실제 AMR 높이 기반 mm/px 비율 계산 및 보정

### ✔ Conveyor Belt Control

* Arduino + Step Motor 기반
* 상태 표시

  ```
  [DISCONNECT] → [INIT] → [READY] → [RUN]
  ```
* 벨트 동작 및 물류 이동 동기화 담당

### ✔ AMR Control

* MoveIt + ROS2 기반 박스 잡기 pose 이동
* PyQt GUI로 AMR 상태 및 카메라 화면 실시간 표시

---

## 💡 담당 업무 정리

* 아루코 마커 & YOLOv8 학습을 통한 위치 인식 개선
* 카메라 Calibration 직접 진행해 오차 보정
* 컨베이어 벨트 전체 제어 로직 설계
  (모터 드라이버, 아두이노 신호, 속도/방향 제어 등)
* GUI 내 AMR 상태 및 Vision 피드 통합 구성

---

# 🏭 Project 3. Smart Factory Automated Simulation

### 📎 기술 스택

* **환경**: Ubuntu 22.04
* **기술**: ROS2, Gazebo, RVIZ2, PyQt, C++ Plugin 개발
* **역할 기여도**:

  * Gazebo 모델 제작 및 물리 엔진 구성 **70%**

---

## 🎯 프로젝트 목표

* 다중 AMR 기반 스마트 팩토리 시나리오 구축
* 실시간 모니터링 & 중앙 관제 시스템 개발
* URDF/SDF 기반 로봇 및 공장 전공정 모델링

---

## 🧱 Gazebo 모델링 구성

* **컨베이어 벨트(직접 C++ 플러그인 제작)**

  * 벨트 표면, 길이 읽어오기
  * 물리 엔진 StepSize 기반, x축 이동 속도 적용
* **World 파일 구성**

  * ground plane
  * 고정 카메라 3대
  * 컨베이어 벨트 5대
  * dump truck 4대
  * 조형물, 벽 등 장애물 모델링

---

## 🤖 AMR 다중 협업 시나리오

* 각 부품마다 8개 공정을 순환
* A* 기반 경로 계획
* Object–컨베이어 흐름 분리
* AMR들의 적재/조달/출하 자동화

---

## 💡 담당 업무 정리

* 스마트 팩토리 전체 구조 모델링
* Gazebo 플러그인(C++)를 통한 Conveyor 제작
* Vision 인식(mAP 0.99급) 기반 목표지점 지정
* 시뮬레이션–관제–센서 데이터를 하나의 시스템으로 설계
