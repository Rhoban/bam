# DYNAMIXEL XM430-W350 — BAM 액추에이터 모델링 가이드

> 작성일: 2026-05-23  
> 저장소: `~/Desktop/robot make/bam`  
> 대상 모터: DYNAMIXEL XM430-W350-T/R

---

## 1. 현재 진행 상황 (완료된 작업)

| 단계 | 내용 | 상태 |
|------|------|------|
| 코드 추가 | `DynamixelXM430W350` 통신 클래스 구현 | ✅ 완료 |
| 코드 추가 | `XM430Actuator` 피팅용 클래스 구현 | ✅ 완료 |
| 코드 수정 | `bam/actuators.py`에 `"xm430"`, `"xm430-w350"` 등록 | ✅ 완료 |
| 코드 수정 | `record.py` — 모터 분기·baudrate·torque-off 보장 | ✅ 완료 |
| 코드 수정 | `all_record.py` — XM430용 kp 스케일, vin 자동 설정 | ✅ 완료 |
| 버그 수정 | `record.py` `dt` 필드 누락 → `simulate.py` KeyError | ✅ 수정됨 |
| 버그 수정 | `--vin` 기본값 모터별 자동 설정 (MX=15V, XL=7.5V, XM430=12V) | ✅ 수정됨 |
| 데이터 수집 | kp×4 × trajectory×4 = **16개 로그** 기록 완료 | ✅ `/tmp/bam_xm430/` |
| **다음 단계** | `bam.fit` 실행 → 파라미터 최적화 | ⬜ 미완료 |

### 기록된 로그 요약
- 경로: `/tmp/bam_xm430/`
- kp: 400, 800, 1200, 1600
- trajectory: sin_sin, lift_and_drop, up_and_down, sin_time_square
- dt: ~0.015 s (sin_sin/up_and_down/sin_time_square), ~0.009 s (lift_and_drop)
- 총 16개 JSON 파일, 약 2.2 MB

---

## 2. XM430-W350 하드웨어 사양 (데이터시트 기준)

| 항목 | 값 |
|------|----|
| 통신 프로토콜 | Protocol 2.0 전용 |
| 공장 기본 Baud Rate | 57,600 bps (addr 8 = 1) |
| 권장 입력전압 | 12.0 V (10.0 – 14.8 V) |
| 정격 스톨 토크 | 4.1 Nm @ 12V / 4.8 Nm @ 14.8V |
| 스톨 전류 | 2.3 A @ 12V |
| 무부하 속도 | 46 rpm @ 12V |
| 위치 해상도 | 4096 steps / 360° (0.088°/step) |
| 기어비 | 353.5 : 1 |
| 무게 | 82 g |
| Position P Gain 범위 | 0 – 16,383 (기본값 800) |

---

## 3. 제어 테이블 주소 (Protocol 2.0)

### EEPROM (전원 꺼도 유지)
| 주소 | 이름 | 크기 | 설명 |
|------|------|------|------|
| 11 | Operating Mode | 1B | **3** = Position Control (기본값) |
| 13 | Protocol Type | 1B | 2 = Protocol 2.0 |
| 36 | PWM Limit | 2B | 최대 PWM 제한 |
| 38 | Current Limit | 2B | 최대 전류 제한 |

### RAM (전원 켤 때마다 초기화)
| 주소 | 이름 | 크기 | 설명 |
|------|------|------|------|
| 64 | Torque Enable | 1B | 1=ON, 0=OFF |
| 80 | Position D Gain | 2B | 0–16383 |
| 82 | Position I Gain | 2B | 0–16383 |
| 84 | Position P Gain | 2B | 0–16383, 기본 800 |
| 98 | Bus Watchdog | 1B | 0=비활성 |
| 108 | Profile Acceleration | 4B | 0=비활성 |
| 112 | Profile Velocity | 4B | 0=비활성 |
| 116 | Goal Position | 4B | 목표 위치 (0–4095) |
| 124 | Present PWM | 2B | 현재 PWM (signed, 0.113%/step) |
| 126 | Present Current | 2B | 현재 전류 (signed, 2.69mA/step) |
| 128 | Present Velocity | 4B | 현재 속도 (signed, 0.229rpm/step) |
| 132 | Present Position | 4B | 현재 위치 (0–4095) |
| 144 | Present Input Voltage | 2B | 입력전압 (0.1V/step) |
| 146 | Present Temperature | 1B | 온도 (°C) |

---

## 4. 위치 변환 규칙

BAM의 각도 규칙: **q = 0 rad → raw 2048** (모터 중심, 180°)

```
raw = round(4096 × (q_rad / (2π) + 0.5))   # rad → raw
q   = 2π × (raw / 4096 − 0.5)              # raw → rad
```

- q = −π rad → raw ≈ 0
- q = +π rad → raw ≈ 4095
- **팬듈럼 설치 시**: 팬듈럼 아래 수직(중력 방향)이 raw 2048이 되도록 기계적으로 정렬

---

## 5. BAM 물리 모델 (`XM430Actuator`)

BAM은 DC 모터 + 전압 제어 모델을 사용한다:

### 5-1. 제어 법칙 (firmware 근사)

```
duty_cycle = error_gain × kp × (q_target − q)
           = (1/128) × kp × (q_target − q)
```

- `error_gain = 1/128`: XM430 Position P Gain 테이블 변환 공식 `Kp_actual = Kp_TBL / 128` 에서 유도
- `kp`: 로그에 저장된 Position P Gain 값 (400–1600 범위로 스윕)
- 이 값은 초기 근사값이며, **oscilloscope로 실제 goal-position error → PWM 응답을 측정**하면 더 정확한 `error_gain`을 얻을 수 있음

```
voltage = vin × clip(duty_cycle, −1, +1)
```

### 5-2. 토크 방정식 (DC 모터)

```
τ = (kt / R) × V − (kt² / R) × dq
```

| 파라미터 | 기호 | 초기값 | 범위 | 단위 |
|----------|------|--------|------|------|
| 입력전압 | vin | 12.0 | — | V |
| 토크 상수 | kt | 1.78 | 1.0–3.0 | Nm/A |
| 등가 저항 | R | 5.2 | 2.0–10.0 | Ω |
| 관성 (armature) | J_arm | 0.005 | 0.0001–0.05 | kg·m² |

**kt 초기값 근거**: 스톨 토크 4.1 Nm / 스톨 전류 2.3 A ≈ 1.78 Nm/A  
**R 초기값 근거**: 12 V / 2.3 A ≈ 5.2 Ω

### 5-3. 운동 방정식 (팬듈럼 테스트벤치)

```
(m·L² + J_arm) × d²q/dt² = τ − m·g·L·sin(q) − b·dq − τ_friction
```

| 파라미터 | 의미 |
|----------|------|
| m | 팬듈럼 질량 (0.1475 kg) |
| L | 팬듈럼 길이 (0.150 m) |
| b | 점성 마찰 (friction_viscous) |
| τ_friction | 쿨롱 마찰 (friction_base) |

---

## 6. 기록 절차 (record.py / all_record.py)

### 단일 trajectory 기록

```bash
python3 -m bam.dynamixel.record \
    --motor xm430 \
    --port /dev/ttyUSB0 \
    --baudrate 57600 \
    --id 1 \
    --mass 0.1475 \
    --length 0.150 \
    --vin 12.0 \
    --kp 800 \
    --logdir /tmp/bam_xm430 \
    --trajectory lift_and_drop
```

### 전체 스윕 (kp×4 × trajectory×4 = 16개)

```bash
python3 -m bam.dynamixel.all_record \
    --motor xm430 \
    --port /dev/ttyUSB0 \
    --baudrate 57600 \
    --id 1 \
    --mass 0.1475 \
    --length 0.150 \
    --vin 12.0 \
    --logdir /tmp/bam_xm430
```

기본 kp 스윕: **400, 800, 1200, 1600**  
커스텀 지정: `--kps "200,400,800,1600"`  
커스텀 trajectory: `--trajectories "sin_sin,lift_and_drop"`

### 기록된 JSON 구조

```json
{
  "mass": 0.1475,
  "length": 0.15,
  "kp": 800,
  "vin": 12.0,
  "motor": "xm430",
  "id": 1,
  "baudrate": 57600,
  "trajectory": "lift_and_drop",
  "dt": 0.009,
  "entries": [
    {
      "position": 0.012,
      "speed": 0.003,
      "load": 5,
      "input_volts": 12.1,
      "temp": 32,
      "current": 0.013,
      "current_raw": 5,
      "pwm": 1.2,
      "pwm_raw": 11,
      "position_raw": 2073,
      "velocity_raw": 1,
      "timestamp": 0.0075,
      "goal_position": 0.0,
      "torque_enable": true
    }
  ]
}
```

---

## 7. 피팅 절차 (bam.fit)

### 모델 선택 기준

| 모델 | 마찰 모델 | 권장 상황 |
|------|-----------|-----------|
| m1 | Coulomb (기본) | 빠른 초기 피팅 |
| m2 | Stribeck | 저속 stick-slip이 보이는 경우 |
| m3 | Load-dependent | 부하에 따라 마찰이 변하는 경우 |
| m4 | Stribeck + Load-dependent | 정밀 모델링 |

### 실행 순서

> **권장 이터레이션: 3000 이상** (적을수록 수렴 불안정, 많을수록 정확도 향상)

```bash
# 1. 초기 피팅 (m1 — Coulomb 마찰)
python3 -m bam.fit \
    --actuator xm430 \
    --logdir /tmp/bam_xm430 \
    --model m1 \
    --trials 3000

# 2. 결과 확인 후 더 정밀한 모델로 재피팅
python3 -m bam.fit \
    --actuator xm430 \
    --logdir /tmp/bam_xm430 \
    --model m3 \
    --trials 3000

# 3. 피팅 결과 평가
python3 -m bam.fit \
    --actuator xm430 \
    --logdir /tmp/bam_xm430 \
    --model m1 \
    --eval
```

피팅 완료 후 출력 JSON 파일은 `--logdir`로 지정한 폴더(`/tmp/bam_xm430/`)에 저장된다.

### 피팅 결과 해석

- **낮은 RMSE** (< 0.05 rad): 모델이 실제 동작을 잘 재현함
- **kt가 범위 한계에 수렴**: `error_gain`이 부정확할 가능성 → oscilloscope 측정 권장
- **R이 너무 작게 수렴**: 전압 데이터(`vin`)가 부정확하거나 기어 손실 영향

---

## 8. 결과 시각화 (bam.plot)

`bam.plot`은 **processed 로그 폴더**를 `--logdir`에 넣어 사용한다.  
`--sim`을 켜면 `--params`에 지정한 모델 JSON의 시뮬레이션 결과와 실제 로그를 함께 그린다.  
`--params`는 여러 개를 넣을 수 있어 모델 간 비교가 가능하다.

### 실로그만 확인 (피팅 전 데이터 점검)

```bash
python3 -m bam.plot \
    --actuator xm430 \
    --logdir /tmp/bam_xm430
```

### 시뮬레이션 vs 실로그 비교 (단일 모델)

```bash
python3 -m bam.plot \
    --actuator xm430 \
    --logdir /tmp/bam_xm430 \
    --sim \
    --params /tmp/bam_xm430/params_m3.json
```

### 여러 모델 동시 비교

```bash
python3 -m bam.plot \
    --actuator xm430 \
    --logdir /tmp/bam_xm430 \
    --sim \
    --params /tmp/bam_xm430/params_m1.json /tmp/bam_xm430/params_m3.json
```

| 인자 | 설명 |
|------|------|
| `--logdir` | processed 로그 폴더 경로 |
| `--sim` | 모델 시뮬레이션 결과를 실로그와 함께 표시 |
| `--params` | 모델 파라미터 JSON 파일 (여러 개 가능, 스페이스로 구분) |

---

## 9. 알려진 제한사항 및 주의사항

### error_gain 정확도
`error_gain = 1/128`은 XM430 Position P Gain 테이블 변환 공식에서 유도한 **초기 근사값**이다.  
XM430의 실제 firmware는 position → current → PWM의 2단 제어 구조이므로, 단순 voltage 제어 모델과 완전히 동일하지 않다.  
더 높은 정확도가 필요하면 oscilloscope로 `goal_position_error` 대비 `Present PWM` 응답을 직접 측정해 `error_gain`을 보정한다.

### Profile 설정
`prepare_for_recording()`에서 `Profile Velocity = 0`, `Profile Acceleration = 0`으로 설정한다.  
이 상태에서 XM430은 가능한 한 빠르게 목표 위치로 이동한다 (트라페조이달 프로파일 비활성).  
BAM 시뮬레이터는 프로파일 없는 직접 위치 제어를 가정하므로 반드시 이 상태로 기록해야 한다.

### I/D Gain
기록 중 I Gain = 0, D Gain = 0으로 설정한다.  
I/D 게인이 활성화되면 BAM의 단순 P 제어 근사가 깨져 피팅 오차가 커진다.

### Baudrate
공장 기본값은 57600 bps. DYNAMIXEL Wizard로 변경했다면 `--baudrate` 인자로 명시한다.  
1 Mbps로 변경된 경우: `--baudrate 1000000`

### torque_enable = False 구간
lift_and_drop 등 일부 trajectory는 중간에 토크를 끄는 구간이 있다.  
이 구간의 데이터는 자유 낙하 동역학을 학습하므로 마찰 파라미터 추정에 중요하다.  
기록이 정상적으로 되었는지 JSON의 `torque_enable` 필드 변화를 확인한다.

---

## 10. 파일 위치 정리

```
~/Desktop/robot make/bam/
├── bam/
│   ├── actuators.py                  ← "xm430", "xm430-w350" 등록
│   └── dynamixel/
│       ├── dynamixel.py              ← DynamixelXM430W350 클래스
│       ├── actuator.py               ← XM430Actuator (피팅용 모델)
│       ├── record.py                 ← 단일 trajectory 기록
│       └── all_record.py             ← 전체 스윕 기록

/tmp/bam_xm430/                       ← 기록된 raw data (16개 JSON)
```
