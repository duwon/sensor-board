
# 📘 BLE Sensor Board (Broadcaster) 최신 정리 (2025-10-12 업데이트)

## 1️⃣ MCU 및 기본 설정 (변동 없음)

* **MCU:** nRF52833
* **클럭:** HFXO 32 MHz + LFXO 32.768 kHz
* **SDK:** nRF Connect SDK v3.1.0 (Zephyr v4.1.99)
* **보드 타깃:** `nrf52833dk_nrf52833` (커스텀 오버레이 적용)

---

## 2️⃣ 핀맵 요약 (확정)

| 기능               | 핀                 | 방향    | 설명                             |
| ---------------- | ----------------- | ----- | ------------------------------ |
| LFXO             | P0.00 / P0.01     | —     | 32.768 kHz                     |
| **I2C SCL/SDA**  | **P0.05 / P0.04** | I/O   | I²C0 (기본 400 kHz, 필요시 100 kHz) |
| **UART TX/RX**   | **P0.15 / P0.17** | 출력/입력 | UART0 115200 8N1 (**HWFC 끔**)  |
| **LED**          | P0.11             | 출력    | 상태 LED (Active High)           |
| **BTN**          | P1.09             | 입력    | 버튼 (Active Low)                |
| **NTC ADC**      | P0.03             | 입력    | SAADC AIN1                     |
| **EN_SENSOR**    | P0.30             | 출력    | 센서 전원 Enable                   |
| **EN_RPU**       | P0.31             | 출력    | RPU/DIP 전원 Enable              |
| **IO_INT**       | P0.20             | 출력    | 센서 인터럽트                        |
| **SOH_ALARM/OK** | P0.09 / P0.10     | 입력    | LTC3337 상태                     |
| **RESET**        | P0.18             | 입력    | JTAG Reset                     |

> **하드웨어 주의:** 압력센서(0x28)가 I²C 라인을 1.8 V로 끌어내리던 이슈 확인. 제거 시 3 V 정상화 및 0x20(TCA9534), 0x6A(LSM6DSO) 응답 정상. 0x28 사용 시 **3 V 통일** 또는 **양방향 I²C 레벨시프터** 필수.

---

## 3️⃣ I²C 버스 구성

| 디바이스              | 주소       | 역할          |
| ----------------- | -------- | ----------- |
| **TCA9534**       | 0x20     | DIP 스위치 입력  |
| **LTC3337**       | 0x64(예정) | 배터리 SOH     |
| **LSM6DSO**       | 0x6A     | IMU         |
| **Honeywell SSC** | 0x28     | 압력센서(전압 유의) |

---

## 4️⃣ DIP 스위치 정의 (TCA9534)

| 비트    | 의미          | 1(H)       | 0(L)       |
| ----- | ----------- | ---------- | ---------- |
| [2:0] | Model(Main) | 모델 선택      | —          |
| [3]   | Mode(Sub)   | On         | Off        |
| [4]   | PHY         | 1 Mbps     | Coded S=8  |
| [5]   | Period      | Long(10 s) | Short(5 s) |
| [6]   | PWR         | Battery    | Adapter    |
| [7]   | Legacy      | New 패킷     | Legacy 패킷  |

---

## 5️⃣ DeviceTree 오버레이 핵심 (오늘 반영 유지)

* **UART0**: TX=P0.15, RX=P0.17, **HWFC 비활성**
* **I2C0**: SCL=P0.05, SDA=P0.04, **Fast(400 kHz)**
* pinctrl 충돌 방지용 라벨 사용(`uart0_my_default/sleep`, `i2c0_default/sleep`) 후 `&uart0`, `&i2c0`에 **명시 바인딩**
* (선택) 콘솔 고정:

  ```dts
  / {
    chosen {
      zephyr,console = &uart0;
      zephyr,shell-uart = &uart0;
    };
  };
  ```

---

## 6️⃣ Kconfig (prj.conf) 핵심 (정상 동작 확인된 세트)

```conf
# UART / Shell
CONFIG_SERIAL=y
CONFIG_UART_CONSOLE=y
CONFIG_PRINTK=y
CONFIG_CONSOLE=y
CONFIG_UART_0_NRF_HWFC=n
CONFIG_SHELL=y
CONFIG_SHELL_BACKEND_SERIAL=y
CONFIG_SHELL_STACK_SIZE=4096

# Logging
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_LOG_RUNTIME_FILTERING=y
CONFIG_LOG_PROCESS_THREAD_STACK_SIZE=2048

# Settings (MAC 고정용: NVS 백엔드 사용)
CONFIG_SETTINGS=y
CONFIG_FLASH=y
CONFIG_NVS=y
CONFIG_SETTINGS_NVS=y
```

---

## 7️⃣ 프로젝트 구조 (업데이트 반영)

```
sensor_board_fw/
├── boards/
│   └── nrf52833dk_nrf52833.overlay
├── src/
│   ├─ main.c            # 루프: Tx_Ble() → Start_Sleep(5/10s) → Wakeup()
│   │                    # DIP 주기 매 루프 반영, LED HB, 진단로그 스위치
│   ├─ ble_adv.c         # Init_Ble(), Tx_Ble(), Ble_Stop()  ← 브로드캐스트 전용
│   │                    #  - 기본 Manufacturer + GAP Appearance(0x0340)
│   │                    #  - Rx_Ble()는 유지해도 "미사용"(loop에서 호출 안 함)
│   │                    #  - settings + bt_id_reset으로 MAC 고정
│   ├─ sleep_if.c        # Start_Sleep(): 인터페이스/광고 off + k_sleep
│   │                    # Wakeup(): 전원 복구 + DIP 재적용 + Init_Ble() (시작은 Tx_Ble)
│   ├─ dip_switch.c      # Get_Switch(): EN_RPU on→TCA9534 두 번 읽기→파싱
│   ├─ gpio_if.c, sensors.c, ltc3337.c
│   ├─ debug.c           # 쉘(diag echo / i2c-scan / imu-who / dip-read / log on|off)
│   └─ app_diag.c        # app_diag_log_enable()/is_enabled()
├── include/
│   ├─ ble_adv.h         # ble_cfg_t, API, **static inline to_0p625()**
│   ├─ app_diag.h, dip_switch.h, gpio_if.h, sensors.h, sleep_if.h, debug.h
├── prj.conf
└── CMakeLists.txt
```




---

## 8️⃣ **BLE 스택/광고 개선 사항 **

* **MAC 주소 고정:**
  `settings_load()`로 `app/ble_id` 로드 → 없으면 **첫 부팅 시 현 주소 저장** → `bt_id_reset(0, stored)`로 **항상 동일 주소 사용**.
* **Device Type(appearance) 노출:**
  AD에 **GAP Appearance = Generic Sensor (0x0340)** 추가 → 스캐너에서 **Device type 표기**.
* **스캔 응답(Rx_Ble):**
  `Rx_Ble()`로 **Scan Response(최대 31B)** 등록 가능(예: Shortened Name), 레거시/EXT 각각 방식에 맞게 반영.
* **초기화/시작 정책:**

  * `Init_Ble()`: **“현재 모드만”** stop 후 파라미터 반영(EXT는 핸들 준비까지만).
  * `Tx_Ble(mfg, len)`: **브로드캐스팅 시작/갱신** 담당(레거시 ↔ EXT 분리 운용).
  * `Ble_Stop()`: **현재 설정된 모드만** stop.
* **인터벌 유틸:** `to_0p625(ms)`(정상 포함). 10 s도 스펙 최대(0x4000)로 클램프 처리.

---

## 9️⃣ Sleep 모드 연계 (컨셉 확정 / 코드 반영)

* 루프: **Tx_Ble() → Start_Sleep(5 s/10 s) → Wakeup() → (센서 수집 예정)**
* `Start_Sleep(sec)`: LED Off, `EN_SENSOR/EN_RPU` Off, **Ble_Stop()** 후 `k_sleep()` 대기
* `Wakeup()`: `EN_RPU`→`EN_SENSOR` On, **DIP 재읽기/적용**, `Init_Ble(last_cfg)` 후 **Tx_Ble()** 재시작

  > DIP 변경 시 즉시 `Init_Ble()`로 PHY/간격/레거시 전환 반영됨.

---

## 🔟 디버그/쉘 (정상 동작)

* **진단 로그 스위치:** `diag log on|off` → 루프 로그 (`g_diag_log_on`) on/off
* **기본 명령:**

  * `diag echo <text…>`: UART RX 에코
  * `diag i2c-scan`: I²C 0x03–0x77 스캔
  * `diag imu-who`: LSM6DSO WHO_AM_I
  * `diag dip-read`: EN_RPU on → DIP 읽기/파싱
  * (선택) `diag ble start/stop`, `diag phy 1m|coded`, `diag period 5|10` 확장 여지

---

## 1️⃣1️⃣ 런타임 동작 체크리스트

1. 부팅 로그에 **BT ID[0]**(고정 MAC) 출력되는지 확인.
2. 스캐너(nRF Connect 앱/데스크톱)에서 **Device type(Generic Sensor)** 표기 확인.
3. DIP[7]=1이면 Legacy, 0이면 EXT로 광고되는지 확인.
4. 루프 주기: DIP[5]에 따라 **5 s/10 s**로 Sleep/깨어남 반복되는지.
5. `diag log off` 시 루프 로그 정지, `diag log on` 시 재개.
6. `diag i2c-scan`에서 0x20/0x6A 응답 확인, 0x28 사용 시 전압조건 충족 전에는 제거 유지.

---

## 1️⃣2️⃣ 할 일

* **sensors.c 마무리**
  * 압력(0x28) 복귀 전략(3 V/레벨시프터) 반영 + 변환/평균화(윈저라이즈드)
  * NTC 변환(B3435) 적용, 온도/압력 평균치 산출
  * LSM6DSO 기본 init + 저속 3축 샘플(필요 시 RMS)
* **LTC3337 SOH**: OK/ALARM 핀 + I²C 맵 정의, 배터리 % 산출 → 광고 필드 반영
* **BLE 패킷 포맷** 고도화
  * **신형 38 B**(확장), DIP[7]=1 시 **Legacy 23 B**
  * 필드 인덱스/엔디안 최종 고정
* **스캐너 호환성 스위치**: 스캐너 인식률 저하 시 **Flags-only + Name Short** fallback 토글
* **쉘 확장**: `diag ble cfg …`, `diag ble sr on/off`, `diag sleep <sec>` 등

---

## 1️⃣3️⃣ 빌드/플래시

```bash
west build -b nrf52833dk_nrf52833 . --domain sensor-board -p
west flash
```

터미널: **115200 8N1, Flow OFF, CR 또는 CR+LF**




좋습니다. 아래는 지금까지의 **BLE Sensor Board (Broadcaster)** 정리 문서에
오늘 논의한 **NTC 온도센서 보정 및 동작 내용**을 반영한 최신 버전입니다.
(기존 섹션 번호 유지하며, 새 섹션 **“1️⃣4️⃣ NTC 온도센서”** 추가됨)

---

## 1️⃣4️⃣ **NTC 온도센서 (AIN1, CX100 단위 반환)**

### 🔹 하드웨어

| 항목 | 내용                                                          |
| -- | ----------------------------------------------------------- |
| 핀  | P0.03 (SAADC AIN1)                                          |
| 회로 | VDD(≈2.18 V) ─ NTC(10 kΩ, B3435) ─ ADC ─ Rpull(10 kΩ) ─ GND |
| 비고 | NTC가 **위쪽**, 풀다운 저항이 **아래쪽**에 연결된 구조                        |

---

### 🔹 펌웨어 구현 (`sensors.c`)

* `read_ntc_ain1_cx100(int16_t *cx100)`
  → SAADC로 AIN1을 읽고 **섭씨 × 100 (CX100)** 단위로 반환.
  → 정상 반환 시 0, 실패 시 음수 에러 코드 (`-ERANGE`, `-ENODEV` 등).

* 채널 설정

  ```c
  .gain       = ADC_GAIN_1_6,       // Full scale ≈ 3.6 V
  .reference  = ADC_REF_INTERNAL,   // 0.6 V ref
  .input_positive = NRF_SAADC_INPUT_AIN1
  ```

* 전압–저항 변환식 (NTC 위 / 풀다운 아래)

  ```
  R_ntc = R_pull * (VDD - Vout) / Vout
  ```

* 온도 변환식 (β 식)

  ```
  1/T = 1/T0 + (1/B) * ln(R/R0)
  → T(°C) = (1/T) - 273.15
  ```

  기본 상수

  ```
  R0 = 10000 Ω,  B = 3435,  T0 = 298.15 K (25 °C)
  ```

---

### 🔹 보정/확인 결과

| 항목         | 측정값      | 계산 결과                          | 비고                |
| ---------- | -------- | ------------------------------ | ----------------- |
| VDD        | 2.18 V   | —                              | SAADC VREF 보정용    |
| Vout       | 1.18 V   | → Rntc ≈ 8.47 kΩ → T ≈ 29.8 °C | 상온(25 °C 근처)와 근접  |
| 출력값 (보정 전) | 52.36 °C | —                              | 분압 식 오류로 2배 높게 계산 |
| 수정 후       | 29.8 °C  | ✅ 정상                           |                   |


---

### 🔹 Shell 테스트 (`debug.c`)

`diag ntc`
→ `read_ntc_ain1_cx100()` 호출, 결과를 `XX.XX °C` 로 출력.
예시:

```
uart:~$ diag ntc
NTC temperature: 29.84 °C
```

---

### 🔹 향후 개선 예정

| 항목        | 계획                                                    |
| --------- | ----------------------------------------------------- |
| VDD 자동 측정 | SAADC 채널 `NRF_SAADC_INPUT_VDD` (VDD/4) 추가 → 실시간 전압 보정 |
| 보정 상수     | R_pull 실측값(예 9.09 kΩ), NTC 정확 Beta 값 적용               |
| 오프셋 보정    | 25 °C / 60 °C 2 포인트 교정 테이블 적용                         |
| 센서 통합     | 압력 / NTC / SOH → 평균값 계산 및 BLE 패킷 필드 반영                |

---

##  GPIO Shell 테스트 가이드 (`diag gpio`)

###  개요

`diag gpio` 명령은 **BLE Sensor Board (Broadcaster)** 의 주요 전원 및 제어 라인을 UART Shell을 통해 직접 제어하기 위한 디버그 인터페이스

이 기능은 **센서 디버깅 모드**, **전원 시퀀스 검증**, **LED 상태 확인** 등을 펌웨어 수정 없이 UART 콘솔에서 바로 수행할 수 있도록 설계되었다.

---

###  제어 대상 요약

| 대상                | 핀(Pin) | 방향 | 설명              | Active    |
| ----------------- | ------ | -- | --------------- | --------- |
| **RPU Enable**    | P0.31  | 출력 | RPU / DIP 전원 제어 | High = ON |
| **Sensor Enable** | P0.30  | 출력 | 센서 전원 제어        | High = ON |
| **LED**           | P0.11  | 출력 | 상태 표시 LED       | High = ON |

> 모든 라인은 `gpio_if.c` 에서 **GPIO_OUTPUT_INACTIVE (OFF)** 로 초기화됩니다.
> 따라서 전원 인가 후에는 기본적으로 모든 출력이 **OFF 상태**입니다.

---

###  명령 구조

```
diag gpio <target> <action>
```

| 항목         | 설명                       |
| ---------- | ------------------------ |
| `<target>` | `rpu` / `sensor` / `led` |
| `<action>` | `on` / `off`             |

---

###  테스트 절차

#### 1 기본 확인

1. 전원을 넣고 UART 콘솔(115200 8N1) 연결
2. `gpio_if_init()`가 정상 초기화되면 로그에 다음이 표시됩니다:

   ```
   <inf> gpio_if: gpio_if ready: RPU P0.31, SEN P0.30, LED P0.11
   ```

---

#### 2 RPU 전원 제어 테스트

| 명령                  | 설명        | 기대 결과                         |
| ------------------- | --------- | ----------------------------- |
| `diag gpio rpu on`  | RPU 전원 인가 | RPU 보드 구동, 전류 증가 또는 외부 LED ON |
| `diag gpio rpu off` | RPU 전원 차단 | 전류 감소, 보드 전원 꺼짐 확인            |

UART 로그 예시:

```
uart:~$ diag gpio rpu on
gpio rpu on: OK
```

---

#### 3 Sensor 전원 제어 테스트

| 명령                     | 설명       | 기대 결과                   |
| ---------------------- | -------- | ----------------------- |
| `diag gpio sensor on`  | 센서 전원 인가 | 센서(I²C, IMU 등) 정상 응답 시작 |
| `diag gpio sensor off` | 센서 전원 차단 | I²C 스캔 시 응답 사라짐         |

예시:

```
uart:~$ diag gpio sensor on
gpio sensor on: OK

uart:~$ diag i2c-scan
0x20 0x6A
```

---

#### 4 LED 제어 테스트

| 명령                  | 설명        | 기대 결과  |
| ------------------- | --------- | ------ |
| `diag gpio led on`  | 상태 LED 켜기 | LED 점등 |
| `diag gpio led off` | 상태 LED 끄기 | LED 소등 |

예시:

```
uart:~$ diag gpio led on
gpio led on: OK
```

---

#### 5️⃣ 전체 시퀀스 예시

아래는 **센서 디버깅용 시퀀스 예시**입니다.

```
uart:~$ diag ble stop        # BLE 비활성화 (라디오 트래픽 차단)
uart:~$ diag gpio rpu on     # RPU 전원 인가
uart:~$ diag gpio sensor on  # 센서 전원 인가
uart:~$ diag gpio led on     # 상태 LED ON
uart:~$ diag i2c-scan        # I²C 장치 응답 확인
uart:~$ diag ntc             # NTC 온도 확인
```
