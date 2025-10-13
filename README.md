
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

## 8️⃣ **BLE 스택/광고 개선 사항 (오늘 반영)**

* **MAC 주소 고정:**
  `settings_load()`로 `app/ble_id` 로드 → 없으면 **첫 부팅 시 현 주소 저장** → `bt_id_reset(0, stored)`로 **항상 동일 주소 사용**.
* **Device Type(appearance) 노출:**
  AD에 **GAP Appearance = Generic Sensor (0x0340)** 추가 → 스캐너에서 **Device type 표기**.
* **스캔 응답(Rx_Ble):**
  `Rx_Ble()`로 **Scan Response(최대 31B)** 등록 가능(예: Shortened Name), 레거시/EXT 각각 방식에 맞게 반영.
* **초기화/시작 정책:**

  * `Init_Ble(cfg)`: **“현재 모드만”** stop 후 파라미터 반영(EXT는 핸들 준비까지만).
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

## 1️⃣2️⃣ 내일 할 일

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
