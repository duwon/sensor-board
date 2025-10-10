

# 📘 BLE Sensor Board (Broadcaster) 정리 요약

## 1️⃣ MCU 및 기본 설정

* **MCU:** nRF52833
* **클럭:** 외부 HFXO(32 MHz) + LFXO(32.768 kHz) 사용
* **SDK:** nRF Connect SDK v3.1.0 (Zephyr v4.1.99 기반)
* **보드:** 커스텀 센서보드 (임시 타깃: `nrf52833dk_nrf52833`)

---

## 2️⃣ 핀맵 요약

| 기능              | 핀             | 방향    | 설명                                 |
| --------------- | ------------- | ----- | ---------------------------------- |
| LFXO            | P0.00 / P0.01 | —     | 32.768 kHz 크리스털                    |
| **I2C_SDA/SCL** | P0.04 / P0.05 | I/O   | I²C Bus0 (센서공통)                    |
| **UART_TX/RX**  | P0.15 / P0.17 | 출력/입력 | 디버깅용 UART0 (115200 8N1)            |
| **LED**         | P0.11         | 출력    | 상태 LED (Active High)               |
| **BTN**         | P1.09         | 입력    | 사용자 버튼 (Active Low)                |
| **NTC ADC**     | P0.03         | 입력    | SAADC AIN1 (온도센서)                  |
| **EN_SENSOR**   | P0.30         | 출력    | 센서전원 Enable (High = On)            |
| **EN_RPU**      | P0.31         | 출력    | RPU전원 Enable (High = On)           |
| **IO_INT**      | P0.20         | 출력    | 외부 센서 인터럽트 출력 (Active Low, 기본 Low) |
| **SOH_ALARM**   | P0.09         | 입력    | LTC3337 ALARM (Active High)        |
| **SOH_OK**      | P0.10         | 입력    | LTC3337 OK (Active High)           |
| **RESET**       | P0.18         | 입력    | JTAG Reset 핀                       |

---

## 3️⃣ I²C 버스 연결 구조

**I2C0 (P0.04 / P0.05)** 에 다음 4개 디바이스가 연결됨:

| 디바이스                   | 주소        | 역할                                    |
| ---------------------- | --------- | ------------------------------------- |
| **TCA9534**            | 0x20      | DIP 스위치 입력 (I/O Expander)             |
| **LTC3337**            | 0x64 (예정) | 배터리 SOH 모니터 (Alarm/OK 핀 P0.09/10 연결)  |
| **LSM6DSO**            | 0x6A      | 가속도계 (2-wire I²C Only)                |
| **Honeywell SSC 압력센서** | 0x28      | 6 종 중 1 개 장착 (010BA/100MD/002ND 중 선택) |

---

## 4️⃣ DIP 스위치 비트 정의 ( TCA9534 )

| 비트    | 의미             | 1 (H)       | 0 (L)         |
| ----- | -------------- | ----------- | ------------- |
| [2:0] | Model(Main)    | 모델 선택 비트    | —             |
| [3]   | Mode(Sub)      | Sub Mode On | Off           |
| [4]   | PHY            | 1 Mbps      | Coded PHY S=8 |
| [5]   | Period         | Long        | Short         |
| [6]   | PWR            | Battery     | Adapter       |
| [7]   | Legacy Support | New 패킷      | Legacy 패킷     |

---

## 5️⃣ GPIO 기본 동작

* `EN_SENSOR`, `EN_RPU` → 부팅 시 자동 High 출력 (GPIO hog)
* `IO_INT` → 기본 Low (필요시 펄스 출력 가능)
* `LED` → Active High 출력, `BTN` → Active Low 입력
* `SOH_ALARM`, `SOH_OK` → Active High 입력
* NTC → AIN1( P0.03 ) ADC 입력

---

## 6️⃣ 프로젝트 구조 ( sensor_board_fw.zip )

```
sensor_board_fw/
├── boards/
│   └── nrf52833dk_nrf52833.overlay    ← 실보드 핀매핑
├── src/
│   ├── gpio_if.c      ← LED/BTN/IO_INT/SOH GPIO 제어
│   ├── dip_switch.c   ← TCA9534 DIP 읽기 및 비트 해석
│   └── main.c         ← (아직 작성 예정) 센서 루프 + BLE 광고
├── prj.conf           ← 모듈 및 로그 설정
└── CMakeLists.txt     ← Zephyr 빌드 설정
```

---

## 7️⃣ 소프트웨어 기능 (현재까지 구현)

* ✅ GPIO 초기화 및 제어 (`board_gpio_init`, `io_int_pulse_us`)
* ✅ DIP 스위치 읽기 (`dip_read_u8`) + 비트 해석 (`parse_dip`)
* ✅ I²C 기반 디바이스 바인딩 (압력센서, IMU, LTC3337 , TCA9534)
* ✅ ADC 채널 활성화 (NTC 센서)
* ✅ UART0 디버깅 콘솔 (115200 8N1)
* 🕒 다음 단계 : 센서 데이터 수집 루프 및 BLE 광고 (레거시 → 확장 전환)

---

## 8️⃣ 다음 할 일

1. `sensors.c` 작성 → 압력(SSC 0x28), NTC, LSM6DSO, LTC3337 데이터 수집 함수 구현
2. `main.c` 작성 → 센서 루프 + BLE 레거시 광고 10 초 주기 갱신
3. DIP 비트 [4] PHY, [7] Legacy 에 따라 확장 광고 38B 전환 로직 추가
4. LTC3337 데이터시트 기반 SOH 상태 레지스터 해석 및 전송 필드 정의





### 🔧 다음 개발 연속 작업

#### 1️⃣ `sensors.c`

* **압력센서 (I2C 0x28)** : `SSCDJNN010BA2A3 / 100MD / 002ND`
* **온도센서 (NTC)** : `ADC AIN1`
* **IMU (LSM6DSO, 0x6A)** : 가속도 3축
* **LTC3337 (0x64)** : SOH 레지스터 읽기
* 모든 센서는 Sleep → Wake 시 전원 Enable → 초기화 후 평균 샘플 계산

#### 2️⃣ `main.c`

* DIP 스위치 해석 결과로 BLE 광고 모드 전환
  (`[7] Legacy Support`, `[4] PHY`, `[5] Period` 등)
* 10초 주기 센싱 루프 + BLE 광고 데이터 업데이트
* LTC3337 배터리 SOH → BLE 패킷 필드 반영

#### 3️⃣ BLE 패킷 포맷

* **기본(신형)** : 확장 광고 38 B
* **호환(구형)** : 레거시 광고 23 B
* 구조는 개발사양서 § 2.4.1 기준 (Index 9–37 구간)

#### 4️⃣ 캘리브레이션 및 평균화

* 압력: offset 보정 + 윈저라이즈드 평균
* NTC: B3435 변환식
* 진동: FIFO 기반 RMS/Peak 계산 루틴 (후속 단계에서 확장 가능)

