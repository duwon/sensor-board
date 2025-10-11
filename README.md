# 📘 BLE Sensor Board (Broadcaster) 최신 정리 (업데이트)

## 1️⃣ MCU 및 기본 설정

* **MCU:** nRF52833
* **클럭:** HFXO 32 MHz + LFXO 32.768 kHz
* **SDK:** nRF Connect SDK v3.1.0 (Zephyr v4.1.99)
* **보드 타깃:** `nrf52833dk_nrf52833` (커스텀 보드 핀 오버레이 적용)

---

## 2️⃣ 핀맵 요약 (확정)

| 기능               | 핀                 | 방향    | 설명                              |
| ---------------- | ----------------- | ----- | ------------------------------- |
| LFXO             | P0.00 / P0.01     | —     | 32.768 kHz                      |
| **I2C SCL/SDA**  | **P0.05 / P0.04** | I/O   | I²C0 (400 kHz, 필요시 100 kHz로 하향) |
| **UART TX/RX**   | **P0.15 / P0.17** | 출력/입력 | UART0 115200 8N1 (**HWFC 끔**)   |
| **LED**          | P0.11             | 출력    | 상태 LED (Active High)            |
| **BTN**          | P1.09             | 입력    | 사용자 버튼 (Active Low)             |
| **NTC ADC**      | P0.03             | 입력    | SAADC AIN1                      |
| **EN_SENSOR**    | P0.30             | 출력    | 센서 전원 Enable                    |
| **EN_RPU**       | P0.31             | 출력    | RPU 전원 Enable (DIP 전원)          |
| **IO_INT**       | P0.20             | 출력    | 센서 인터럽트                         |
| **SOH_ALARM/OK** | P0.09 / P0.10     | 입력    | LTC3337 상태                      |
| **RESET**        | P0.18             | 입력    | JTAG Reset                      |

> **주의(하드 이슈 기록):** 압력센서(0x28)가 I²C 라인을 1.8 V로 끌어내리던 문제 발견. 제거 시 3 V 정상화 및 0x20(TCA9534), 0x6A(LSM6DSO) 응답 확인. 0x28 사용 시 **3 V 통일** 또는 **양방향 I²C 레벨시프터** 필요.

---

## 3️⃣ I²C 버스 구성

| 디바이스              | 주소       | 역할                        |
| ----------------- | -------- | ------------------------- |
| **TCA9534**       | 0x20     | DIP 스위치 입력 (I/O Expander) |
| **LTC3337**       | 0x64(예정) | 배터리 SOH                   |
| **LSM6DSO**       | 0x6A     | IMU                       |
| **Honeywell SSC** | 0x28     | 압력센서(모델 가변, **전압 유의**)    |

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

## 5️⃣ DeviceTree 오버레이 핵심

* **UART0**: TX=P0.15, RX=P0.17, **hw-flow-control 속성 제거**
* **I2C0**: SCL=P0.05, SDA=P0.04, 기본 **Fast(400 kHz)**
* pinctrl 라벨 충돌 방지 위해 `uart0_my_default/sleep` 사용, `&uart0`의 `pinctrl-0/1`로 **명시 교체**
* (선택) `chosen { zephyr,console = &uart0; zephyr,shell-uart = &uart0; }`

---

## 6️⃣ Kconfig (prj.conf) 핵심

```conf
# UART/Console
CONFIG_SERIAL=y
CONFIG_UART_CONSOLE=y
CONFIG_PRINTK=y
CONFIG_CONSOLE=y
CONFIG_UART_0_NRF_HWFC=n   # HWFC 끔

# Shell
CONFIG_SHELL=y
CONFIG_SHELL_BACKEND_SERIAL=y
CONFIG_SHELL_STACK_SIZE=4096

# Logging
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_LOG_RUNTIME_FILTERING=y
CONFIG_LOG_PROCESS_THREAD_STACK_SIZE=2048
```

---

## 7️⃣ 프로젝트 구조 (업데이트)

```
sensor_board_fw/
├── boards/
│   └── nrf52833dk_nrf52833.overlay        # 확정 핀매핑(USB-UART RX=0.17)
├── include/
│   └── debug.h, dip_switch.h, ...
├── src/
│   ├── main.c                             # 센서 루프 + BLE 광고 + LED HB + 로그스위치
│   ├── gpio_if.c                          # LED/BTN/IO_INT/SOH
│   ├── dip_switch.c                       # (통합) Get_Switch() + 파싱
│   ├── sensors.c                          # 센서 초기화/샘플 (진행중)
│   ├── ltc3337.c                          # SOH(진행중)
│   ├── ble_adv.c                          # EXT Adv 기본, Legacy 옵션
│   └── debug.c                            # 쉘 명령(diag …), I2C 스캔/IMU/DIP/로그 on/off
├── prj.conf
└── CMakeLists.txt
```

---

## 8️⃣ 구현 현황 (오늘 반영 포함)

* ✅ **UART RX 문제 해결**: pinctrl 재정의, HWFC 비활성.
* ✅ **Shell 모드 정상화**: `diag` 명령 사용 가능.
* ✅ **디버그 쉘 명령**

  * `diag echo <text…>`: UART RX 에코
  * `diag i2c-scan`: 0x03–0x77 스캔
  * `diag imu-who`: LSM6DSO WHO_AM_I 확인
  * `diag dip-read`: EN_RPU on → TCA9534 입력 읽기/파싱
  * `diag log [on|off]`: **주기 로그 on/off 스위치**
* ✅ **DIP 읽기 절차 통합**: `dip_switch.c` 안에 `Get_Switch()` 포함(EN_RPU on → 2회 읽기 디바운스 → 파싱)
* ✅ **LED Heartbeat**: ok=0.5 s 토글, 에러=120 ms 토글
* ✅ **메인 루프**: DIP period(5 s/10 s)로 주기 스케줄, 광고 페이로드 갱신 로그
* ✅ **BLE 광고**: 확장 광고(Non-connectable) 100 ms, payload 업데이트 로그. 스캐너에서 보이도록 개선(필드 구성 정리됨)

---

## 9️⃣ 빌드/플래시 & 런북

```bash
# 클린 빌드 권장
west build -b nrf52833dk_nrf52833 . --domain sensor-board -p
west flash
```

터미널(115200 8N1, flow=OFF, CR 또는 CR+LF):

```
uart:~$ help
uart:~$ diag dip-read
uart:~$ diag i2c-scan
uart:~$ diag log off   # 루프 로그 정지
uart:~$ diag log on    # 루프 로그 재개
```

---

## 🔟 내일 진행할 “바로 다음” 작업

1. **sensors.c 마무리**

   * 압력(0x28) 복귀 전략: 3 V 통일 또는 레벨시프터 경유 시퀀스 반영
   * NTC 변환(B=3435) 함수 적용, 온도/압력 평균화(윈저라이즈드)
   * LSM6DSO 기본 init + Acc 3축 샘플(저속)
2. **LTC3337 SOH 읽기**: OK/ALARM 핀 반영 + I²C 레지스터 맵 정의/적용
3. **BLE 패킷 포맷 확정**

   * 기본 38 B(신형), `DIP[7]=1`일 때 Legacy 23 B
   * `DIP[4]`에 따라 1 Mbps vs Coded S=8 선택 적용
4. **옵션 B(스캐너 호환 개선)**: 스캐너 인식률 저하 시 Legacy + Flags-only AD fallback 스위치 추가
5. (선택) **diag 확장**: `diag ble start/stop`, `diag phy 1m|coded`, `diag period 5|10` 등

