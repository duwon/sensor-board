/** 
button     
	- 1초 미만 리셋
	- 2~10초  10초간 스캔응답 모드  (스캔응답중 LED 1초간격 브링크)
	- 10초 이상 캘리브레이션 (LED 200ms 간격 3회반복)
	
전원 최초 기동시
	- i2c 초기화
	- dip sw 읽음 (이후 wakeup 시는 기존 읽은 정보사용)
	- 센서 상태 정상 시 LED 500ms 1회, 오류 시 500ms 2회

기타
	- 외부 전원시 전원상태를 200% 로 보고
	- bat 는 10분 마다 읽기
	- bat 용량을 17000ma 로,  프로스케일러값을 3으로
	- gpio 안쓰는 포트는 pull down
	- code8 인 경우 scan 응답모드인 경우 phy 를 1M 로 변경
	
	

dip switch 
    1-3 model			1번부터 1,2,4 사용자기준 비트배열
	4   model sub		0=
    5   통신거리		0=1M, 1=code8
    6   송신주기		배터리 (0=30초 1=10초),  어댑터(0=5초, 1=1초)
    7   전원			1=배터리, 0=어댑터
    8   legacy 호환		0=신형, 1=구형
	

dip sw (bit little endian 방식)
0 1 2 3 4 5 6 7  
----- +------------ sub
   +--------------- id

15	1111	inlet	  	XGZP6897D010KPDPN(1000h2o)	0x58
7	1110				SSCDJNN100MD2A3(1020h2o)	0x28

14	0111	outlet		XGZP6897D010KPDPN(1000h2o)	0x58
6	0110				SSCDJNN100MD2A3(1020h2o)	0x28

13	1011	air header	XGZP6847DC001MPGPN -1~10bar 0x6d
5	1010				SSCDJNN010BA2A3 0~10 bar	0x28

12	0011	air flow	XGZP6897D001KPDPN 100mmh2o	0x58
4	0010				SSCDJNN002ND2A3 59,8 mmh2o	0x28

3	110x	ntc			10k ntc
11

10	0101	가속도  4g								0x6a	
2	0100	가속도 16g
	
9	1001	속도	4g								0c6a
1	1000	속도   16g


en_sensor 	보드i2c, 외부i2c, ntc, 가속도
en_dipsw	dipsw,   
			bat칩 상시전원

가속도는 10~1000Hz ms2  의 rms, peak 값으로
속도는   10~1000Hz mmps 의 rms, peak 값으로 
*/

#include <zephyr/bluetooth/bluetooth.h> 
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/watchdog.h> 
#include "gpio_if.h"
#include "dip_switch.h"
#include "ltc3337.h"
#include "sensors.h"
#include "ble_adv.h"
#include "debug.h"
#include "app_diag.h"
#include "sleep_if.h"
#include "xgzp6897d.h"
#include "ntc.h"
#include "lsm6dso.h"
#include "wdt.h" 


LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/**
 * @brief Manufacturer Specific Data를 담는 구조체 변수를 초기화합니다.
 */
static struct sensor_adv_data_t mfg_data = {    // 0xFFFF를 Little Endian으로 변환하여 저장
    .company_id = sys_cpu_to_le16(0xFFFF), 		/* [Index 5-6] Company ID */
    .structure_version = 0x01,             		/* [Index 7] Version v1 */
    .model_code = 0x01,                    		/* [Index 8] Model Code (Inlet 압력) */

    // 상태 관리 정보
    .device_status = 0x00,       				/* [Index 9] Device Status (정상) */
    .error_info = 0x00,          				/* [Index 10] Error Info */
    .mcu_temperature = 25,       				/* [Index 11] MCU Temperature (예: 25°C) */
    .battery_percent = 95,       				/* [Index 12] Battery % (예: 95%) */
    .value_presence_mask = 0x01, 				/* [Index 13] Inlet 압력은 Sensor Value 1만 사용 (0b00000001) */

    // 센서 값 (압력 값 10.25 mmH2O를 100배하여 1025로 변환)
    .sensor_value_1 = sys_cpu_to_le32(1025), 	/* [Index 14-17] Sensor Value 1 (압력 값) */

    // 나머지 미사용 값은 0x00으로 설정됨
    .sensor_value_2 = 0, /* [Index 18-21] 미사용 */
    .sensor_value_3 = 0, /* [Index 22-25] 미사용 */
    .sensor_value_4 = 0, /* [Index 26-29] 미사용 */
    .sensor_value_5 = 0, /* [Index 30-33] 미사용 */
    .sensor_value_6 = 0, /* [Index 34-37] 미사용 */
};

struct k_work_delayable loop_work;
static struct k_work_delayable led_work;
static struct k_work_delayable adv_stop_work;
struct Status Stat = {0, 2, 500, 2, false, 0};
struct flash_data cfg;					// bat 측정값(10분간격 확인), 가속도 캘리브레이션 값

uint64_t wakeup_time;

int flash_init(void);
int flash_read_data();
int flash_write_data();
void Set_Value ();
void get_sensor_data();
void led_blink (int slp);
static void adv_stop_fn(struct k_work *work);
void set_led_err();
/* --------------------------------- 디버그용 --------------------------------- */
void debug_run_code(void)
{
    // while (1) {
    power_sensor(true);
    k_sleep(K_MSEC(10)); // 센서 안정화

    float pressure_pa = 0.0f;
    float temperature_c = 0.0f;

    /* TODO: 사용 중인 센서 타입으로 바꿔서 테스트
     * - XGZP6897D001KPDPN : XGZP6897_RANGE_001K
     * - XGZP6897D010KPDPN : XGZP6897_RANGE_010K
     */
    xgzp6897_range_t range = XGZP6897_RANGE_001K; /* 필요 시 010K로 변경 */

    int ret = read_xgzp6897_filtered(range, &pressure_pa, &temperature_c, true);
    if (ret == 0)
    {
        /* Pa → mmH2O 로도 같이 보고 싶으면 아래 변환 사용 */
        float pressure_mmH2O = pressure_pa / 9.80665f;

        LOG_INF("XGZP6897D: P = %.3f Pa (%.3f mmH2O), T = %.2f C", (double)pressure_pa, (double)pressure_mmH2O, (double)temperature_c);
    }
    else
    {
        LOG_ERR("xgzp6897_read_measurement failed, err=%d", ret);
    }

    /* 1초 대기 */
    k_sleep(K_SECONDS(1));
    // }
}
//--------------------------------------------------------------------------
static void led_fn(struct k_work *w)
{
    if (app_is_hold()) // BLE 일시정지 상태, 디버깅용
    {
		k_work_reschedule(&led_work, K_MSEC(50));
        return;
    }

	if (Stat.Led_Cnt) 
		{
		--Stat.Led_Cnt;
		k_work_reschedule(&led_work, K_MSEC(Stat.Led_Interval));
		if (Stat.Led_Cnt % 2)
			board_led_set(1);
		else
			board_led_set(0);
		}
	else
		{
		if (Stat.Complete)
			{
			k_work_schedule(&loop_work, K_SECONDS(Stat.Sleep_Sec));
 			Start_Sleep(Stat.Sleep_Sec);
			}
		else
			{
			k_work_reschedule(&led_work, K_MSEC(50));
			}
		}
}
/**
 7 6 5  4   3 2 1 0 
 -----  -   - 
 765  	model 
 4		sub       0=
 3		phy		  0=code8,  1=1M	
 2		interval  Bat 1=30, 0=10,  AC 1=5, 0=1
 1		power	  0=AC,  1=Bat	
 0		legacy
 


1000  속소16g
0100  가속도16
1100  ntc
0010  SSCDJNN002ND2A3
1010  SSCDJNN010BA2A3
0110  SSCDJNN100MD2A3
1110  SSCDJNN100MD2A3
1001  속도4
0101  가속도4
0011  XGZP6897D001KPDPN
1011  XGZP6847DC001MPGPN
0111  XGZP6897D010KPDPN
1111  XGZP6897D010KPDPN

 
 
 */
//---------------------------------------------------------------------- 
static void loop_fn(struct k_work *w)
{
uint8_t pass=false, scan=false;
static uint16_t  Bat_Timer = 0;

	wdt_feed_dog();     /* ← 루프 진입 즉시 feed (가장 중요) */

	wakeup_time = k_uptime_get();
	printk("\r\n\r\n%llu Wakeup ID=%u \r\n", wakeup_time, Stat.Dipsw & 0x0f);    

	Wakeup();

	btn_evt_t btn = Get_BtnStatus();
	
	Stat.Complete = false;
	
    switch (btn)
		{
		case BTN_EVT_LONG:						// 10초 이상
			printk("\r\nLong button -> Calibration");
			Stat.Led_Interval = 250;			// 캘리브레이션 
			Stat.Led_Cnt = 6;					// 3회 브링크
			Set_Calibration (Stat.Dipsw & 0x0f);
			pass = true;
			break;
		
		case BTN_EVT_SHORT:						// 2~10초 미만
			printk("\r\nShort button -> Scan Start");
			Stat.Led_Interval = 1000;			// 스캔 중 1초간격
			Stat.Led_Cnt = 10;					// 5회 브링크
			scan = true;						// scan 동작
			break;

		default:
			Stat.Led_Interval = 50;
			Stat.Led_Cnt = 2;					// 1회 브링크
			break;
		}
	
	k_work_reschedule(&led_work, K_MSEC(10));		// 바로 실행

	ble_setup ((Stat.Dipsw&0x10)>>4, scan);		// phy 0=code8, 1=1M

	if (pass || scan) goto Sleep_start;

    get_sensor_data();
	
//    debug_run_code();

    if (!app_is_hold()) // BLE 정지, 디버깅용
		{
		Tx_Ble((const uint8_t *)&mfg_data, sizeof(mfg_data));
		 k_work_reschedule(&adv_stop_work, K_SECONDS(5));
		}

	Bat_Timer += Stat.Sleep_Sec;
	if (Bat_Timer > 30 && Stat.Dipsw & 0x40)	// 10분 마다 Bat 확인  (Bat 동작)
		{
		printk ("Bat Checking\r\n");
		Bat_Timer = 0;
		Stat.Led_Cnt = 0;							// 바로 sleep 진입토록
		mfg_data.battery_percent = Bat_Percent();	// 보고용 배터리 잔량계산	

		if (mfg_data.battery_percent <= 10)
		mfg_data.device_status |= 0x02;				// 베터리 10% 미만
		}

Sleep_start:
	Stat.Complete = true;
}
//--------------------------------------------------------------------
// 15회 전송후 자동종료해야하는 전송 시작시간이 늦어지면 중지가 안되 4초후 중지
static void adv_stop_fn(struct k_work *work)
{
    Ble_Stop ();				
}
//-------------------------------------------------------------------
int main(void)
{
    
	NRF_POWER->DCDCEN = 1;		// DCDC 기능 사용  항상on 
	Stat.Led_Interval = 500;	// 처음 led 정상은 500 msec 1회
	wdt_init();
	
	if (board_gpio_init())
		{
		printk("Err gpio\n");
		set_led_err();
		}
   
	if (Get_Switch(&Stat.Dipsw, NULL))
		{
		printk("Err Dip Switch\n");
		set_led_err();
		}

    if (Init_Sensor())
		{
		printk("Err init_Sensor\n");
		set_led_err();
		}

	if (ltc3337_init ())
		{
//		printk("Err LTC1337 init\n");
//		set_led_err();
		}
	
	if (bt_enable(NULL))  
		{
		printk("Err bt_enable\n");
		set_led_err();
		}
    	
    if (ble_adv_ext_init())
		{
		printk("Err ble_adv_ext_init\n");
		set_led_err();
		}
    	
	k_work_init_delayable(&adv_stop_work, adv_stop_fn);		
//	flash_init();					
//  flash_read_data();					// config Read

	Set_Value();						// dipsw 값으로 광고정보 초기화
		
    /* 디버깅 코드 실행 */
//    debug_run_startup();


	get_sensor_data();					// 센서가 정상인지 감지하여 LED 표시
	if (mfg_data.device_status == 0x01) 
		{	
		printk("Sensor Err\n");
		set_led_err();
		}


    k_work_init_delayable(&led_work, led_fn);
    k_work_schedule(&led_work, K_MSEC(200));

    k_work_init_delayable(&loop_work, loop_fn);
    k_work_schedule(&loop_work, K_SECONDS(2));
	return 0;
}
//--------------------------------------------------------------------------
void set_led_err()
{
	Stat.Led_Interval = 200;
	Stat.Led_Cnt = 4;
}



/*--------------------------------------------------------------------------
dip sw (bit little endian 방식)
0 1 2 3 4 5 6 7  
----- +------------ sub
   +--------------- id

15	1111	inlet	  	XGZP6897D010KPDPN(1000h2o)	0x58
7	1110				SSCDJNN100MD2A3(1020h2o)	0x28

14	0111	outlet		XGZP6897D010KPDPN(1000h2o)	0x58
6	0110				SSCDJNN100MD2A3(1020h2o)	0x28

13	1011	air header	XGZP6847DC001MPGPN -1~10bar 0x6d
5	1010				SSCDJNN010BA2A3 0~10 bar	0x28

12	0011	air flow	XGZP6897D001KPDPN 100mmh2o	0x58
4	0010				SSCDJNN002ND2A3 59,8 mmh2o	0x28

3	110x	ntc			10k ntc
11

10	0101	가속도  4g								0x6a	
2	0100	가속도 16g
	
9	1001	속도	4g								0c6a
1	1000	속도   16g


extern lsm6dso_stats_t g_lsm6dso_stats;
//----------------------- Sensor & Main Loop ---------------------------------*/
void get_sensor_data()
{
static uint8_t check_cnt = 10;

	mfg_data.value_presence_mask = 0x01;					// 어떤값이 유효한지 디폴

	switch (Stat.Dipsw & 0x0f)
		{
		case 1  :										// 속도 16g
		case 9  :										// 속도 4g
			lsm6dso_init();
			if (Get_Imu_Value (Stat.Dipsw & 0x0f) < 0)
				mfg_data.sensor_value_1 = -1;
			else
				{
				mfg_data.sensor_value_1 = g_lsm6dso_stats.bl_peak_mmps_x100[0];
				mfg_data.sensor_value_2 = g_lsm6dso_stats.bl_peak_mmps_x100[1];
				mfg_data.sensor_value_3 = g_lsm6dso_stats.bl_peak_mmps_x100[2];
				mfg_data.sensor_value_4 = g_lsm6dso_stats.bl_rms_mmps_x100[0];
				mfg_data.sensor_value_5 = g_lsm6dso_stats.bl_rms_mmps_x100[1];
				mfg_data.sensor_value_6 = g_lsm6dso_stats.bl_rms_mmps_x100[2];
				mfg_data.value_presence_mask = 0x3f;
				}
			
			printk ("Vel-%s Peak=%d,%d,%d  RMS=%d,%d,%d\n", 
			((Stat.Dipsw&0x0f) == 9) ? "4g" : "16g", 
			mfg_data.sensor_value_1,mfg_data.sensor_value_2,mfg_data.sensor_value_3,
			mfg_data.sensor_value_4,mfg_data.sensor_value_5,mfg_data.sensor_value_6);
			break;

		case 2  :										// 가속도 16g
		case 10 :										// 가속도 4g
			lsm6dso_init();
			if (Get_Imu_Value (Stat.Dipsw & 0x0f) < 0)
				mfg_data.sensor_value_1 = -1;
			else
				{
				mfg_data.sensor_value_1 = g_lsm6dso_stats.bl_peak_ms2_x100[0];
				mfg_data.sensor_value_2 = g_lsm6dso_stats.bl_peak_ms2_x100[1];
				mfg_data.sensor_value_3 = g_lsm6dso_stats.bl_peak_ms2_x100[2];
				mfg_data.sensor_value_4 = g_lsm6dso_stats.bl_rms_ms2_x100[0];
				mfg_data.sensor_value_5 = g_lsm6dso_stats.bl_rms_ms2_x100[1];
				mfg_data.sensor_value_6 = g_lsm6dso_stats.bl_rms_ms2_x100[2];
				mfg_data.value_presence_mask = 0x3f;
				}
			printk ("Acc-%s Peak=%d,%d,%d  RMS=%d,%d,%d\n", 
			((Stat.Dipsw&0x0f) == 10) ? "4g" : "16g", 
			mfg_data.sensor_value_1,mfg_data.sensor_value_2,mfg_data.sensor_value_3,
			mfg_data.sensor_value_4,mfg_data.sensor_value_5,mfg_data.sensor_value_6);
			break;

		case 3	:										// ntc
		case 11	:
			mfg_data.sensor_value_1 = Get_Sensor_Value(3);
			printk ("temp = %d\n", mfg_data.sensor_value_1);
			break;
		case 4  :										// air flow SSCDJNN002ND2A3 59,8 mmh2o		0x28
			mfg_data.sensor_value_1 = Get_Sensor_Value(4);
			break;
		case 12 :										// air flow XGZP6897D001KPDPN 100mmh2o		0x58
			mfg_data.sensor_value_1 = Get_Sensor_Value(12);
			break;
		case 5  :										// air header SSCDJNN010BA2A3 0~10 bar		0x28
			mfg_data.sensor_value_1 = Get_Sensor_Value(5);
			break;
		case 13 :										// air header XGZP6847DC001MPGPN -1~10bar 	0x6d
			mfg_data.sensor_value_1 = Get_Sensor_Value(13);
			break;
		case 6  :										// outlet SSCDJNN100MD2A3(1020h2o)			0x28
			mfg_data.sensor_value_1 = Get_Sensor_Value(6);
			break;
		case 14 :										// outlet XGZP6897D010KPDPN(1000h2o)		0x58
			mfg_data.sensor_value_1 = Get_Sensor_Value(14);
			break;
		case 7  :										// inlet  SSCDJNN100MD2A3(1020h2o)			0x28
			mfg_data.sensor_value_1 = Get_Sensor_Value(7);
			break;
		case 15 :										// inlet  XGZP6897D010KPDPN(1000h2o)		0x58
			mfg_data.sensor_value_1 = Get_Sensor_Value(15);		
			break;
			
		default :
			mfg_data.value_presence_mask = 0x00;					// 없으면 에러로		
		}
	
/*
bit 0 	wdt 리셋발생시
bit 1   배터리 10% 이하
bit 2   배터리 칩셋에러
bit 3   센서연결불량
bit 4   과열 80도 이상
*/
	if (mfg_data.sensor_value_1 < 0)
		mfg_data.device_status = 0x08;			// 센서에러 
	else	
		mfg_data.device_status = 0x00;			// 정상
	
	mfg_data.error_info    = 0x00;							// Error info
  
	if (++check_cnt > 10)
		{
		check_cnt = 0;
		Get_MCU_Temperature (&mfg_data.mcu_temperature);  	// MCU 온도 및 배터리 업데이트 (8-bit)
		
		if (mfg_data.mcu_temperature >= 80)
			mfg_data.device_status |= 0x10;			// 과열
		}
}

//  0000 1 sec
//  0100 5	
//  0010 10 
//  0110 30 
//    +------------ power 0=ac 1=bat
//-------------------------------------------------------------------------------------
void Set_Value ()
{
uint8_t v=0;

	switch (Stat.Dipsw & 0x60)
		{
		case 0x00 : Stat.Sleep_Sec = 1; break;	
		case 0x20 : Stat.Sleep_Sec = 5; break;	
		case 0x40 : Stat.Sleep_Sec = 10; break;
		case 0x60 : Stat.Sleep_Sec = 30; break;
		}

	switch (Stat.Dipsw & 0x0f)
		{
		case 1  :	v=0x71;  break;					// 속도 16g
		case 9  :	v=0x70;  break;					// 속도 4g
		case 2  :	v=0x61;  break;					// 가속도 16g
		case 10 :	v=0x60;  break;					// 가속도 4g
		case 3	:	
		case 11	:	v=0x50;  break;	
		case 4  :	v=0x41;  break;					// air flow SSCDJNN002ND2A3 59,8 mmh2o		0x28
		case 12 :	v=0x40;  break;					// air flow XGZP6897D001KPDPN 100mmh2o		0x58
		case 5  :	v=0x31;  break;					// air header SSCDJNN010BA2A3 0~10 bar		0x28
		case 13 :	v=0x30;  break;					// air header XGZP6847DC001MPGPN -1~10bar 	0x6d
		case 6  :	v=0x21;  break;					// outlet SSCDJNN100MD2A3(1020h2o)			0x28
		case 14 :	v=0x20;  break;					// outlet XGZP6897D010KPDPN(1000h2o)		0x58
		case 7  :	v=0x11;  break;					// inlet  SSCDJNN100MD2A3(1020h2o)			0x28
		case 15 :	v=0x10;  break;					// inlet  XGZP6897D010KPDPN(1000h2o)		0x58
		}		
	Stat.Model = v;
	mfg_data.model_code = v;
	mfg_data.structure_version = 0x01;

	if (Stat.Dipsw & 0x40)
		mfg_data.battery_percent = cfg.bat_value;
	else	
		mfg_data.battery_percent = 200;		// ac 면 200%
}




