#pragma once
#include <stdint.h>

/* 시스템 내 인터페이스 전원 다운 후 x초 뒤 깨어나기 */
int Start_Sleep(uint32_t seconds);

/* 깨어난 뒤 인터페이스(전원/광고 등) 재기동 */
int Wakeup(void);
