/**
 * @file filter_winsor.h
 * @brief 윈저라이즈드 평균(노이즈 제거)
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 윈저라이즈드 평균 기본 설정 */
#define WINSOR_SAMPLES_TOTAL  10U  /**< 전체 샘플 수 */
#define WINSOR_SAMPLES_USED   10U  /**< 정렬 후 사용하는 샘플 수 */

/**
 * @brief 10개 샘플에 대해 윈저라이즈드 평균을 계산한다.
 *
 * 알고리즘:
 *  - 입력 samples[0..10]를 오름차순 정렬
 *  - 하위 10개(0~9 인덱스)를 대상으로:
 *      - 최솟값 2개(0,1)  → 3번째 값(인덱스 2)로 클램프
 *      - 최댓값 2개(8,9)  → 8번째 값(인덱스 7)로 클램프
 *  - 클램프된 10개(0~9)의 전체 평균을 반환
 *
 * @param[in]  samples  길이 10의 float 배열 (입력값은 정렬 과정에서 내부 복사본 사용)
 * @param[out] out_mean 계산된 윈저라이즈드 평균 값 (NULL 허용하지 않음)
 *
 * @retval 0        성공
 * @retval -EINVAL  파라미터 오류 (out_mean == NULL)
 */
int winsor_mean_10f(const float *samples, float *out_mean);

#ifdef __cplusplus
}
#endif
