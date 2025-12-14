/**
 * @file filter_winsor.c
 * @brief 윈저라이즈드 평균(노이즈 제거)
 */

#include "filter_winsor.h"
#include <string.h>
#include <errno.h>

/* 단순 삽입 정렬 (float 오름차순) */
static void sort_float_asc(float *buf, uint32_t n)
{
    for (uint32_t i = 1; i < n; i++)
    {
        float key = buf[i];
        int32_t j = (int32_t)i - 1;

        while (j >= 0 && buf[j] > key)
        {
            buf[j + 1] = buf[j];
            j--;
        }
        buf[j + 1] = key;
    }
}

int winsor_mean_10f(const float *samples, float *out_mean)
{
    if ((samples == NULL) || (out_mean == NULL))
    {
        return -EINVAL;
    }

    /* 1) 입력 샘플 복사 */
    float buf[WINSOR_SAMPLES_TOTAL];
    memcpy(buf, samples, sizeof(buf));

    /* 2) 오름차순 정렬 */
    sort_float_asc(buf, WINSOR_SAMPLES_TOTAL);

    /* 3) 10개만 사용 (인덱스 0~9) */
    float w[WINSOR_SAMPLES_USED];
    for (uint32_t i = 0; i < WINSOR_SAMPLES_USED; i++)
    {
        w[i] = buf[i];
    }

    /* 4) 윈저라이징(극단값 클램핑)
     *
     *  - 최솟값 2개 (0,1)  → 3번째 값(인덱스 2)로 클램프
     *  - 최댓값 2개 (8,9)  → 8번째 값(인덱스 7)로 클램프
     */
    w[0] = w[2];
    w[1] = w[2];
    w[8] = w[7];
    w[9] = w[7];

    /* 5) 10개 전체 평균 */
    float sum = 0.0f;
    for (uint32_t i = 0; i < WINSOR_SAMPLES_USED; i++)
    {
        sum += w[i];
    }

    *out_mean = sum / (float)WINSOR_SAMPLES_USED;
    return 0;
}
