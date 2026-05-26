#ifndef _LINE_ERROR_FILTER_H_
#define _LINE_ERROR_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LINE_ERROR_JUMP_THRESHOLD      4.0f
#define LINE_ERROR_JUMP_CONFIRM_TICKS  2U

typedef struct {
    float last_accepted_error;
    uint8_t has_accepted_error;
    uint8_t jump_ticks;
} LineErrorFilter_t;

typedef struct {
    float error;
    uint8_t accepted;
    uint8_t ignored;
} LineErrorFilter_Result_t;

void LineErrorFilter_Reset(LineErrorFilter_t *filter);
LineErrorFilter_Result_t LineErrorFilter_Update(LineErrorFilter_t *filter, float raw_error);

#ifdef __cplusplus
}
#endif

#endif
