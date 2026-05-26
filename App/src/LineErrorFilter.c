#include "LineErrorFilter.h"

static float absf_local(float value)
{
    return (value < 0.0f) ? -value : value;
}

void LineErrorFilter_Reset(LineErrorFilter_t *filter)
{
    filter->last_accepted_error = 0.0f;
    filter->has_accepted_error = 0U;
    filter->jump_ticks = 0U;
}

LineErrorFilter_Result_t LineErrorFilter_Update(LineErrorFilter_t *filter, float raw_error)
{
    LineErrorFilter_Result_t result;

    if (!filter->has_accepted_error) {
        filter->last_accepted_error = raw_error;
        filter->has_accepted_error = 1U;
        filter->jump_ticks = 0U;

        result.error = raw_error;
        result.accepted = 1U;
        result.ignored = 0U;
        return result;
    }

    if (absf_local(raw_error - filter->last_accepted_error) > LINE_ERROR_JUMP_THRESHOLD) {
        if (filter->jump_ticks < LINE_ERROR_JUMP_CONFIRM_TICKS) {
            filter->jump_ticks++;
        }

        if (filter->jump_ticks < LINE_ERROR_JUMP_CONFIRM_TICKS) {
            result.error = filter->last_accepted_error;
            result.accepted = 0U;
            result.ignored = 1U;
            return result;
        }
    } else {
        filter->jump_ticks = 0U;
    }

    filter->last_accepted_error = raw_error;
    filter->jump_ticks = 0U;

    result.error = raw_error;
    result.accepted = 1U;
    result.ignored = 0U;
    return result;
}
