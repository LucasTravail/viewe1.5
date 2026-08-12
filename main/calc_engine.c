#include "calc_engine.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CALC_MAX_ABS_MM 1.0e6 /* 1km - generously beyond anything a caliper measures, just an overflow guard */
#define MM_PER_INCH 25.4

static void push_history(calc_state_t *st, double a, double b, calc_op_t op, double result)
{
    calc_history_entry_t *e = &st->history[st->history_next];
    e->a = a;
    e->b = b;
    e->op = op;
    e->result = result;

    st->history_next = (st->history_next + 1) % CALC_HISTORY_LEN;
    if (st->history_count < CALC_HISTORY_LEN)
    {
        st->history_count++;
    }
}

static bool finite_and_in_range(double v)
{
    return isfinite(v) && fabs(v) <= CALC_MAX_ABS_MM;
}

static bool compute_binary(calc_op_t op, double a, double b, double *out, calc_err_t *err)
{
    switch (op)
    {
    case CALC_OP_ADD:
        *out = a + b;
        break;
    case CALC_OP_SUB:
        *out = a - b;
        break;
    case CALC_OP_MUL:
        *out = a * b;
        break;
    case CALC_OP_DIV:
        if (b == 0.0)
        {
            *err = CALC_ERR_DIV_ZERO;
            return false;
        }
        *out = a / b;
        break;
    default:
        return false;
    }

    if (!finite_and_in_range(*out))
    {
        *err = CALC_ERR_OVERFLOW;
        return false;
    }
    return true;
}

static void apply_unary(calc_state_t *st, calc_op_t op, double (*fn)(double))
{
    if (!st->has_value)
    {
        st->error = CALC_ERR_NO_VALUE;
        return;
    }

    double a = st->value;
    double result = fn(a);
    if (!finite_and_in_range(result))
    {
        st->error = CALC_ERR_OVERFLOW;
        return;
    }

    push_history(st, a, 0.0, op, result);
    st->value = result;
    st->error = CALC_ERR_NONE;
}

static double half_fn(double a)
{
    return a / 2.0;
}
static double double_fn(double a)
{
    return a * 2.0;
}
static double abs_fn(double a)
{
    return fabs(a);
}
static double neg_fn(double a)
{
    return -a;
}

void calc_init(calc_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->display_round = CALC_ROUND_0_001;
    st->unit = CALC_UNIT_MM;
}

void calc_clear(calc_state_t *st)
{
    st->has_value = false;
    st->value = 0.0;
    st->pending_op = CALC_OP_NONE;
    st->error = CALC_ERR_NONE;
    /* memory & history intentionally kept */
}

void calc_feed_value(calc_state_t *st, double v)
{
    st->error = CALC_ERR_NONE;

    if (!st->has_value)
    {
        st->value = v;
        st->has_value = true;
        return;
    }

    if (st->pending_op != CALC_OP_NONE)
    {
        double a = st->value;
        double result;
        calc_err_t err = CALC_ERR_NONE;

        if (compute_binary(st->pending_op, a, v, &result, &err))
        {
            push_history(st, a, v, st->pending_op, result);
            st->value = result;
        }
        else
        {
            st->error = err; /* keep the old value - a failed op doesn't lose it */
        }
        st->pending_op = CALC_OP_NONE;
        return;
    }

    /* No operator pending: a fresh value simply replaces the old one
     * (matches CAPTURE/manual-entry starting a brand new part). */
    st->value = v;
}

void calc_set_op(calc_state_t *st, calc_op_t op)
{
    if (!st->has_value)
    {
        st->error = CALC_ERR_NO_VALUE;
        return;
    }
    st->error = CALC_ERR_NONE;
    st->pending_op = op;
}

void calc_cancel_op(calc_state_t *st)
{
    st->pending_op = CALC_OP_NONE;
}

void calc_apply_half(calc_state_t *st)
{
    apply_unary(st, CALC_OP_HALF, half_fn);
}
void calc_apply_double(calc_state_t *st)
{
    apply_unary(st, CALC_OP_DOUBLE, double_fn);
}
void calc_apply_abs(calc_state_t *st)
{
    apply_unary(st, CALC_OP_ABS, abs_fn);
}
void calc_toggle_sign(calc_state_t *st)
{
    apply_unary(st, CALC_OP_NEGATE, neg_fn);
}

void calc_memory_add(calc_state_t *st)
{
    if (!st->has_value)
    {
        st->error = CALC_ERR_NO_VALUE;
        return;
    }
    st->memory = st->memory_has_value ? st->memory + st->value : st->value;
    st->memory_has_value = true;
    st->error = CALC_ERR_NONE;
}

void calc_memory_sub(calc_state_t *st)
{
    if (!st->has_value)
    {
        st->error = CALC_ERR_NO_VALUE;
        return;
    }
    st->memory = st->memory_has_value ? st->memory - st->value : -st->value;
    st->memory_has_value = true;
    st->error = CALC_ERR_NONE;
}

void calc_memory_recall(calc_state_t *st)
{
    if (!st->memory_has_value)
    {
        st->error = CALC_ERR_NO_VALUE;
        return;
    }
    calc_feed_value(st, st->memory);
}

void calc_memory_clear(calc_state_t *st)
{
    st->memory = 0.0;
    st->memory_has_value = false;
}

double calc_display_round(double v, calc_round_t r)
{
    double factor;
    switch (r)
    {
    case CALC_ROUND_INT:
        factor = 1.0;
        break;
    case CALC_ROUND_0_1:
        factor = 10.0;
        break;
    case CALC_ROUND_0_01:
        factor = 100.0;
        break;
    case CALC_ROUND_0_001:
    default:
        factor = 1000.0;
        break;
    }
    return round(v * factor) / factor;
}

double calc_mm_to_unit(double mm, calc_unit_t unit)
{
    return (unit == CALC_UNIT_IN) ? (mm / MM_PER_INCH) : mm;
}

double calc_unit_to_mm(double v, calc_unit_t unit)
{
    return (unit == CALC_UNIT_IN) ? (v * MM_PER_INCH) : v;
}

const char *calc_op_symbol(calc_op_t op)
{
    switch (op)
    {
    case CALC_OP_ADD:
        return "+";
    case CALC_OP_SUB:
        return "-";
    case CALC_OP_MUL:
        return "x";
    case CALC_OP_DIV:
        return "/";
    case CALC_OP_HALF:
        return "/2";
    case CALC_OP_DOUBLE:
        return "x2";
    case CALC_OP_ABS:
        return "ABS";
    case CALC_OP_NEGATE:
        return "+/-";
    default:
        return "";
    }
}

void calc_format_value(double mm, calc_unit_t unit, calc_round_t round_mode, char *buf, size_t buf_len)
{
    double v = calc_mm_to_unit(mm, unit);
    double rounded = calc_display_round(v, round_mode);
    int decimals;

    switch (round_mode)
    {
    case CALC_ROUND_INT:
        decimals = 0;
        break;
    case CALC_ROUND_0_1:
        decimals = 1;
        break;
    case CALC_ROUND_0_01:
        decimals = 2;
        break;
    case CALC_ROUND_0_001:
    default:
        decimals = 3;
        break;
    }

    snprintf(buf, buf_len, "%.*f %s", decimals, rounded, (unit == CALC_UNIT_IN) ? "in" : "mm");
}

void calc_format_history_entry(const calc_history_entry_t *e, calc_unit_t unit, calc_round_t round_mode, char *buf,
                                size_t buf_len)
{
    char a_str[24];
    char r_str[24];

    calc_format_value(e->a, unit, round_mode, a_str, sizeof(a_str));
    calc_format_value(e->result, unit, round_mode, r_str, sizeof(r_str));

    if (e->op == CALC_OP_HALF || e->op == CALC_OP_DOUBLE || e->op == CALC_OP_ABS || e->op == CALC_OP_NEGATE)
    {
        snprintf(buf, buf_len, "%s %s = %s", a_str, calc_op_symbol(e->op), r_str);
    }
    else
    {
        char b_str[24];
        calc_format_value(e->b, unit, round_mode, b_str, sizeof(b_str));
        snprintf(buf, buf_len, "%s %s %s = %s", a_str, calc_op_symbol(e->op), b_str, r_str);
    }
}

const calc_history_entry_t *calc_history_get(const calc_state_t *st, int index_from_newest)
{
    if (index_from_newest < 0 || index_from_newest >= st->history_count)
    {
        return NULL;
    }
    int idx = (st->history_next - 1 - index_from_newest + CALC_HISTORY_LEN * 2) % CALC_HISTORY_LEN;
    return &st->history[idx];
}
