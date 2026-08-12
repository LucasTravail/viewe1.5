#ifndef CALC_ENGINE_H
#define CALC_ENGINE_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Pure calculation engine for the "Calculatrice de cotes" app - no LVGL or
 * any UI dependency, so it stays reusable (and testable) on its own.
 *
 * Values are always stored in millimeters, at full `double` precision.
 * Rounding (calc_display_round / calc_format_value) is purely a display
 * concern: it never mutates calc_state_t::value, so chained calculations
 * always use the real captured precision, never an already-rounded one. */

typedef enum
{
    CALC_OP_NONE = 0, /* no pending binary operator */
    CALC_OP_ADD,
    CALC_OP_SUB,
    CALC_OP_MUL,
    CALC_OP_DIV,
    /* Unary ops: only ever appear as a history entry's `op` (never as a
     * *pending* operator) - they apply immediately, no 2nd operand needed. */
    CALC_OP_HALF,
    CALC_OP_DOUBLE,
    CALC_OP_ABS,
    CALC_OP_NEGATE,
} calc_op_t;

typedef enum
{
    CALC_ERR_NONE = 0,
    CALC_ERR_DIV_ZERO,   /* "Division impossible" */
    CALC_ERR_OVERFLOW,   /* result outside any sane mechanical range */
    CALC_ERR_NO_VALUE,   /* an operator/unary op was used before anything was captured/entered */
    CALC_ERR_NO_MEASURE, /* CAPTURE was pressed but the live measurement isn't valid */
} calc_err_t;

/* Display precision only - see the file-level comment. */
typedef enum
{
    CALC_ROUND_INT = 0,
    CALC_ROUND_0_1,
    CALC_ROUND_0_01,
    CALC_ROUND_0_001,
} calc_round_t;

typedef enum
{
    CALC_UNIT_MM = 0,
    CALC_UNIT_IN,
} calc_unit_t;

#define CALC_HISTORY_LEN 8

typedef struct
{
    double a;   /* 1st operand (or the value itself, for unary ops) */
    double b;   /* 2nd operand; unused (0) for unary ops */
    calc_op_t op;
    double result;
} calc_history_entry_t;

typedef struct
{
    bool has_value;        /* is `value` meaningful, i.e. is there something to compute with? */
    double value;          /* current working value, in mm, full precision */
    calc_op_t pending_op;  /* CALC_OP_NONE, or +,-,x,/ waiting for a 2nd operand */
    calc_err_t error;      /* CALC_ERR_NONE unless the last action failed */
    calc_round_t display_round; /* how the UI should format `value` (doesn't affect it) */
    calc_unit_t unit;      /* how the UI should display `value` (doesn't affect it) */

    bool memory_has_value;
    double memory;

    calc_history_entry_t history[CALC_HISTORY_LEN];
    int history_count; /* how many of the slots below are valid, up to CALC_HISTORY_LEN */
    int history_next;  /* ring-buffer write cursor */
} calc_state_t;

void calc_init(calc_state_t *st);

/* Clears the current calculation (value/operator/error) - the "C" button.
 * Memory and history are intentionally left untouched. */
void calc_clear(calc_state_t *st);

/* Feeds a new operand - from CAPTURE, the manual keypad, or history reuse.
 * Becomes the first value if none is set yet; completes and resolves a
 * pending binary operator if one was selected; otherwise (no operator
 * pending) simply replaces the current value with this fresh one. */
void calc_feed_value(calc_state_t *st, double v);

/* Selects +, -, x or / as the pending operator for the next fed value.
 * Sets CALC_ERR_NO_VALUE and does nothing if there's no value yet. */
void calc_set_op(calc_state_t *st, calc_op_t op);

/* Cancels a pending operator (e.g. the wrong one was tapped) without
 * touching the value itself. */
void calc_cancel_op(calc_state_t *st);

/* Unary operators: apply immediately to the current value. */
void calc_apply_half(calc_state_t *st);
void calc_apply_double(calc_state_t *st);
void calc_apply_abs(calc_state_t *st);
void calc_toggle_sign(calc_state_t *st);

/* Memory. */
void calc_memory_add(calc_state_t *st);   /* M+ */
void calc_memory_sub(calc_state_t *st);   /* M- */
void calc_memory_recall(calc_state_t *st); /* MR: feeds the memory in as if captured */
void calc_memory_clear(calc_state_t *st); /* MC */

/* Display-only rounding: does not touch `v` itself, just returns a rounded
 * copy. `v` is expected to already be in the unit you want rounded (see
 * calc_mm_to_unit/calc_unit_to_mm). */
double calc_display_round(double v, calc_round_t r);

double calc_mm_to_unit(double mm, calc_unit_t unit);
double calc_unit_to_mm(double v, calc_unit_t unit);

/* Short ASCII label for an operator ("+", "-", "x", "/", "/2", "x2", "ABS",
 * "+/-"), or "" for CALC_OP_NONE. Deliberately ASCII (x/-, not proper
 * unicode x-B7/u00F7): the bundled LVGL fonts only cover ASCII + the
 * LV_SYMBOL_* icons, anything else just doesn't render. */
const char *calc_op_symbol(calc_op_t op);

/* Formats `mm` (a raw engine value, in millimeters) into buf as e.g.
 * "12.525 mm" or "1.000 in": converts to `unit` and rounds to
 * `round_mode` for display purposes only. */
void calc_format_value(double mm, calc_unit_t unit, calc_round_t round_mode, char *buf, size_t buf_len);

/* Formats a whole history line, e.g. "25.05 + 12.40 = 37.45 mm". */
void calc_format_history_entry(const calc_history_entry_t *e, calc_unit_t unit, calc_round_t round_mode, char *buf,
                                size_t buf_len);

/* index_from_newest = 0 is the most recent entry, up to history_count - 1
 * the oldest one still kept. Returns NULL if out of range. */
const calc_history_entry_t *calc_history_get(const calc_state_t *st, int index_from_newest);

#ifdef __cplusplus
}
#endif

#endif /* CALC_ENGINE_H */
