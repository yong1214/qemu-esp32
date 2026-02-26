#include "qemu/osdep.h"
#include <inttypes.h>
#include "hw/irq.h"
#include "hw/gpio/esp32_gpio.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/misc/esp32_ledc.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>

#define ESP32_LEDC_REGS_SIZE (A_LEDC_CONF_REG + 4)

#define APB_CLK_HZ 80000000U
#define LEDC_CONF0_SIG_OUT_EN BIT(2)
#define LEDC_CONF0_IDLE_LV    BIT(1)
#define LEDC_CONF1_DUTY_START BIT(4)

#define LEDC_MAX_DUTY_BITS 20
#define LEDC_TIMER_MAX_DIV 0x3FFFF

#define PWM_MSG_SIZE 14

static const int ledc_default_pins[ESP32_LEDC_CHANNEL_CNT] = {
    18, 19, 23, 5, 17, 16, 4, 2,
    12, 13, 14, 15, 25, 26, 27, 32,
};

/* ═══════════════════════════════════════════════════════════
 *  PWM Pipe Monitor (Unified Binary Protocol)
 *
 *  14-byte messages: [PWM\0][pin][en][duty:2LE][top:2LE][freq:4LE]
 * ═══════════════════════════════════════════════════════════ */

static int pwm_pipe_fd = -1;
static int pwm_pipe_ready = 0;

/* Per-channel previous state for change detection */
static uint32_t prev_duty[ESP32_LEDC_CHANNEL_CNT];
static uint32_t prev_freq[ESP32_LEDC_CHANNEL_CNT];
static uint16_t prev_top[ESP32_LEDC_CHANNEL_CNT];
static uint8_t  prev_enabled[ESP32_LEDC_CHANNEL_CNT];

static int ensure_fifo(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0) {
        if (S_ISFIFO(st.st_mode)) return 0;
        unlink(path);
    }
    if (mkfifo(path, 0666) != 0 && errno != EEXIST) {
        qemu_log("❌ ESP32 LEDC PWM: mkfifo(%s) failed (errno=%d)\n", path, errno);
        return -1;
    }
    return 0;
}

static void pwm_pipe_send(int pin, uint8_t enabled,
                          uint16_t duty, uint16_t top, uint32_t freq)
{
    if (!pwm_pipe_ready || pwm_pipe_fd < 0) return;

    /* Change detection */
    int ch_idx = -1;
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        /* Match by pin (channel_pin is set in realize) */
        if (ledc_default_pins[i] == pin) { ch_idx = i; break; }
    }
    if (ch_idx >= 0) {
        if (prev_duty[ch_idx] == duty && prev_freq[ch_idx] == freq &&
            prev_top[ch_idx] == top && prev_enabled[ch_idx] == enabled) {
            return;  /* no change */
        }
        prev_duty[ch_idx] = duty;
        prev_freq[ch_idx] = freq;
        prev_top[ch_idx] = top;
        prev_enabled[ch_idx] = enabled;
    }

    uint8_t msg[PWM_MSG_SIZE] = {
        0x50, 0x57, 0x4D, 0x00,                /* "PWM\0" */
        (uint8_t)pin,
        enabled,
        (uint8_t)(duty & 0xFF),
        (uint8_t)((duty >> 8) & 0xFF),
        (uint8_t)(top & 0xFF),
        (uint8_t)((top >> 8) & 0xFF),
        (uint8_t)(freq & 0xFF),
        (uint8_t)((freq >>  8) & 0xFF),
        (uint8_t)((freq >> 16) & 0xFF),
        (uint8_t)((freq >> 24) & 0xFF)
    };

    ssize_t written = write(pwm_pipe_fd, msg, PWM_MSG_SIZE);
    if (written < 0 && errno == EPIPE) {
        close(pwm_pipe_fd);
        pwm_pipe_fd = -1;
        pwm_pipe_ready = 0;
    }
}

void esp32_ledc_pwm_pipe_init(const char *pipe_path)
{
    if (pwm_pipe_ready) return;
    if (!pipe_path) return;

    if (ensure_fifo(pipe_path) < 0) return;

    memset(prev_duty, 0, sizeof(prev_duty));
    memset(prev_freq, 0, sizeof(prev_freq));
    memset(prev_top, 0, sizeof(prev_top));
    memset(prev_enabled, 0, sizeof(prev_enabled));

    qemu_log("⏳ ESP32 LEDC PWM: waiting for reader on %s ...\n", pipe_path);
    pwm_pipe_fd = open(pipe_path, O_WRONLY);
    if (pwm_pipe_fd < 0) {
        qemu_log("❌ ESP32 LEDC PWM: open(%s) failed (errno=%d)\n",
                 pipe_path, errno);
        return;
    }
    fcntl(pwm_pipe_fd, F_SETFL, O_NONBLOCK);
    pwm_pipe_ready = 1;
    qemu_log("✅ ESP32 LEDC PWM pipe opened: %s\n", pipe_path);
}

void esp32_ledc_pwm_pipe_cleanup(void)
{
    if (pwm_pipe_fd >= 0) {
        close(pwm_pipe_fd);
        pwm_pipe_fd = -1;
    }
    pwm_pipe_ready = 0;
}

/* ═══════════════════════════════════════════════════════════
 *  Original LEDC logic
 * ═══════════════════════════════════════════════════════════ */

static inline uint32_t ledc_extract_duty(uint32_t reg)
{
    return (reg >> 4) & ((1u << LEDC_MAX_DUTY_BITS) - 1);
}

static inline uint32_t ledc_extract_hpoint(uint32_t reg)
{
    return reg & ((1u << LEDC_MAX_DUTY_BITS) - 1);
}

static inline bool ledc_channel_enabled(uint32_t conf0)
{
    return (conf0 & LEDC_CONF0_SIG_OUT_EN) != 0;
}

static inline int ledc_channel_timer_index(int channel, uint32_t conf0)
{
    int timer_sel = (conf0 >> 8) & 0x3;
    return (channel < 8) ? timer_sel : (timer_sel + 4);
}

static uint32_t ledc_timer_resolution_bits(Esp32LEDCState *s, int timer_idx)
{
    uint32_t res = s->duty_res[timer_idx] & 0x1F;
    if (res == 0) res = 1;
    if (res > LEDC_MAX_DUTY_BITS) res = LEDC_MAX_DUTY_BITS;
    return res;
}

static uint32_t ledc_timer_freq(Esp32LEDCState *s, int timer_idx)
{
    uint32_t raw = (s->timer_conf_reg[timer_idx] >> 5) & LEDC_TIMER_MAX_DIV;
    if (raw == 0) return 0;
    double src_hz = (s->timer_conf_reg[timer_idx] & BIT(25)) ? (double)APB_CLK_HZ : 1000000.0;
    uint32_t res_bits = ledc_timer_resolution_bits(s, timer_idx);
    double freq = src_hz / ((double)raw * (double)(1u << res_bits));
    return (uint32_t)freq;
}

static void ledc_update_irq(Esp32LEDCState *s)
{
    bool active = (s->int_raw & s->int_ena) != 0;
    qemu_set_irq(s->irq, active ? 1 : 0);
}

static void ledc_schedule_timer(Esp32LEDCState *s, int timer_idx);

static void ledc_apply_gpio(Esp32LEDCState *s, int ch, bool level)
{
    if (s->gpio && s->channel_pin[ch] >= 0) {
        esp32_gpio_set_input_level(s->gpio, s->channel_pin[ch], level);
    }
}

static void ledc_update_channel(Esp32LEDCState *s, int channel, uint64_t event_time)
{
    uint32_t conf0 = s->channel_conf0_reg[channel];
    bool enabled = ledc_channel_enabled(conf0);
    bool idle_lv = (conf0 & LEDC_CONF0_IDLE_LV) != 0;
    int timer_idx = ledc_channel_timer_index(channel, conf0);
    uint32_t res_bits = ledc_timer_resolution_bits(s, timer_idx);
    uint32_t period = 1u << res_bits;
    uint32_t duty = ledc_extract_duty(s->channel_duty_r_reg[channel]);
    if (duty >= period && period > 0) {
        duty = period - 1;
    }
    bool level = idle_lv;
    if (enabled && period > 0) {
        uint32_t counter = s->timer_counter[timer_idx] % period;
        uint32_t hpoint = ledc_extract_hpoint(s->channel_hpoint_reg[channel]) % period;
        uint32_t rel = (counter + period - hpoint) % period;
        level = rel < duty;
    }
    if (s->channel_level[channel] != level) {
        bool prev_level = s->channel_level[channel];
        s->channel_level[channel] = level;
        ledc_apply_gpio(s, channel, level);
        int pin = s->channel_pin[channel];
        if (pin >= 0) {
            fprintf(stdout,
                    "PWM_EVT channel=%d pin=%d level=%d time=%" PRIu64 " phase=edge edge=%s prev=%d\n",
                    channel,
                    pin,
                    level ? 1 : 0,
                    event_time,
                    level ? "rise" : "fall",
                    prev_level ? 1 : 0);
            fflush(stdout);
        }
    }
    unsigned percent = 0;
    if (period > 1) {
        percent = (duty * 100U) / (period - 1U);
        if (percent > 100U) {
            percent = 100U;
        }
    }
    led_set_intensity(&s->led[channel], percent);
}

static void ledc_timer_cb(void *opaque)
{
    Esp32LedcTimerCtx *ctx = (Esp32LedcTimerCtx *)opaque;
    Esp32LEDCState *s = ctx->s;
    int timer_idx = ctx->index;
    uint32_t res_bits = ledc_timer_resolution_bits(s, timer_idx);
    uint32_t period = 1u << res_bits;
    if (period == 0) return;

    uint32_t raw = (s->timer_conf_reg[timer_idx] >> 5) & LEDC_TIMER_MAX_DIV;
    if (raw == 0) return;

    uint32_t tick_sel = (s->timer_conf_reg[timer_idx] >> 25) & 0x1;
    uint64_t src_hz = tick_sel ? APB_CLK_HZ : 1000000ULL;
    uint64_t numerator = (uint64_t)raw * 1000000000ULL;
    uint64_t denominator = src_hz * 256ULL;
    uint64_t step_ns = denominator ? (numerator + (denominator / 2)) / denominator : 0;
    if (step_ns == 0) return;

    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t elapsed = now - s->timer_last_ns[timer_idx];
    s->timer_accum_ns[timer_idx] += elapsed;
    s->timer_last_ns[timer_idx] = now;

    uint64_t ticks_to_process = s->timer_accum_ns[timer_idx] / step_ns;
    s->timer_accum_ns[timer_idx] -= ticks_to_process * step_ns;

    for (uint64_t tick = 0; tick < ticks_to_process; ++tick) {
        uint64_t tick_time = step ? (now - s->timer_accum_ns[timer_idx] - (step_ns * (ticks_to_process - 1 - tick))) : now;
        s->timer_counter[timer_idx] = (s->timer_counter[timer_idx] + 1) % period;
        s->timer_value_reg[timer_idx] = s->timer_counter[timer_idx];
    
        /* Emit PWM_EVT on every period wrap (rising edge) for frequency measurement */
        if (s->timer_counter[timer_idx] == 0) {
            if (timer_idx < 4) {
                s->int_raw |= BIT(16 + timer_idx);
            } else {
                s->int_raw |= BIT(20 + (timer_idx - 4));
            }
            /* Emit period boundary events for all active channels on this timer */
            for (int ch = 0; ch < ESP32_LEDC_CHANNEL_CNT; ++ch) {
                uint32_t conf0 = s->channel_conf0_reg[ch];
                if (!ledc_channel_enabled(conf0)) {
                    continue;
                }
                if (ledc_channel_timer_index(ch, conf0) == timer_idx) {
                    int pin = s->channel_pin[ch];
                    if (pin >= 0) {
                        fprintf(stdout,
                                "PWM_EVT channel=%d pin=%d level=%d time=%" PRIu64 " phase=wrap\n",
                                ch,
                                pin,
                                s->channel_level[ch] ? 1 : 0,
                                tick_time);
                        fflush(stdout);
                    }
                }
            }
        }
    
        for (int ch = 0; ch < ESP32_LEDC_CHANNEL_CNT; ++ch) {
            uint32_t conf0 = s->channel_conf0_reg[ch];
            if (!ledc_channel_enabled(conf0)) {
                continue;
            }
            if (ledc_channel_timer_index(ch, conf0) == timer_idx) {
                ledc_update_channel(s, ch, tick_time);
            }
        }
    }

    ledc_update_irq(s);

    timer_mod_ns(s->timer[timer_idx], now + step_ns);
}

static void ledc_schedule_timer(Esp32LEDCState *s, int timer_idx)
{
    if (timer_idx < 0 || timer_idx >= ESP32_LEDC_TIMER_CNT) return;
    if (!s->timer[timer_idx]) {
        Esp32LedcTimerCtx *ctx = g_new0(Esp32LedcTimerCtx, 1);
        ctx->s = s;
        ctx->index = timer_idx;
        s->timer_ctx[timer_idx] = ctx;
        s->timer[timer_idx] = timer_new_ns(QEMU_CLOCK_VIRTUAL, ledc_timer_cb, ctx);
    }
    uint32_t raw = (s->timer_conf_reg[timer_idx] >> 5) & LEDC_TIMER_MAX_DIV;
    if (raw == 0) return;
    uint32_t tick_sel = (s->timer_conf_reg[timer_idx] >> 25) & 0x1;
    uint64_t src_hz = tick_sel ? APB_CLK_HZ : 1000000ULL;
    uint64_t numerator = (uint64_t)raw * 1000000000ULL;
    uint64_t denominator = src_hz * 256ULL;
    uint64_t step = denominator ? (numerator + (denominator / 2)) / denominator : 0;
    if (step == 0) return;
    s->timer_last_ns[timer_idx] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->timer_accum_ns[timer_idx] = 0;
    timer_mod_ns(s->timer[timer_idx], s->timer_last_ns[timer_idx] + step);
}

static void ledc_set_duty(Esp32LEDCState *s, int channel, uint32_t value)
{
    s->channel_duty_reg[channel] = value;
    s->channel_duty_r_reg[channel] = value;
    if (channel < 8) {
        s->int_raw |= BIT(channel);
    } else {
        s->int_raw |= BIT(8 + channel - 8);
    }
    ledc_update_channel(s, channel, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    ledc_update_irq(s);

    /* ── PWM pipe: emit duty change ── */
    {
        uint32_t conf0 = s->channel_conf0_reg[channel];
        bool enabled = ledc_channel_enabled(conf0);
        int timer_idx = ledc_channel_timer_index(channel, conf0);
        uint32_t res_bits = ledc_timer_resolution_bits(s, timer_idx);
        uint16_t top = (uint16_t)((1u << res_bits) - 1);
        uint16_t duty16 = (uint16_t)(ledc_extract_duty(value) & 0xFFFF);
        uint32_t freq = ledc_timer_freq(s, timer_idx);
        int pin = s->channel_pin[channel];
        if (pin >= 0) {
            pwm_pipe_send(pin, enabled ? 1 : 0, duty16, top, freq);
        }
    }
}

static bool ledc_decode_timer_addr(hwaddr addr, int *timer_idx, bool *is_value)
{
    if (addr >= A_LEDC_HSTIMER0_CONF_REG && addr < A_LEDC_LSTIMER0_CONF_REG) {
        hwaddr rel = addr - A_LEDC_HSTIMER0_CONF_REG;
        *timer_idx = rel / 0x8;
        hwaddr off = rel & 0x7;
        *is_value = (off == 0x4);
        return off == 0 || off == 0x4;
    }
    if (addr >= A_LEDC_LSTIMER0_CONF_REG && addr < (A_LEDC_LSTIMER3_CONF_REG + 0x8)) {
        hwaddr rel = addr - A_LEDC_LSTIMER0_CONF_REG;
        *timer_idx = 4 + (rel / 0x8);
        hwaddr off = rel & 0x7;
        *is_value = (off == 0x4);
        return off == 0 || off == 0x4;
    }
    return false;
}

static bool ledc_decode_channel_addr(hwaddr addr, int *channel, uint32_t *offset)
{
    if (addr >= A_LEDC_HSCH0_CONF0_REG && addr < A_LEDC_LSCH0_CONF0_REG) {
        hwaddr rel = addr - A_LEDC_HSCH0_CONF0_REG;
        *channel = rel / 0x14;
        *offset = rel % 0x14;
        return *channel < 8;
    }
    if (addr >= A_LEDC_LSCH0_CONF0_REG && addr < (A_LEDC_LSCH7_CONF0_REG + 0x14)) {
        hwaddr rel = addr - A_LEDC_LSCH0_CONF0_REG;
        *channel = 8 + rel / 0x14;
        *offset = rel % 0x14;
        return *channel < ESP32_LEDC_CHANNEL_CNT;
    }
    return false;
}

static uint64_t ledc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    int timer_idx;
    bool is_value;
    if (ledc_decode_timer_addr(addr, &timer_idx, &is_value)) {
        if (is_value) {
            return s->timer_value_reg[timer_idx];
        }
        return s->timer_conf_reg[timer_idx];
    }
    int channel;
    uint32_t offset;
    if (ledc_decode_channel_addr(addr, &channel, &offset)) {
        switch (offset) {
        case 0x0:
            return s->channel_conf0_reg[channel];
        case 0x4:
            return s->channel_hpoint_reg[channel];
        case 0x8:
            return s->channel_duty_reg[channel];
        case 0xC:
            return s->channel_conf1_reg[channel];
        case 0x10:
            return s->channel_duty_r_reg[channel];
        default:
            return 0;
        }
    }
    switch (addr) {
    case A_LEDC_INT_RAW_REG:
        return s->int_raw;
    case A_LEDC_INT_ST_REG:
        return s->int_raw & s->int_ena;
    case A_LEDC_INT_ENA_REG:
        return s->int_ena;
    case A_LEDC_INT_CLR_REG:
        return 0;
    case A_LEDC_CONF_REG:
        return 0;
    default:
        return 0;
    }
}

static void ledc_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32LEDCState *s = ESP32_LEDC(opaque);
    int timer_idx;
    bool is_value;
    if (ledc_decode_timer_addr(addr, &timer_idx, &is_value)) {
        if (!is_value) {
            s->timer_conf_reg[timer_idx] = value;
            uint32_t raw = (value >> 5) & 0x3FFFF;
            uint32_t duty_res = value & 0x1F;
            uint32_t tick_sel = (value >> 25) & 0x1;
            uint64_t src_hz = tick_sel ? APB_CLK_HZ : 1000000ULL;
            uint64_t numerator = (uint64_t)raw * 1000000000ULL;
            uint64_t denominator = src_hz * 256ULL;
            uint64_t step_ns = denominator ? (numerator + (denominator / 2)) / denominator : 0;
            fprintf(stdout,
                    "LED write timer%d: value=0x%08" PRIx64 " raw=%u duty_res=%u tick_sel=%u step_ns=%" PRIu64 "\n",
                    timer_idx, value, raw, duty_res, tick_sel, step_ns);
            fflush(stdout);
            if ((value & 0x1F) != 0) {
                s->duty_res[timer_idx] = value & 0x1F;
            }
            s->timer_counter[timer_idx] = 0;
            s->timer_value_reg[timer_idx] = 0;
            ledc_schedule_timer(s, timer_idx);

            /* ── PWM pipe: emit freq change for all channels on this timer ── */
            {
                uint32_t freq = ledc_timer_freq(s, timer_idx);
                uint32_t res_bits = ledc_timer_resolution_bits(s, timer_idx);
                uint16_t top = (uint16_t)((1u << res_bits) - 1);
                for (int ch = 0; ch < ESP32_LEDC_CHANNEL_CNT; ++ch) {
                    uint32_t conf0 = s->channel_conf0_reg[ch];
                    if (ledc_channel_timer_index(ch, conf0) == timer_idx) {
                        bool enabled = ledc_channel_enabled(conf0);
                        uint16_t duty16 = (uint16_t)(ledc_extract_duty(s->channel_duty_r_reg[ch]) & 0xFFFF);
                        int pin = s->channel_pin[ch];
                        if (pin >= 0) {
                            pwm_pipe_send(pin, enabled ? 1 : 0, duty16, top, freq);
                        }
                    }
                }
            }
        }
        return;
    }
    int channel;
    uint32_t offset;
    if (ledc_decode_channel_addr(addr, &channel, &offset)) {
        switch (offset) {
        case 0x0:
            s->channel_conf0_reg[channel] = value;
            ledc_update_channel(s, channel, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
            ledc_schedule_timer(s, ledc_channel_timer_index(channel, value));

            /* ── PWM pipe: emit enable/disable or timer reassignment ── */
            {
                bool enabled = ledc_channel_enabled(value);
                int ti = ledc_channel_timer_index(channel, value);
                uint32_t res_bits = ledc_timer_resolution_bits(s, ti);
                uint16_t top = (uint16_t)((1u << res_bits) - 1);
                uint16_t duty16 = (uint16_t)(ledc_extract_duty(s->channel_duty_r_reg[channel]) & 0xFFFF);
                uint32_t freq = ledc_timer_freq(s, ti);
                int pin = s->channel_pin[channel];
                if (pin >= 0) {
                    pwm_pipe_send(pin, enabled ? 1 : 0, duty16, top, freq);
                }
            }
            break;
        case 0x4:
            s->channel_hpoint_reg[channel] = value;
            break;
        case 0x8:
            ledc_set_duty(s, channel, value);
            break;
        case 0xC:
            s->channel_conf1_reg[channel] = value;
            if (value & LEDC_CONF1_DUTY_START) {
                ledc_set_duty(s, channel, s->channel_duty_reg[channel]);
            }
            break;
        case 0x10:
            s->channel_duty_r_reg[channel] = value;
            ledc_update_channel(s, channel, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
            break;
        default:
            break;
        }
        return;
    }
    switch (addr) {
    case A_LEDC_INT_RAW_REG:
        s->int_raw |= value;
        ledc_update_irq(s);
        break;
    case A_LEDC_INT_ENA_REG:
        s->int_ena = value;
        ledc_update_irq(s);
        break;
    case A_LEDC_INT_CLR_REG:
        s->int_raw &= ~value;
        ledc_update_irq(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps esp32_ledc_ops = {
    .read = ledc_read,
    .write = ledc_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_ledc_realize(DeviceState *dev, Error **errp)
{
    Esp32LEDCState *s = ESP32_LEDC(dev);
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        qdev_realize(DEVICE(&s->led[i]), NULL, &error_fatal);
        s->channel_pin[i] = ledc_default_pins[i];
        s->channel_level[i] = false;
        s->channel_conf0_reg[i] = 0;
        s->channel_conf1_reg[i] = 0;
        s->channel_duty_reg[i] = 0;
        s->channel_duty_r_reg[i] = 0;
        s->channel_hpoint_reg[i] = 0;
        led_set_intensity(&s->led[i], 0);
    }
    for (int t = 0; t < ESP32_LEDC_TIMER_CNT; ++t) {
        s->timer_conf_reg[t] = 0;
        s->timer_value_reg[t] = 0;
        s->duty_res[t] = 1;
        s->timer_counter[t] = 0;
        s->timer_last_ns[t] = 0;
        s->timer_accum_ns[t] = 0;
        s->timer[t] = NULL;
        s->timer_ctx[t] = NULL;
    }
    s->int_raw = 0;
    s->int_ena = 0;

    /* Open PWM pipe if env var is set */
    const char *pwm_path = getenv("QEMU_ESP32_PWM_PIPE");
    if (pwm_path && pwm_path[0]) {
        esp32_ledc_pwm_pipe_init(pwm_path);
    }
}

static void esp32_ledc_init(Object *obj)
{
    Esp32LEDCState *s = ESP32_LEDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    memory_region_init_io(&s->iomem, obj, &esp32_ledc_ops, s,
                          TYPE_ESP32_LEDC, ESP32_LEDC_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    for (int i = 0; i < ESP32_LEDC_CHANNEL_CNT; i++) {
        object_initialize_child(obj, g_strdup_printf("led%d", i + 1),
                                &s->led[i], TYPE_LED);
        s->led[i].color = (char *)"blue";
    }
}

static void esp32_ledc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = esp32_ledc_realize;
}

static const TypeInfo esp32_ledc_info = {
        .name = TYPE_ESP32_LEDC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(Esp32LEDCState),
        .instance_init = esp32_ledc_init,
        .class_init = esp32_ledc_class_init
};

static void esp32_ledc_register_types(void)
{
    type_register_static(&esp32_ledc_info);
}

type_init(esp32_ledc_register_types)

void esp32_ledc_attach_gpio(Esp32LEDCState *s, Esp32GpioState *gpio)
{
    s->gpio = gpio;
    for (int ch = 0; ch < ESP32_LEDC_CHANNEL_CNT; ++ch) {
        ledc_apply_gpio(s, ch, s->channel_level[ch]);
    }
}
