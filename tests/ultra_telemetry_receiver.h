/* The Diff-Power ultra receiver (board firmware BOARD=2) as a library for tests/ultra_telemetry_contract_test.cpp:
 * the firmware's own whole-firmware model (test/test_soak.c: src/app/main.c on a model of the hardware, with
 * test/test_soak_rc.c's return channel), built by tests/ultra_telemetry_receiver.c. Plain C types only: the C++ test
 * includes the component's ultra_protocol.h, the receiver the firmware's copy. */
#ifndef ULTRA_TELEMETRY_RECEIVER_H
#define ULTRA_TELEMETRY_RECEIVER_H

/* tests/run_ultra_telemetry_contract_test.py compiles the firmware's src/core files with -include of this header and
 * UCRX_FW_SECTION: every variable of the firmware (and of the soak's model) goes into one section, so a power-up can
 * give them their load-time values again (the C startup's job on the chip) */
#ifdef UCRX_FW_SECTION
#if defined(_WIN32)
#pragma clang section data="ucfw$m"
#else
#pragma clang section data="ucfw"
#endif
#endif

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* what the receiver did in one RW frame's window (test_soak_rc.c's recording of hal/hw.c's return-channel calls) */
typedef struct {
    int took_part;               /* turned around (a valid POLL in an RW frame, clock rated, in time) */
    int checked, quiet, sent;    /* the turnaround check; quiet for the rest of the window; the burst went out */
    int stalled;                 /* a flash write in the gap right before the frame (the frame is lost to it) */
    int missed;                  /* the firmware counted the POLL as missed (clock not rated, main loop too late) */
    double turn_us, check_us, tx_us, fwd_us;   /* after the frame's last falling edge, host us */
    uint16_t brr;                /* USART3's divider for the burst */
    uint8_t answer[32];          /* the burst's answer (after the lead byte) */
} ucrx_window_t;

/* the firmware's own state (main.c's variables), sampled right after it built the answer (or at the window's end) */
typedef struct {
    /* STATUS / IDENTITY */
    uint32_t fw_version;         /* the version word (major << 24 | minor << 16 | patch << 8 | pre-release) */
    char fw_text[24];            /* main.c's text of it ("0.6.8") */
    uint8_t board, hw_id, hw_rev, bl_version;
    uint8_t uid[12];
    uint32_t features;           /* main.c RC_FEATURES */
    uint16_t hw_caps;            /* main.c RC_HW_CAPS */
    uint32_t up_s;
    uint32_t reset_flags;        /* RCC RSTSCKR at boot */
    uint8_t power_state;         /* pwrev_state */
    int16_t mcu_c10;
    uint16_t vdda_mv;
    uint8_t vbus_valid;
    uint32_t vbus_mv;
    int32_t i_avg_ma;
    uint8_t isense_enabled, isense_calibrated, pwm_preset, proto_mode, stagger;
    uint16_t config_crc;
    /* OUTPUTS */
    uint16_t out[5];
    uint8_t fx_lights, defined_lights, scene_hold;
    uint8_t limiting, tripped, ch_limited, dark_hold, pattern, soft_start, signal_lost, test;
    /* POWER (pwrev_t) */
    uint8_t latched;
    uint16_t limit_events, ovl_trips, short_trips;
    uint32_t limit_age_s, trip_age_s;
    uint16_t limit_ma, event_ma, trip_ma, factor;
    uint8_t limit_src, last_cause, trip_cause;
    uint32_t retry_ms;
    /* SUPPLY (vdip_t, the newest logged dip) */
    uint16_t dips;
    uint8_t logged;
    uint32_t dip_t_s;            /* uptime at its first low reading */
    uint16_t dip_mv, dip_ms, dip_ma;
    uint8_t dip_pct;
    uint16_t supply_mv;          /* the soak's supply (PA4's input) */
    /* HEALTH */
    uint8_t he_flags, he_class[5];
    int8_t he_test_dev[5], he_model_dev[5];
    int16_t he_dark_ma, temp_peak;
    /* ENERGY */
    uint32_t boot_mwh, total_wh;
    uint8_t energy_saved;
    uint32_t saved_s;            /* uptime of the lifetime record's last write */
    /* the return channel's own counters */
    uint32_t rc_windows, rc_missed, rc_answers, rc_aborts;
    uint8_t clock_rated, ultra_on;
    /* protocol 0.2.5 (a firmware with its header: has_025; else all 0) */
    uint8_t has_025;
    uint8_t why;                 /* STATUS 8 bits 4-7 (main.c light_why) */
    uint8_t vb_valid;            /* ENERGY 15: the LED supply after the buck */
    uint32_t vb_mv;
    uint32_t lt_on_s, lt_lit_s, lt_ups, lt_dips, lt_writes;   /* LIFETIME: en.lt */
    uint16_t lt_trips[2], lt_updates;
    uint8_t lt_wd, lt_peak_t, lt_peak_a;
    uint8_t fm_trip, fm_dip_pct, fm_dip_cs, fm_dip_a, fm_rst, fm_rst_h;   /* DIAG part 1: the fault memory */
    uint16_t fm_trip_h;
    uint8_t bc_valid, bc_cause;  /* DIAG part 0: the breadcrumbs, the supply impedance, the flash */
    uint16_t bc_pc;
    uint32_t bc_up_s, supply_z;
    uint8_t flash_err;
    uint8_t cal_end, cal_model, cal_done;   /* CALIBRATION: the last test's end, the model, a complete test this boot */
    uint32_t conn_ma;                       /* ... the connected load (climit) */
    /* the update path (core/upd_app.h, core/ack.h): transparent, sessions decided, the status LED's pattern */
    uint8_t upd_transparent, ack_active, ack_led_only;
    uint32_t upd_sessions;
} ucrx_truth_t;

/* once, first: keeps the load-time image of every firmware / model variable (the power-ups restore it) */
void ucrx_init(void);
/* a power-up: every variable as at load time, the flash erased (factory new) or kept (keep_flash: the receiver's
 * records survive, as on the chip), RCC's reset flags as given (0x0C000000: power-on + pin; 0x20000000: watchdog),
 * then main() boots and the hub's normal frames (50 fps, the soak's host) start; the hub's clock runs on */
void ucrx_power_up(int keep_flash, uint32_t reset_flags);
/* the hub's time (ms, runs on across power-ups) */
uint32_t ucrx_hub_ms(void);
/* the model: ms with the hub's normal frames (commands each 3 times, else NOP) */
void ucrx_run_ms(uint32_t ms);
/* one RW frame as this receiver sees it: slots[0] its own slot (its POLL), then the n - 1 slots after it, the header
 * (ULTRA_HDR_* flags) with the hub's time at the frame end; the model runs to window_us + 300 us after the frame's
 * last edge. late_us: the main loop stalls that long at the frame end; noise: answers of the boards behind it
 * pass its DIN in the window. *truth: the firmware's state when the answer was built (NULL: not wanted). */
void ucrx_window(const uint8_t *slots, uint32_t n, uint8_t hdr_flags, uint32_t window_us, uint32_t late_us, int noise,
                 ucrx_window_t *o, ucrx_truth_t *truth);
/* (auto_retry) the app as behind bootloader bl_version (hw_id, hw_rev), running version: what upd_boot() reads from
 * the bootloader's info and the app's descriptor (the model's build has no bootloader: UPD_APP 0) */
void ucrx_update_info(uint8_t bl_version, uint8_t hw_id, uint8_t hw_rev, uint32_t version);
/* one whole frame (nbytes, its own trailer) on the wire, the hub's normal frames off; then the line stays low
 * gap_us after the frame's reset (300 us) */
void ucrx_raw_frame(const uint8_t *f, uint32_t nbytes, uint32_t gap_us);
/* the hub's normal frames (the soak's host, 50 fps) on / off */
void ucrx_host(int on);
/* the line reads LOW (taken) at the next turnaround check / burst */
void ucrx_contend(int check, int tx);
void ucrx_truth(ucrx_truth_t *t);
/* the hub's commands (queued, each sent in 3 frames) */
void ucrx_cmd_def(uint8_t light, uint8_t ch_first, uint8_t n, uint16_t gamma_x1000);
void ucrx_cmd_set(uint8_t light, uint16_t m, uint32_t dur_ms);
void ucrx_cmd_effect(uint8_t light, uint8_t fx, const uint8_t *params, uint32_t n);
void ucrx_cmd_config(uint8_t id, uint16_t value);
void ucrx_cmd_identify(uint8_t what, uint8_t arg);
int ucrx_cmd_pending(void);
/* the hardware: a short (the fast trip's ADC samples), the supply (mV at PA4's divider), a channel's strip current */
void ucrx_short(void);
void ucrx_supply_mv(uint32_t mv);
void ucrx_load_ma(uint8_t ch, uint32_t ma);
/* flash writes since the last power-up (the soak's count of the CPU stalls) */
uint32_t ucrx_flash_writes(void);
/* the model's own checks (test_soak.c's at a scenario's end: the flash model's invariants, no flash operation in a
 * response window), of every power-up so far; call once, at the end */
int ucrx_soak_failures(void);

#ifdef __cplusplus
}
#endif

#endif
