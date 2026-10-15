/* The Diff-Power ultra receiver (board firmware BOARD=2) for tests/ultra_telemetry_contract_test.cpp: the firmware's
 * own whole-firmware model, test/test_soak.c (src/app/main.c itself on a model of the hardware: the DIN waveform, the
 * edge counter, the ring copies, the ADCs, the flash; test/test_soak_rc.c records the return channel's hal calls),
 * included here unchanged and driven through ultra_telemetry_receiver.h. Nothing in the firmware tree is changed; what
 * this file adds around it:
 *  - test_soak.c's kernel32 calls (fibers, the flash model's VirtualAlloc at 0x08000000) go to ucrx_* below:
 *    kernel32 on Windows, ucontext / mmap elsewhere (zig cc on Linux has no kernel32, and its clang no __declspec);
 *  - its main() is renamed (#pragma redefine_extname: test_soak.c #undefs a `main` macro before defining it);
 *  - power-ups: every variable of the firmware and of the model lives in one section (#pragma clang section here,
 *    -include ultra_telemetry_receiver.h with UCRX_FW_SECTION for the files of src/core, and
 *    -fno-zero-initialized-in-bss), so a power-up gives them their load-time values again, as the C startup does on
 *    the chip (main.c's .noinit breadcrumbs keep theirs); the flash model keeps its content if asked (the receiver's
 *    records survive a restart). test_soak.c itself boots once per process; this is what lets one process restart it.
 * The window (ucrx_window) is test_soak_rc.c's rc_window with the hub's own slots and W. */
#if !defined(_WIN32)
#define _GNU_SOURCE
#include <sys/mman.h>
#include <ucontext.h>
#endif
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ultra_telemetry_receiver.h"

/* ================================================================== platform (outside the firmware's section) */
#if defined(_WIN32)
__declspec(dllimport) void *__stdcall VirtualAlloc(void *addr, size_t size, unsigned long type, unsigned long prot);
__declspec(dllimport) void *__stdcall ConvertThreadToFiber(void *param);
__declspec(dllimport) void *__stdcall CreateFiber(size_t stack, void(__stdcall *start)(void *), void *param);
__declspec(dllimport) void __stdcall SwitchToFiber(void *fiber);
__declspec(dllimport) void __stdcall DeleteFiber(void *fiber);
#endif

static void *ucrx_flash_map;
static size_t ucrx_flash_size;

/* the flash model at its real address (once: later power-ups get the same mapping) */
void *ucrx_VirtualAlloc(void *addr, size_t size, unsigned long type, unsigned long prot)
{
    if (ucrx_flash_map == NULL) {
#if defined(_WIN32)
        ucrx_flash_map = VirtualAlloc(addr, size, type, prot);
#else
#ifndef MAP_FIXED_NOREPLACE
#define MAP_FIXED_NOREPLACE 0x100000
#endif
        void *p = mmap(addr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
        (void)type;
        (void)prot;
        ucrx_flash_map = p == MAP_FAILED ? NULL : p;
#endif
        ucrx_flash_size = size;
    }
    return ucrx_flash_map == addr && size <= ucrx_flash_size ? ucrx_flash_map : NULL;
}

#if defined(_WIN32)
static void *ucrx_main_fiber, *ucrx_fw_fiber;
void *ucrx_ConvertThreadToFiber(void *param)
{
    if (ucrx_main_fiber == NULL) {
        ucrx_main_fiber = ConvertThreadToFiber(param);   /* (a second call fails: the thread is a fiber already) */
    }
    return ucrx_main_fiber;
}
void *ucrx_CreateFiber(size_t stack, void (*start)(void *), void *param)
{
    if (ucrx_fw_fiber != NULL) {
        DeleteFiber(ucrx_fw_fiber);                      /* a power-up: the last run's main() (suspended) */
    }
    ucrx_fw_fiber = CreateFiber(stack, (void(__stdcall *)(void *))start, param);
    return ucrx_fw_fiber;
}
void ucrx_SwitchToFiber(void *fiber) { SwitchToFiber(fiber); }
#else
typedef struct {
    ucontext_t uc;
    void (*start)(void *);
    void *param, *stack;
} ucrx_fiber_t;
static ucrx_fiber_t ucrx_main_fiber, *ucrx_cur, *ucrx_fw_fiber;
void *ucrx_ConvertThreadToFiber(void *param)
{
    (void)param;
    ucrx_cur = &ucrx_main_fiber;
    return &ucrx_main_fiber;
}
static void ucrx_fiber_entry(void)
{
    ucrx_cur->start(ucrx_cur->param);
    abort();                                             /* (main() never returns) */
}
void *ucrx_CreateFiber(size_t stack, void (*start)(void *), void *param)
{
    ucrx_fiber_t *f = (ucrx_fiber_t *)calloc(1, sizeof *f);
    if (ucrx_fw_fiber != NULL) {
        free(ucrx_fw_fiber->stack);                      /* a power-up: the last run's main() (suspended) */
        free(ucrx_fw_fiber);
    }
    if (f == NULL || (f->stack = malloc(stack)) == NULL || getcontext(&f->uc) != 0) {
        abort();
    }
    f->uc.uc_stack.ss_sp = f->stack;
    f->uc.uc_stack.ss_size = stack;
    f->uc.uc_link = NULL;
    f->start = start;
    f->param = param;
    makecontext(&f->uc, ucrx_fiber_entry, 0);
    ucrx_fw_fiber = f;
    return f;
}
void ucrx_SwitchToFiber(void *fiber)
{
    ucrx_fiber_t *from = ucrx_cur;
    ucrx_cur = (ucrx_fiber_t *)fiber;
    if (swapcontext(&from->uc, &ucrx_cur->uc) != 0) {
        abort();
    }
}
#endif

/* ================================================================== the firmware and its model (the section) */
#undef __declspec
#undef __stdcall
#define __declspec(x)
#define __stdcall
#define VirtualAlloc ucrx_VirtualAlloc
#define ConvertThreadToFiber ucrx_ConvertThreadToFiber
#define CreateFiber ucrx_CreateFiber
#define SwitchToFiber ucrx_SwitchToFiber
#pragma redefine_extname main ucrx_soak_main
#if defined(_WIN32)
#pragma clang section data="ucfw$m"
#else
#pragma clang section data="ucfw"
#endif

#include "test_soak.c"

#pragma clang section data=""
#undef VirtualAlloc
#undef ConvertThreadToFiber
#undef CreateFiber
#undef SwitchToFiber

#if !BOARD_RETURN
#error "the receiver's build: BOARD=2"
#endif

/* the section's bounds */
#if defined(_WIN32)
__attribute__((section("ucfw$a"), used)) static uint8_t ucfw_begin[16] = { 1 };
__attribute__((section("ucfw$z"), used)) static uint8_t ucfw_end[16] = { 1 };
#define UCFW_BEGIN ucfw_begin
#define UCFW_END   ucfw_end
#else
extern uint8_t __start_ucfw[], __stop_ucfw[];
#define UCFW_BEGIN __start_ucfw
#define UCFW_END   __stop_ucfw
#endif

/* ================================================================== the API (outside the section: kept) */
static uint8_t *volatile ucrx_sec_begin, *volatile ucrx_sec_end;   /* (volatile: no object size for the copies) */
static uint8_t *ucrx_image;                              /* the section at load time */
static size_t ucrx_image_n;
static uint8_t *ucrx_flash_keep;
static uint32_t ucrx_hub_base_ms;                        /* the hub's ms at this power-up */
static uint64_t ucrx_hub_host_us = 7000000u;             /* the hub's clock (us) at this power-up */
static int ucrx_booted;
static int ucrx_fails;                                   /* the soak's CHECK failures of earlier power-ups */
static int ucrx_checked;                                 /* this power-up's end checks done */

static int ucrx_in_section(const void *p)
{
    return (const uint8_t *)p >= ucrx_sec_begin && (const uint8_t *)p < ucrx_sec_end;
}

void ucrx_init(void)
{
    ucrx_sec_begin = UCFW_BEGIN;
    ucrx_sec_end = UCFW_END;
    ucrx_image_n = (size_t)(ucrx_sec_end - ucrx_sec_begin);
    ucrx_image = (uint8_t *)malloc(ucrx_image_n);
    if (ucrx_image == NULL) {
        abort();
    }
    memcpy(ucrx_image, ucrx_sec_begin, ucrx_image_n);
    /* the section holds main.c's, the model's and src/core's variables (-include), not this file's own */
    if (!(ucrx_in_section(&up_s) && ucrx_in_section(&temp_peak) && ucrx_in_section(&sim_ns) &&
          ucrx_in_section(&ultra_fx_rejected) && ucrx_in_section(core_scratch) && !ucrx_in_section(&ucrx_image) &&
          !ucrx_in_section(&ucrx_flash_map))) {
        printf("receiver: the firmware's variables are not in their section (%p .. %p)\n", (void *)UCFW_BEGIN,
               (void *)UCFW_END);
        abort();
    }
}

/* test_soak.c main()'s checks at a scenario's end, and test_soak_rc.c's: the flash model saw no half-word programmed
 * that wasn't erased, operations only on the record pages, erases only while every output was below 15 %, and none
 * while a response window ran */
static void ucrx_end_checks(void)
{
    if (!ucrx_booted || ucrx_checked) {
        return;
    }
    ucrx_checked = 1;
    CHECK(fl_bad == 0u, "receiver: no half-word programmed that wasn't erased (%u)", fl_bad);
    CHECK(fl_outside == 0u, "receiver: flash operations only on the record pages and the update status records (%u)",
          fl_outside);
    CHECK(fl_bright_erases == 0u, "receiver: erases only while every output is below 15 %% (%u erases while brighter, "
          "%u while lit)", fl_bright_erases, fl_lit_erases);
    CHECK(rc_flash_in_window == 0u, "receiver: no flash operation while a response window runs (%u)",
          rc_flash_in_window);
    ucrx_fails += g_fail;
}

void ucrx_power_up(int keep_flash, uint32_t reset_flags)
{
    if (ucrx_booted) {
        ucrx_end_checks();
        ucrx_hub_base_ms += (uint32_t)(sim_ns / 1000000u);
        ucrx_hub_host_us = host_us(sim_ns);              /* (wraps at 2^32 as the hub's) */
        if (keep_flash) {
            if (ucrx_flash_keep == NULL && (ucrx_flash_keep = (uint8_t *)malloc(FL_SIZE)) == NULL) {
                abort();
            }
            memcpy(ucrx_flash_keep, fl, FL_SIZE);
        }
    }
    memcpy(ucrx_sec_begin, ucrx_image, ucrx_image_n);    /* every variable as at load time (the C startup) */
    flash_init();                                        /* the flash model: mapped once, erased */
    if (ucrx_booted && keep_flash) {
        memcpy(fl, ucrx_flash_keep, FL_SIZE);
    }
    soak_reset_flags = reset_flags;
    boot(5000u, 100000u, ucrx_hub_host_us);              /* (as test_soak_rc.c rc_boot_locked) */
    host_on = 1;
    ucrx_booted = 1;
    ucrx_checked = 0;
}

uint32_t ucrx_hub_ms(void) { return ucrx_hub_base_ms + (uint32_t)(sim_ns / 1000000u); }

void ucrx_run_ms(uint32_t ms) { run_ms(ms); }

void ucrx_update_info(uint8_t bl_version, uint8_t hw_id, uint8_t hw_rev, uint32_t version)
{
    upd.info.bl_version = bl_version;
    upd.info.hw_id = hw_id;
    upd.info.hw_rev = hw_rev;
    upd.info.version = version;
    upd.info.desc_ok = 1u;
    upd.info.confirmed = 1u;
    upd.confirm_done = 1u;
}

void ucrx_raw_frame(const uint8_t *f, uint32_t nbytes, uint32_t gap_us)
{
    static uint8_t b[64];
    uint64_t t_end;
    if (nbytes == 0u || nbytes > sizeof b) {
        abort();
    }
    host_on = 0;
    while (wf_on) {
        run_ticks(1);
    }
    memcpy(b, f, nbytes);
    wire_frame(b, nbytes * 8u, sim_ns + 1000u);
    t_end = wf_fall(nbytes * 8u - 1u) + 300000u + (uint64_t)gap_us * 1000u;
    while (sim_ns < t_end) {
        run_ticks(1);
    }
}

void ucrx_host(int on) { host_on = on; }

void ucrx_contend(int check, int tx)
{
    rc_contend_check = check;
    rc_contend_tx = tx;
}

void ucrx_truth(ucrx_truth_t *t)
{
    const vdip_ev_t *e = vdip_event(&vdip, 0);
    uint32_t i;
    memset(t, 0, sizeof *t);
    t->fw_version = fw_version;
    strncpy(t->fw_text, fw_version_text, sizeof t->fw_text - 1u);   /* (memset above: NUL-terminated) */
    t->board = BOARD;
    t->hw_id = stat.hw_id;
    t->hw_rev = stat.hw_rev;
    t->bl_version = stat.bl_version;
    memcpy(t->uid, (const uint8_t *)soak_uid, 12);
    t->features = RC_FEATURES;
    t->hw_caps = RC_HW_CAPS;
    t->up_s = up_s;
    t->reset_flags = stat.reset_flags;
    t->power_state = pwrev_state(&pw);
    t->mcu_c10 = mt.c10;
    t->vdda_mv = mt.vdda_mv;
    t->vbus_valid = vs_sup.valid;
    t->vbus_mv = vs_sup.mv;
    t->i_avg_ma = is.i_avg;
    t->isense_enabled = is.enabled;
    t->isense_calibrated = is.calibrated;
    t->pwm_preset = eff.pwm_preset;
    t->proto_mode = frx.ps.mode;
    t->stagger = eff.stagger;
    t->config_crc = rc_cfg_crc();
    for (i = 0; i < 5u; i++) {
        t->out[i] = out_now[i];
    }
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        t->fx_lights |= (uint8_t)((ul.light[i].fx != ULTRA_FX_NONE) << i);
        t->defined_lights |= (uint8_t)((ul.light[i].defined != 0u) << i);
    }
    t->scene_hold = srun.hold;
    t->limiting = is.limiting != 0u;
    t->tripped = is.tripped != ISENSE_TRIP_NONE;
    t->ch_limited = ul.ch_limited != 0u;
    t->dark_hold = dk.active != 0u;
    t->pattern = ack.active != 0u;
    t->soft_start = ss.state == SS_RUN;
    t->signal_lost = signal_lost != 0u;
    t->test = TJ_ACTIVE() || is.cal_step != 0u;
    t->latched = pw.latched;
    t->limit_events = pw.limit_events;
    t->ovl_trips = pw.ovl_trips;
    t->short_trips = pw.short_trips;
    t->limit_age_s = pw.limit_age_s;
    t->trip_age_s = pw.trip_age_s;
    t->limit_ma = pw.in.limit_ma;
    t->event_ma = pw.event_ma;
    t->trip_ma = pw.trip_ma;
    t->factor = pw.in.factor;
    t->limit_src = pw.in.limit_src;
    t->last_cause = pw.last_cause;
    t->trip_cause = pw.in.trip_cause;
    t->retry_ms = pw.in.retry_ms;
    t->dips = vdip.count;
    t->logged = vdip.n_log;
    if (e != NULL) {
        t->dip_t_s = e->t_s;
        t->dip_mv = (uint16_t)vsense_mv(&vs_sup, e->min_x, 1u);
        t->dip_pct = (uint8_t)vdip_pct(e);
        t->dip_ms = e->dur_ms;
        t->dip_ma = e->i_ma;
    }
    t->supply_mv = (uint16_t)supply_mv;
    t->he_flags = hl.flags;
    for (i = 0; i < 5u; i++) {
        t->he_class[i] = hl.cls[i];
        t->he_test_dev[i] = hl.test_dev[i];
        t->he_model_dev[i] = hl.model_dev[i];
    }
    t->he_dark_ma = hl.dark_ma;
    t->temp_peak = temp_peak;
    t->boot_mwh = en.boot_mwh;
    t->total_wh = energy_total_wh(&en);
    t->energy_saved = en.saved;
    t->saved_s = en.saved_s;
    t->rc_windows = rc_windows;
    t->rc_missed = rc_missed;
    t->rc_answers = rc_answers;
    t->rc_aborts = rc_aborts;
    t->clock_rated = ul.clk.rated;
    t->ultra_on = ultra_on;
    t->upd_transparent = (uint8_t)upd_app_transparent(&upd);
    t->upd_sessions = upd.sessions_seen;
    t->ack_active = ack.active;
    t->ack_led_only = ack.led_only;
#ifdef ULTRA_FEAT_LIFETIME                               /* (a 0.6.9 firmware: protocol 0.2.5) */
    t->has_025 = 1u;
    t->why = (uint8_t)light_why();
    t->vb_valid = vs_aux.valid;
    t->vb_mv = vs_aux.mv;
    t->lt_on_s = en.lt.on_s;
    t->lt_lit_s = en.lt.lit_s;
    t->lt_ups = en.lt.ups;
    t->lt_dips = en.lt.dips;
    t->lt_writes = en.lt.writes;
    t->lt_trips[0] = en.lt.trips[0];
    t->lt_trips[1] = en.lt.trips[1];
    t->lt_updates = en.lt.updates;
    t->lt_wd = en.lt.wd;
    t->lt_peak_t = en.lt.peak_t;
    t->lt_peak_a = en.lt.peak_a;
    t->fm_trip = en.lt.trip;
    t->fm_trip_h = en.lt.trip_h;
    t->fm_dip_pct = en.lt.dip_pct;
    t->fm_dip_cs = en.lt.dip_cs;
    t->fm_dip_a = en.lt.dip_a;
    t->fm_rst = en.lt.rst;
    t->fm_rst_h = en.lt.rst_h;
    t->bc_valid = stat.bc.magic == BC_MAGIC;
    t->bc_cause = stat.bc.cause;
    t->bc_pc = stat.bc.pc;
    t->bc_up_s = stat.bc.up_s;
    t->supply_z = vdip_z_mohm(&vdip);
    t->flash_err = flash_err;
#endif
}

/* test_soak_rc.c rc_window, with the hub's slots: one RW frame (n slots, this receiver's own first) and its window */
void ucrx_window(const uint8_t *slots, uint32_t n, uint8_t hdr_flags, uint32_t window_us, uint32_t late_us, int noise,
                 ucrx_window_t *o, ucrx_truth_t *truth)
{
    static uint8_t f[32u * 64u + 8u];
    uint8_t nz[64];
    uint32_t i, nbits, stalls0 = sim_stalls, win0, miss0;
    uint64_t t_start, t_last, t_end;
    int was = host_on, sampled = 0;
    if (n == 0u || n > 64u) {
        abort();
    }
    nbits = 256u * n + 64u;
    host_on = 0;
    while (wf_on) {
        run_ticks(1);
    }
    run_ms(1);                                           /* >= 280 us of reset gap */
    memcpy(f, slots, 32u * n);
    t_start = sim_ns + rc_phase_ns;
    rc_phase_ns = 1000u + (rnd32() % 48000u);            /* (the next frame at another tick phase) */
    ultra_build_header_flags(f + 32u * n, host_us(t_start + (uint64_t)nbits * 1250u), hdr_flags);
    memset(&rcr, 0, sizeof rcr);
    stalls0 = sim_stalls - stalls0;                      /* (flash writes in the gap before the frame) */
    win0 = rc_windows;
    miss0 = rc_missed;
    wire_frame(f, nbits, t_start);
    t_last = wf_fall(nbits - 1u);
    while (sim_ns < t_last) {
        run_ticks(1);
    }
    if (late_us) {
        rc_ticks_no_pass(late_us / 50u + 4u);            /* (the frame end comes 100-150 us after the edge) */
    }
    t_end = t_last + (uint64_t)window_us * 1000u + 300000u;
    while (sim_ns < t_end) {
        run_ticks(1);
        if (truth != NULL && !sampled && rc_windows != win0) {
            ucrx_truth(truth);                           /* (the pass that built the answer) */
            sampled = 1;
        }
        if (noise && !wf_on && sim_ns >= t_last + 1150000u && sim_ns < t_last + 1200000u) {
            for (i = 0; i < sizeof nz; i++) {
                nz[i] = (uint8_t)(rnd32() >> 7);         /* (relayed answers: any edges) */
            }
            wire_frame(nz, 256u, sim_ns + 1000u);
            noise = 0;
        }
    }
    if (truth != NULL && !sampled) {
        ucrx_truth(truth);
    }
    host_on = was;
    memset(o, 0, sizeof *o);
    o->took_part = rcr.turns != 0u;
    o->checked = rcr.checks != 0u;
    o->quiet = rcr.quiets != 0u;
    o->sent = rcr.sends != 0u;
    o->stalled = stalls0 != 0u;
    o->missed = rc_missed != miss0;
    o->turn_us = rc_host_us_since(rcr.turn_ns, t_last);
    o->check_us = rc_host_us_since(rcr.check_ns, t_last);
    o->tx_us = rc_host_us_since(rcr.tx_ns, t_last);
    o->fwd_us = rc_host_us_since(rcr.fwd_ns, t_last);
    o->brr = rcr.brr;
    memcpy(o->answer, rcr.buf, 32);
}

void ucrx_cmd_def(uint8_t light, uint8_t ch_first, uint8_t n, uint16_t gamma_x1000)
{
    cmd_def(light, ch_first, n, gamma_x1000);
}
void ucrx_cmd_set(uint8_t light, uint16_t m, uint32_t dur_ms) { cmd_set(light, m, dur_ms, 0); }
void ucrx_cmd_effect(uint8_t light, uint8_t fx, const uint8_t *params, uint32_t n) { cmd_effect(light, fx, params, n); }
void ucrx_cmd_config(uint8_t id, uint16_t value) { cmd_config(id, value); }
void ucrx_cmd_identify(uint8_t what, uint8_t arg) { cmd_identify(what, arg); }
int ucrx_cmd_pending(void) { return hq_head != hq_tail; }

void ucrx_short(void)
{
    expect_trips++;
    short_ticks = 4;                                     /* (test_soak_rc.c rcpower: the fast trip) */
}
void ucrx_supply_mv(uint32_t mv) { supply_mv = mv; }
void ucrx_load_ma(uint8_t ch, uint32_t ma) { ld_ma[ch < 5u ? ch : 0u] = ma; }
uint32_t ucrx_flash_writes(void) { return sim_stalls; }
int ucrx_soak_failures(void)
{
    ucrx_end_checks();
    return ucrx_fails;
}
