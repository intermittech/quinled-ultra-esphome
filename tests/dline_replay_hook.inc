/* Included by tests/run_ultra_dline_replay_test.py's patched copy of the firmware's test/test_dline.c, just before its run():
 * the model's hub (gen_item) is replaced by a REPLAY of what the ESPHome hub's digital-port code put on the wire in
 * tests/ultra_digital_transport_test.cpp (the RMT symbols it gave the driver, and when the direction pin let go of the
 * line). Everything else is the firmware's: the receiver's delay line (dline.c) on a sample-exact model of its hardware, and
 * its checks on the strip: every pixel frame whole and in order, no marker / frame / release / answer on the output, every
 * poll answered, every capture decoding to the frame sent.
 *
 * Trace file lines:  T <start_us> <P pixels | F marker frame | Q poll> <nbytes> <bytes hex> <nwords> <RMT words hex>
 *                    D <t_us> <0|1>        the direction pin (0: the hub let go of the line)
 */
#include <stddef.h>

typedef struct {
    long long start_us;
    char kind;
    int nbytes, nwords;
    unsigned char bytes[2048];
    unsigned int *words;
} rec_t;

static rec_t *RP;
static int nRP, iRP;
static long long rp_base_us;
static long long *de_t;
static int *de_l;
static int n_de;

static int rp_load(const char *path)
{
    static char line[1 << 17];
    FILE *f = fopen(path, "r");
    int cap = 0, markers = 0;
    if (f == NULL) {
        return -1;
    }
    nRP = n_de = iRP = 0;
    free(RP);
    RP = NULL;
    while (fgets(line, sizeof line, f) != NULL) {
        if (line[0] == 'T') {
            rec_t *r;
            char kind;
            long long start;
            int nb, nw, off = 0, k;
            char *p;
            if (sscanf(line, "T %lld %c %d%n", &start, &kind, &nb, &off) != 3 || nb > (int)sizeof RP[0].bytes) {
                fclose(f);
                return -2;
            }
            if (kind != 'P' && markers >= 58) {
                continue;                               /* (the model keeps 64 marker frames) */
            }
            if (nRP == cap) {
                cap = cap ? cap * 2 : 256;
                RP = (rec_t *)realloc(RP, (size_t)cap * sizeof *RP);
            }
            r = &RP[nRP];
            memset(r, 0, offsetof(rec_t, words));
            r->start_us = start;
            r->kind = kind;
            r->nbytes = nb;
            p = line + off;
            for (k = 0; k < nb; k++) {
                unsigned v;
                sscanf(p, " %2x", &v);
                r->bytes[k] = (unsigned char)v;
                p += 3;
            }
            if (sscanf(p, " %d%n", &nw, &off) != 1) {
                fclose(f);
                return -3;
            }
            p += off;
            r->nwords = nw;
            r->words = (unsigned int *)malloc((size_t)nw * 4u);
            for (k = 0; k < nw; k++) {
                unsigned v;
                sscanf(p, " %x%n", &v, &off);
                r->words[k] = v;
                p += off;
            }
            markers += kind != 'P';
            nRP++;
        } else if (line[0] == 'D') {
            long long t;
            int l;
            if (sscanf(line, "D %lld %d", &t, &l) == 2) {
                de_t = (long long *)realloc(de_t, (size_t)(n_de + 1) * sizeof *de_t);
                de_l = (int *)realloc(de_l, (size_t)(n_de + 1) * sizeof *de_l);
                de_t[n_de] = t;
                de_l[n_de] = l;
                n_de++;
            }
        }
    }
    fclose(f);
    if (getenv("ULTRA_REPLAY_MUTATE") != NULL) {          /* negative control: the LOW after every marker frame 290 us short (10 us left) */
        int j;
        for (j = 0; j + 1 < nRP; j++) {
            if (RP[j].kind == 'F' && RP[j + 1].kind == 'P') {
                unsigned int *w = &RP[j].words[RP[j].nwords - 1];
                *w = 400u | (400u << 16);                  /* the tail word: 10 us */
                RP[j + 1].start_us -= 290;
            }
        }
    }
    if (nRP > 0) {
        rp_base_us = RP[0].start_us - 1000;
    }
    return nRP;
}

static double rp_ns(long long us)
{
    return (double)(us - rp_base_us) * 1000.0;
}

static void replay_gen_item(void)
{
    const rec_t *t;
    double ns;
    int k, first = 0, pix, bit_i = 0, nbits;
    long long last_fall_s = 0, first_fall_s = 0, first_rise_s = 0;
    double last_fall_ns = 0;
    itn++;
    t = &RP[iRP++];
    pix = t->kind == 'P';
    ns = rp_ns(t->start_us);
    if (eh > 0 && ns < ns_of(eb[(eh - 1) & (EB_N - 1)].s) - ts_ns) {
        printf("  replay: record %d (%c, %lld us) starts %.2f us before the line's last edge (retake at %.2f us)\n", iRP - 1, t->kind,
               t->start_us - rp_base_us, (ns_of(eb[(eh - 1) & (EB_N - 1)].s) - ns) / 1000.0, ns_of(retake_s) / 1000.0);
    }
    while (first < t->nwords && !((t->words[first] >> 15) & 1u) && !((t->words[first] >> 31) & 1u)) {
        first++;                                        /* the lead LOW */
    }
    if (!pix) {
        first++;                                        /* the marker's word */
    }
    nbits = t->nbytes * 8;
    for (k = 0; k < t->nwords; k++) {
        const unsigned int w = t->words[k];
        const double d0 = (double)(w & 0x7FFFu) * 1000.0 / 80.0, d1 = (double)((w >> 16) & 0x7FFFu) * 1000.0 / 80.0;
        const int l0 = (int)((w >> 15) & 1u), l1 = (int)((w >> 31) & 1u);
        const int is_bit = k >= first && k < first + nbits;
        put(ns, l0, l0 ? (pix ? K_PIX : K_WIN) : K_LOW);
        if (is_bit) {
            if (bit_i == 0) {
                first_rise_s = s_of(ns);
                first_fall_s = s_of(ns + d0);
            }
            last_fall_s = s_of(ns + d0);
            last_fall_ns = ns + d0;
            bit_i++;
        }
        put(ns + d0, l1, l1 ? (pix ? K_PIX : K_WIN) : K_LOW);
        ns += d0 + d1;
    }
    put(ns, 0, K_LOW);
    if (pix) {
        xframe_t *x = &xf[nxf < XF_N ? nxf++ : XF_N - 1];
        x->nbits = nbits;
        x->bits = (unsigned char *)realloc(x->bits, (size_t)t->nbytes);
        memcpy(x->bits, t->bytes, (size_t)t->nbytes);
        x->s0 = first_rise_s;
        x->s1 = last_fall_s;
        x->ff = first_fall_s;
        hub_ns = ns;
    } else {
        mframe_t *m = &mf[nmf < MF_N ? nmf++ : MF_N - 1];
        memset(m, 0, sizeof *m);
        m->rw = t->kind == 'Q';
        m->nbits = nbits;
        memcpy(m->bytes, t->bytes, (size_t)t->nbytes);
        m->t_end = last_fall_s;
        m->t_rel = -1;
        m->expect = m->rw ? 1 : 0;
        if (m->rw) {
            /* the hub let go of the line when its direction pin said so; the retake at the window's end (W_D) */
            double rel_ns = 20e3;
            int j;
            for (j = 0; j < n_de; j++) {
                const double t_ns = rp_ns(de_t[j]);
                if (de_l[j] == 0 && t_ns >= last_fall_ns - 1e3 && t_ns <= last_fall_ns + 400e3) {
                    rel_ns = t_ns - last_fall_ns;
                    break;
                }
            }
            put(last_fall_ns + rel_ns, 1, K_WIN);       /* the fail-safe HIGH */
            m->t_rel = s_of(last_fall_ns + rel_ns);
            hub_wait = 1;
            wd_s = s_of(last_fall_ns + (double)ultra_dp_window_us(ULTRA_RW_RATE_1M) * 1000.0);
            retake_s = wd_s;
            hub_dead = 1;
            post_us = 0.0;
            ret_lat_us = 0.0;
            poll_rate = ULTRA_RW_RATE_1M;
        } else {
            hub_ns = ns;
        }
    }
}
#define gen_item replay_gen_item
