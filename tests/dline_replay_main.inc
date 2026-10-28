/* Appended by tests/run_ultra_dline_replay_test.py to its patched copy of the firmware's test/test_dline.c (see
 * dline_replay_hook.inc): one scenario per trace file given on the command line, the strip a WS2812B. */
int main(int argc, char **argv)
{
    int i;
    only = NULL;
    printf("dline replay: the ESPHome hub's digital port on the receiver's delay line\n");
    for (i = 1; i < argc; i++) {
        const char *slash = strrchr(argv[i], '/');
        char *nm = (char *)malloc(strlen(argv[i]) + 16);
        int n;
        sprintf(nm, "replay %s", slash != NULL ? slash + 1 : argv[i]);
        n = rp_load(argv[i]);
        if (n <= 0) {
            printf("  FAIL: trace %s: %d\n", argv[i], n);
            g_fail++;
            continue;
        }
        sc_begin(nm, C_WS2812);
        S.nit = n;
        go();
    }
    printf("dline replay: %ld checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
