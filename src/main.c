// gyroicons - plugin de usuario para PS Vita (taiHEN), se carga en *SceShell
// Las burbujas del Live Area solo se animan cuando el giroscopio detecta movimiento.
// Sin libc: solo funciones del kernel (sceClib*) para no depender de newlib dentro del shell.

#include <psp2/kernel/modulemgr.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/kernel/clib.h>
#include <psp2/motion.h>
#include <psp2/io/fcntl.h>
#include <taihen.h>

#define CFG_PATH "ux0:data/gyroicons.cfg"
#define LOG_PATH "ux0:data/gyroicons_log.txt"

// Stubs vacíos: evitan el error de enlazado "_free_vita_newlib"
void __attribute__((weak)) _init_vita_newlib(void) {}
void __attribute__((weak)) _free_vita_newlib(void) {}

static volatile float g_gate = 1.0f;
static volatile int   g_run  = 1;
static SceUID g_thread = -1;
static SceUID g_hook   = -1;
static tai_hook_ref_t g_ref;

static float  g_threshold = 0.35f;
static float  g_attack    = 0.35f;
static float  g_decay     = 0.06f;
static int    g_log       = 0;
static unsigned int g_offset = 0;
static int    g_thumb     = 1;

static float parse_float(const char *s) {
    float sign = 1.0f, v = 0.0f, scale = 0.1f;
    if (*s == '-') { sign = -1.0f; s++; }
    while (*s >= '0' && *s <= '9') { v = v * 10.0f + (float)(*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { v += (float)(*s - '0') * scale; scale *= 0.1f; s++; }
    }
    return sign * v;
}

static unsigned int parse_uint(const char *s) {
    unsigned int v = 0;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        s += 2;
        for (;; s++) {
            char c = *s;
            int d;
            if (c >= '0' && c <= '9') d = c - '0';
            else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
            else break;
            v = v * 16 + (unsigned int)d;
        }
    } else {
        while (*s >= '0' && *s <= '9') { v = v * 10 + (unsigned int)(*s - '0'); s++; }
    }
    return v;
}

static void apply_line(const char *l) {
    if (!sceClibStrncmp(l, "threshold=", 10))      g_threshold = parse_float(l + 10);
    else if (!sceClibStrncmp(l, "attack=", 7))     g_attack = parse_float(l + 7);
    else if (!sceClibStrncmp(l, "decay=", 6))      g_decay = parse_float(l + 6);
    else if (!sceClibStrncmp(l, "log=", 4))        g_log = (int)parse_uint(l + 4);
    else if (!sceClibStrncmp(l, "thumb=", 6))      g_thumb = (int)parse_uint(l + 6);
    else if (!sceClibStrncmp(l, "offset=", 7))     g_offset = parse_uint(l + 7);
}

static void load_cfg(void) {
    char buf[512];
    sceClibMemset(buf, 0, sizeof(buf));
    SceUID fd = sceIoOpen(CFG_PATH, SCE_O_RDONLY, 0);
    if (fd < 0) return;
    int n = sceIoRead(fd, buf, sizeof(buf) - 1);
    sceIoClose(fd);
    if (n <= 0) return;
    buf[n] = 0;

    char *p = buf;
    while (*p) {
        char *line = p;
        while (*p && *p != '\n' && *p != '\r') p++;
        if (*p) { *p = 0; p++; }
        if (*line) apply_line(line);
    }
}

static int sensor_thread(SceSize args, void *argp) {
    SceMotionState ms;
    float level = 1.0f;
    SceUID log_fd = -1;
    int tick = 0;

    sceMotionStartSampling();

    if (g_log)
        log_fd = sceIoOpen(LOG_PATH, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);

    while (g_run) {
        if (sceMotionGetState(&ms) == 0) {
            float x = ms.angularVelocity.x;
            float y = ms.angularVelocity.y;
            float z = ms.angularVelocity.z;
            // Se compara el cuadrado de la magnitud: evita usar sqrt (y libm)
            float mag2 = x * x + y * y + z * z;
            float thr2 = g_threshold * g_threshold;

            float target = (mag2 > thr2) ? 1.0f : 0.0f;
            float k = (target > level) ? g_attack : g_decay;
            level += (target - level) * k;
            if (level < 0.001f) level = 0.0f;
            g_gate = level;

            // Cada ~0.5 s: mag2 y gate multiplicados por 1000 (enteros)
            if (log_fd >= 0 && (++tick % 30) == 0) {
                char line[96];
                int n = sceClibSnprintf(line, sizeof(line), "mag2=%d gate=%d\n",
                                        (int)(mag2 * 1000.0f), (int)(level * 1000.0f));
                if (n > 0) sceIoWrite(log_fd, line, (SceSize)n);
            }
        }
        sceKernelDelayThread(16000);
    }

    if (log_fd >= 0) sceIoClose(log_fd);
    sceMotionStopSampling();
    return 0;
}

// SUPUESTO (verificar en Ghidra): la función recibe el delta de tiempo como float en r0.
static int anim_hook(unsigned int dt_bits) {
    union { unsigned int u; float f; } c;
    c.u = dt_bits;
    c.f *= g_gate;
    return TAI_CONTINUE(int, g_ref, c.u);
}

int _start(SceSize argc, const void *args) __attribute__((weak, alias("module_start")));
int module_start(SceSize argc, const void *args) {
    load_cfg();

    g_thread = sceKernelCreateThread("gyroicons_sensor", sensor_thread, 0x10000100, 0x4000, 0, 0, NULL);
    if (g_thread >= 0) sceKernelStartThread(g_thread, 0, NULL);

    if (g_offset != 0) {
        tai_module_info_t info;
        info.size = sizeof(info);
        if (taiGetModuleInfo("SceShell", &info) >= 0)
            g_hook = taiHookFunctionOffset(&g_ref, info.modid, 0, g_offset, g_thumb, anim_hook);
    }
    return SCE_KERNEL_START_SUCCESS;
}

int module_stop(SceSize argc, const void *args) {
    g_run = 0;
    if (g_thread >= 0) {
        sceKernelWaitThreadEnd(g_thread, NULL, NULL);
        sceKernelDeleteThread(g_thread);
    }
    if (g_hook >= 0) taiHookRelease(g_hook, g_ref);
    return SCE_KERNEL_STOP_SUCCESS;
}
