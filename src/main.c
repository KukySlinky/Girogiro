// gyroicons - plugin de usuario para PS Vita (taiHEN), se carga en *main (SceShell)
// Las burbujas del Live Area solo se animan cuando el giroscopio detecta movimiento.
// Sin libc: solo funciones del kernel (sceClib*) para no depender de newlib dentro del shell.
//
// Modo "reloj virtual" (no necesita offset):
//   Engancha las funciones con las que el shell lee la hora y devuelve un reloj que avanza
//   a velocidad = gate (0 = congelado, 1 = normal). El gate depende del giroscopio.
//
// gyroicons.cfg:
//   hooks=0/1     instala los hooks de reloj (0 = no toca nada del shell)
//   clock=0..3    qué reloj se congela: bit0 = ProcessTime, bit1 = SystemTime
//                 clock=0 => solo CUENTA llamadas (modo diagnóstico, no cambia nada visible)
//   minrate=0.0   velocidad mínima del reloj cuando la consola está quieta (0 = congelado)

#include <psp2/kernel/modulemgr.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/clib.h>
#include <psp2/motion.h>
#include <psp2/io/fcntl.h>
#include <taihen.h>

#define CFG_PATH "ux0:data/gyroicons.cfg"
#define LOG_PATH "ux0:data/gyroicons_log.txt"

// NIDs del firmware 3.60 (vita-headers db/360)
#define NID_PROCESS_TIME_WIDE 0xB110C123  // sceKernelGetProcessTimeWide (SceLibKernel)
#define NID_PROCESS_TIME_LOW  0xE9F973B1  // sceKernelGetProcessTimeLow  (SceLibKernel)
#define NID_SYSTEM_TIME_WIDE  0xF4EE4FA9  // sceKernelGetSystemTimeWide  (SceThreadmgr)

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
static float  g_minrate   = 0.0f;
static int    g_log       = 0;
static int    g_hooks     = 0;
static int    g_clock     = 0;
static unsigned int g_offset = 0;
static int    g_thumb     = 1;

// ---------------------------------------------------------------------------
// Reloj virtual lineal por tramos: virt = virt_base + (real - real_base) * rate
// Lo escribe solo el hilo del sensor (cada ~16 ms); los hooks solo leen (seqlock).
// Al cambiar de tramo se conserva el valor virtual, así que el reloj nunca retrocede.
// ---------------------------------------------------------------------------
typedef struct {
    volatile unsigned int seq;
    volatile SceUInt64 real_base;
    volatile SceUInt64 virt_base;
    volatile float rate;
} vclock_t;

static vclock_t g_clk_proc;
static vclock_t g_clk_sys;

static void clk_init(vclock_t *c, SceUInt64 now) {
    c->seq = 0;
    c->real_base = now;
    c->virt_base = now;
    c->rate = 1.0f;
}

static void clk_tick(vclock_t *c, SceUInt64 real_now, float new_rate) {
    SceUInt64 v = c->virt_base;
    if (real_now > c->real_base)
        v += (SceUInt64)((float)(real_now - c->real_base) * c->rate);
    c->seq++;
    __sync_synchronize();
    c->virt_base = v;
    c->real_base = real_now;
    c->rate = new_rate;
    __sync_synchronize();
    c->seq++;
}

static SceUInt64 clk_read(vclock_t *c, SceUInt64 real_now) {
    for (int i = 0; i < 2000; i++) {
        unsigned int s1 = c->seq;
        if (s1 & 1) continue;
        __sync_synchronize();
        SceUInt64 rb = c->real_base;
        SceUInt64 vb = c->virt_base;
        float r = c->rate;
        __sync_synchronize();
        if (c->seq == s1) {
            if (real_now <= rb) return vb;
            return vb + (SceUInt64)((float)(real_now - rb) * r);
        }
    }
    return real_now; // no se pudo leer un tramo consistente: reloj real
}

// Contadores de llamadas por función (para saber cuál usa el shell)
static volatile unsigned int g_cnt_pw, g_cnt_pl, g_cnt_sw;
static SceUID g_h_pw = -1, g_h_pl = -1, g_h_sw = -1;
static int    g_r_pw = 0,  g_r_pl = 0,  g_r_sw = 0;   // resultado de cada hook (para el log)
static tai_hook_ref_t g_ref_pw, g_ref_pl, g_ref_sw;

static SceUInt64 hook_process_wide(void) {
    __sync_fetch_and_add(&g_cnt_pw, 1);
    SceUInt64 real = TAI_CONTINUE(SceUInt64, g_ref_pw);
    return (g_clock & 1) ? clk_read(&g_clk_proc, real) : real;
}

static SceUInt32 hook_process_low(void) {
    __sync_fetch_and_add(&g_cnt_pl, 1);
    SceUInt32 real = TAI_CONTINUE(SceUInt32, g_ref_pl);
    if (!(g_clock & 1)) return real;
    // Este módulo no está enganchado a sí mismo: esta llamada devuelve la hora real
    return (SceUInt32)clk_read(&g_clk_proc, sceKernelGetProcessTimeWide());
}

static SceInt64 hook_system_wide(void) {
    __sync_fetch_and_add(&g_cnt_sw, 1);
    SceInt64 real = TAI_CONTINUE(SceInt64, g_ref_sw);
    return (g_clock & 2) ? (SceInt64)clk_read(&g_clk_sys, (SceUInt64)real) : real;
}

// ---------------------------------------------------------------------------
// Configuración (sin libc)
// ---------------------------------------------------------------------------
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
    else if (!sceClibStrncmp(l, "minrate=", 8))    g_minrate = parse_float(l + 8);
    else if (!sceClibStrncmp(l, "log=", 4))        g_log = (int)parse_uint(l + 4);
    else if (!sceClibStrncmp(l, "hooks=", 6))      g_hooks = (int)parse_uint(l + 6);
    else if (!sceClibStrncmp(l, "clock=", 6))      g_clock = (int)parse_uint(l + 6);
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
    if (g_minrate < 0.0f) g_minrate = 0.0f;
    if (g_minrate > 1.0f) g_minrate = 1.0f;
}

// ---------------------------------------------------------------------------
// Hilo del sensor
// ---------------------------------------------------------------------------
static int sensor_thread(SceSize args, void *argp) {
    SceMotionState ms;
    float level = 1.0f;
    SceUID log_fd = -1;
    int tick = 0;

    sceMotionStartSampling();

    if (g_log) {
        log_fd = sceIoOpen(LOG_PATH, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
        if (log_fd >= 0) {
            char line[128];
            int n = sceClibSnprintf(line, sizeof(line),
                "hooks=%d clock=%d res pw=%x pl=%x sw=%x\n",
                g_hooks, g_clock, g_r_pw, g_r_pl, g_r_sw);
            if (n > 0) sceIoWrite(log_fd, line, (SceSize)n);
        }
    }

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

            if (g_hooks) {
                float rate = g_minrate + (1.0f - g_minrate) * level;
                clk_tick(&g_clk_proc, sceKernelGetProcessTimeWide(),
                         (g_clock & 1) ? rate : 1.0f);
                clk_tick(&g_clk_sys, (SceUInt64)sceKernelGetSystemTimeWide(),
                         (g_clock & 2) ? rate : 1.0f);
            }

            // Cada ~0.5 s: mag2 y gate x1000 (enteros) + llamadas a cada función de reloj
            if (log_fd >= 0 && (++tick % 30) == 0) {
                char line[128];
                int n = sceClibSnprintf(line, sizeof(line),
                    "mag2=%d gate=%d pw=%u pl=%u sw=%u\n",
                    (int)(mag2 * 1000.0f), (int)(level * 1000.0f),
                    g_cnt_pw, g_cnt_pl, g_cnt_sw);
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

    SceUInt64 now_p = sceKernelGetProcessTimeWide();
    SceUInt64 now_s = (SceUInt64)sceKernelGetSystemTimeWide();
    clk_init(&g_clk_proc, now_p);
    clk_init(&g_clk_sys, now_s);

    // Los hooks se instalan ANTES de arrancar el hilo, así el log ya sabe su resultado
    if (g_hooks) {
        g_h_pw = taiHookFunctionImport(&g_ref_pw, "SceShell", TAI_ANY_LIBRARY,
                                       NID_PROCESS_TIME_WIDE, hook_process_wide);
        g_h_pl = taiHookFunctionImport(&g_ref_pl, "SceShell", TAI_ANY_LIBRARY,
                                       NID_PROCESS_TIME_LOW, hook_process_low);
        g_h_sw = taiHookFunctionImport(&g_ref_sw, "SceShell", TAI_ANY_LIBRARY,
                                       NID_SYSTEM_TIME_WIDE, hook_system_wide);
        g_r_pw = (int)g_h_pw;
        g_r_pl = (int)g_h_pl;
        g_r_sw = (int)g_h_sw;
    }

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
    if (g_hook >= 0)    taiHookRelease(g_hook, g_ref);
    if (g_h_pw >= 0)    taiHookRelease(g_h_pw, g_ref_pw);
    if (g_h_pl >= 0)    taiHookRelease(g_h_pl, g_ref_pl);
    if (g_h_sw >= 0)    taiHookRelease(g_h_sw, g_ref_sw);
    return SCE_KERNEL_STOP_SUCCESS;
}
