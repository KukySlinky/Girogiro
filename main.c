// gyroicons - plugin de usuario para PS Vita (taiHEN), se carga en *SceShell
// Idea: las burbujas del Live Area solo se animan cuando el giroscopio detecta movimiento.
//
// ESTADO: la mitad del sensor (leer giroscopio + calcular "gate") está completa.
// La mitad del hook (congelar la animación idle de SceShell 3.60) necesita un offset
// que hay que encontrar con ingeniería inversa; se lee de ux0:data/gyroicons.cfg.

#include <psp2/kernel/modulemgr.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/motion.h>
#include <psp2/io/fcntl.h>
#include <taihen.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CFG_PATH "ux0:data/gyroicons.cfg"
#define LOG_PATH "ux0:data/gyroicons_log.txt"

static volatile float g_gate = 1.0f;   // 0 = animación congelada, 1 = normal
static volatile int   g_run  = 1;
static SceUID g_thread = -1;
static SceUID g_hook   = -1;
static tai_hook_ref_t g_ref;

// Valores ajustables (se pueden cambiar en gyroicons.cfg)
static float  g_threshold = 0.35f;  // rad/s: por debajo se considera "quieto"
static float  g_attack    = 0.35f;  // qué tan rápido se activa al mover
static float  g_decay     = 0.06f;  // qué tan rápido se frena al quedar quieto
static int    g_log       = 0;      // 1 = guardar lecturas del giroscopio en LOG_PATH
static unsigned int g_offset = 0;   // offset de la función de animación en SceShell
static int    g_thumb     = 1;      // 1 = código Thumb (lo normal en Vita)

static void load_cfg(void) {
    char buf[512];
    memset(buf, 0, sizeof(buf));
    SceUID fd = sceIoOpen(CFG_PATH, SCE_O_RDONLY, 0);
    if (fd < 0) return;
    sceIoRead(fd, buf, sizeof(buf) - 1);
    sceIoClose(fd);

    char *line = strtok(buf, "\r\n");
    while (line) {
        if (!strncmp(line, "threshold=", 10)) g_threshold = (float)atof(line + 10);
        else if (!strncmp(line, "attack=", 7))  g_attack = (float)atof(line + 7);
        else if (!strncmp(line, "decay=", 6))   g_decay = (float)atof(line + 6);
        else if (!strncmp(line, "log=", 4))     g_log = atoi(line + 4);
        else if (!strncmp(line, "thumb=", 6))   g_thumb = atoi(line + 6);
        else if (!strncmp(line, "offset=", 7))  g_offset = (unsigned int)strtoul(line + 7, NULL, 0);
        line = strtok(NULL, "\r\n");
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
            float mag = sqrtf(x * x + y * y + z * z);

            float target = (mag > g_threshold) ? 1.0f : 0.0f;
            float k = (target > level) ? g_attack : g_decay;
            level += (target - level) * k;
            if (level < 0.001f) level = 0.0f;
            g_gate = level;

            // Un renglón cada ~0.5 s para poder verificar que el giroscopio responde
            if (log_fd >= 0 && (++tick % 30) == 0) {
                char line[96];
                int n = snprintf(line, sizeof(line), "mag=%.3f gate=%.3f\n", mag, level);
                sceIoWrite(log_fd, line, n);
            }
        }
        sceKernelDelayThread(16000); // ~60 Hz
    }

    if (log_fd >= 0) sceIoClose(log_fd);
    sceMotionStopSampling();
    return 0;
}

// Hook de la función de animación idle de SceShell.
// SUPUESTO (hay que verificarlo en Ghidra): la función recibe el delta de tiempo
// como float en r0 (ABI softfp de Vita). Al multiplicarlo por g_gate, la animación
// avanza solo cuando hay movimiento.
static int anim_hook(unsigned int dt_bits) {
    float dt;
    memcpy(&dt, &dt_bits, sizeof(dt));
    dt *= g_gate;
    memcpy(&dt_bits, &dt, sizeof(dt));
    return TAI_CONTINUE(int, g_ref, dt_bits);
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
