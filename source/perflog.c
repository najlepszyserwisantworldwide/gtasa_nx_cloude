/* perflog.c -- per-thread CPU time and frame timing log (config.perf_log)
 *
 * With perf_log 1 a writer thread on core 2 appends one line every ~2 s to
 * /switch/gtasa/perf_threads.log:
 *   - presented frames, timed at eglSwapBuffers on the engine's RenderQueue
 *     thread: frame interval avg/max, frames slower than 20 ms / 36 ms, and
 *     the time spent inside the swap itself
 *   - main-thread logic time (implOnDrawFrame)
 *   - CPU time of every registered thread (main, engine threads, Mesa
 *     workers) in % of one core, and the sum of those threads per core
 *   - engine counters from libGame.so when the symbols exist: visible
 *     entities/LODs, LOD distance scales (follow the draw distance slider),
 *     peds, random cars
 *   - CPU/GPU/MEM clocks actually applied, and handheld/docked
 * Every launch starts with a header holding the config, so several runs can
 * be collected in one file.
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <switch.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "config.h"
#include "so_util.h"
#include "util.h"
#include "perflog.h"

#ifndef APP_VERSION_STR
#define APP_VERSION_STR "?"
#endif

extern so_module game_mod; // main.c

#define PL_FILE        "/switch/gtasa/perf_threads.log"
#define PL_PERIOD_NS   2000000000ULL
#define PL_MAX_THREADS 48
#define PL_CORES       3          // cores an application owns
#define TICKS_20MS     384000ULL  // 19.2 MHz system tick
#define TICKS_36MS     691200ULL

typedef struct {
  Handle handle;
  char name[16];
  int ready;               // published last by the registering thread
  int seen;                // writer only: baseline taken
  u64 last[PL_CORES];      // writer only
} PlThread;

static PlThread g_thr[PL_MAX_THREADS];
static int g_nthr;
static int g_armed;

static FILE *g_file;
static Thread g_writer;
static int g_writer_running;
static UEvent g_stop_event;
static u64 g_t0;
static u32 g_tick_info = InfoType_ThreadTickCount;

// per-window accumulators, swapped out by the writer
static u32 g_frames, g_over20, g_over36;
static u64 g_ft_sum, g_ft_max, g_swap_sum, g_swap_max;
static u64 g_last_swap;                 // RenderQueue thread only
static u32 g_logic_n, g_vis_max;
static u64 g_logic_sum, g_logic_max, g_vis_sum;
static u64 g_logic_t0;                  // main thread only

// engine counters in libGame.so (NULL when the symbol is missing)
static int *p_vis, *p_lods, *p_peds, *p_cars;
static float *p_lod_scale, *p_lowlod_scale;

static int g_clk_ok;
static ClkrstSession g_clk_cpu, g_clk_gpu, g_clk_emc;

static inline u64 take64(u64 *p) { return __atomic_exchange_n(p, 0, __ATOMIC_RELAXED); }
static inline u32 take32(u32 *p) { return __atomic_exchange_n(p, 0, __ATOMIC_RELAXED); }

static inline void max64(u64 *p, u64 v) {
  u64 cur = __atomic_load_n(p, __ATOMIC_RELAXED);
  while (v > cur &&
         !__atomic_compare_exchange_n(p, &cur, v, true, __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {}
}

static inline void max32(u32 *p, u32 v) {
  u32 cur = __atomic_load_n(p, __ATOMIC_RELAXED);
  while (v > cur &&
         !__atomic_compare_exchange_n(p, &cur, v, true, __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {}
}

static inline double ticks_ms(u64 t) {
  return (double)t * 1000.0 / (double)armGetSystemTickFreq();
}

static inline bool armed(void) {
  return __atomic_load_n(&g_armed, __ATOMIC_ACQUIRE) != 0;
}

// ---------------------------------------------------------------------------
// threads

void perflog_register_thread(const char *name) {
  if (!armed())
    return;
  const Handle h = threadGetCurHandle();
  const int n = __atomic_load_n(&g_nthr, __ATOMIC_ACQUIRE);
  for (int i = 0; i < n && i < PL_MAX_THREADS; i++)
    if (__atomic_load_n(&g_thr[i].ready, __ATOMIC_ACQUIRE) && g_thr[i].handle == h)
      return; // already registered by another hook
  const int i = __atomic_fetch_add(&g_nthr, 1, __ATOMIC_ACQ_REL);
  if (i >= PL_MAX_THREADS)
    return;
  g_thr[i].handle = h;
  snprintf(g_thr[i].name, sizeof(g_thr[i].name), "%s", name ? name : "?");
  __atomic_store_n(&g_thr[i].ready, 1, __ATOMIC_RELEASE);
}

void perflog_register_thread_numbered(const char *prefix, int *counter) {
  if (!armed())
    return;
  char name[16];
  snprintf(name, sizeof(name), "%s%d", prefix,
           __atomic_fetch_add(counter, 1, __ATOMIC_RELAXED));
  perflog_register_thread(name);
}

// CPU ticks the thread spent on each application core
static bool thread_ticks(Handle h, u64 out[PL_CORES]) {
  for (int c = 0; c < PL_CORES; c++)
    if (R_FAILED(svcGetInfo(&out[c], g_tick_info, h, (u64)c)))
      return false;
  return true;
}

// ---------------------------------------------------------------------------
// hot paths

void perflog_logic_begin(void) {
  if (armed())
    g_logic_t0 = armGetSystemTick();
}

void perflog_logic_end(void) {
  if (!armed() || !g_logic_t0)
    return;
  const u64 d = armGetSystemTick() - g_logic_t0;
  __atomic_fetch_add(&g_logic_n, 1, __ATOMIC_RELAXED);
  __atomic_fetch_add(&g_logic_sum, d, __ATOMIC_RELAXED);
  max64(&g_logic_max, d);
  if (p_vis) {
    const int v = *(volatile int *)p_vis;
    if (v > 0) {
      __atomic_fetch_add(&g_vis_sum, (u64)v, __ATOMIC_RELAXED);
      max32(&g_vis_max, (u32)v);
    }
  }
}

uint64_t perflog_swap_begin(void) {
  if (!armed())
    return 0;
  const u64 now = armGetSystemTick();
  if (g_last_swap) {
    const u64 d = now - g_last_swap;
    __atomic_fetch_add(&g_frames, 1, __ATOMIC_RELAXED);
    __atomic_fetch_add(&g_ft_sum, d, __ATOMIC_RELAXED);
    max64(&g_ft_max, d);
    if (d > TICKS_20MS)
      __atomic_fetch_add(&g_over20, 1, __ATOMIC_RELAXED);
    if (d > TICKS_36MS)
      __atomic_fetch_add(&g_over36, 1, __ATOMIC_RELAXED);
  }
  g_last_swap = now;
  return now;
}

void perflog_swap_end(uint64_t t0) {
  if (!t0)
    return;
  const u64 d = armGetSystemTick() - t0;
  __atomic_fetch_add(&g_swap_sum, d, __ATOMIC_RELAXED);
  max64(&g_swap_max, d);
}

// ---------------------------------------------------------------------------
// writer

static u32 clk_mhz(ClkrstSession *s) {
  u32 hz = 0;
  if (R_FAILED(clkrstGetClockRate(s, &hz)))
    return 0;
  return hz / 1000000;
}

static void clocks_init(void) {
  if (!hosversionAtLeast(8, 0, 0) || R_FAILED(clkrstInitialize()))
    return;
  if (R_FAILED(clkrstOpenSession(&g_clk_cpu, PcvModuleId_CpuBus, 3)))
    goto fail;
  if (R_FAILED(clkrstOpenSession(&g_clk_gpu, PcvModuleId_GPU, 3)))
    goto fail_cpu;
  if (R_FAILED(clkrstOpenSession(&g_clk_emc, PcvModuleId_EMC, 3)))
    goto fail_gpu;
  g_clk_ok = 1;
  return;
fail_gpu:
  clkrstCloseSession(&g_clk_gpu);
fail_cpu:
  clkrstCloseSession(&g_clk_cpu);
fail:
  clkrstExit();
}

static void clocks_exit(void) {
  if (!g_clk_ok)
    return;
  g_clk_ok = 0;
  clkrstCloseSession(&g_clk_emc);
  clkrstCloseSession(&g_clk_gpu);
  clkrstCloseSession(&g_clk_cpu);
  clkrstExit();
}

static const char *model_name(void) {
  SetSysProductModel m = SetSysProductModel_Invalid;
  if (R_SUCCEEDED(setsysInitialize())) {
    setsysGetProductModel(&m);
    setsysExit();
  }
  switch (m) {
    case SetSysProductModel_Nx:   return "Erista/V1";
    case SetSysProductModel_Iowa: return "Mariko/V2";
    case SetSysProductModel_Hoag: return "Lite";
    case SetSysProductModel_Aula: return "OLED";
    default:                      return "unknown";
  }
}

static const char *mode_name(void) {
  return appletGetOperationMode() == AppletOperationMode_Console ? "docked" : "handheld";
}

static void write_header(void) {
  char ts[32] = "?";
  const time_t now = time(NULL);
  const struct tm *tm = localtime(&now);
  if (tm)
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm);
  const u32 v = hosversionGet();
  fprintf(g_file, "\n=== gtasa_nx %s | %s | fw %u.%u.%u | %s | %s | render %dx%d\n",
          APP_VERSION_STR, ts, HOSVER_MAJOR(v), HOSVER_MINOR(v), HOSVER_MICRO(v),
          model_name(), mode_name(), screen_width, screen_height);
  fprintf(g_file, "cfg gl_backend=%d glthread=%d gallium_thread=%d render_core_mask=%d "
                  "engine_core_mask=%d mesa_core_mask=%d mesa_prio=%d fps_cap_30=%d "
                  "no_offscreen_despawn=%d ps2_color_filter=%d ps2_corona_rotation=%d "
                  "trilinear_filter=%d disable_ped_spec=%d\n",
          config.gl_backend, config.glthread, config.gallium_thread,
          config.render_core_mask, config.engine_core_mask, config.mesa_core_mask,
          config.mesa_prio, config.fps_cap_30, config.no_offscreen_despawn,
          config.ps2_color_filter, config.ps2_corona_rotation,
          config.trilinear_filter, config.disable_ped_spec);
  fprintf(g_file, "syms vis=%d lods=%d lod=%d lowlod=%d peds=%d cars=%d clocks=%d\n",
          p_vis != NULL, p_lods != NULL, p_lod_scale != NULL, p_lowlod_scale != NULL,
          p_peds != NULL, p_cars != NULL, g_clk_ok);
  fprintf(g_file, "# t=s since start | fps | ft=frame interval avg/max ms, frames >20ms/>36ms"
                  " | swap=eglSwapBuffers avg/max ms | logic=implOnDrawFrame avg/max ms"
                  " | thread=%%CPU of one core (core:%% when split) | c0..c2=listed threads"
                  " per core | vis=visible entities avg/max, lods | lod=lodDistScale/low"
                  " | peds cars | clk=CPU/GPU/MEM MHz | h/d=handheld/docked\n");
  fflush(g_file);
}

#define APPEND(...)                                                          \
  do {                                                                       \
    if (len < (int)sizeof(line))                                             \
      len += snprintf(line + len, sizeof(line) - (size_t)len, __VA_ARGS__);  \
  } while (0)

static void write_line(u64 now, u64 dt) {
  const double sec = (double)dt / (double)armGetSystemTickFreq();
  const u32 frames = take32(&g_frames);
  const u32 over20 = take32(&g_over20), over36 = take32(&g_over36);
  const u64 ft_sum = take64(&g_ft_sum), ft_max = take64(&g_ft_max);
  const u64 sw_sum = take64(&g_swap_sum), sw_max = take64(&g_swap_max);
  const u32 logic_n = take32(&g_logic_n);
  const u64 logic_sum = take64(&g_logic_sum), logic_max = take64(&g_logic_max);
  const u64 vis_sum = take64(&g_vis_sum);
  const u32 vis_max = take32(&g_vis_max);

  char line[1536];
  int len = 0;
  APPEND("t=%.1f fps=%.1f ft=%.1f/%.1f >20ms=%u >36ms=%u swap=%.1f/%.1f logic=%.1f/%.1f |",
         ticks_ms(now - g_t0) / 1000.0, frames / sec,
         frames ? ticks_ms(ft_sum) / frames : 0.0, ticks_ms(ft_max), over20, over36,
         frames ? ticks_ms(sw_sum) / frames : 0.0, ticks_ms(sw_max),
         logic_n ? ticks_ms(logic_sum) / logic_n : 0.0, ticks_ms(logic_max));

  u64 core_sum[PL_CORES] = { 0 };
  int n = __atomic_load_n(&g_nthr, __ATOMIC_ACQUIRE);
  if (n > PL_MAX_THREADS)
    n = PL_MAX_THREADS;
  for (int i = 0; i < n; i++) {
    PlThread *t = &g_thr[i];
    if (!__atomic_load_n(&t->ready, __ATOMIC_ACQUIRE))
      continue;
    u64 cur[PL_CORES];
    if (!thread_ticks(t->handle, cur)) {
      APPEND(" %s=x", t->name);
      continue;
    }
    if (!t->seen) {
      memcpy(t->last, cur, sizeof(cur));
      t->seen = 1;
      APPEND(" %s=new", t->name);
      continue;
    }
    u64 d[PL_CORES], total = 0;
    int cores_used = 0;
    for (int c = 0; c < PL_CORES; c++) {
      d[c] = cur[c] - t->last[c];
      t->last[c] = cur[c];
      total += d[c];
      core_sum[c] += d[c];
      if (d[c] * 100 >= dt)
        cores_used++;
    }
    APPEND(" %s=%u", t->name, (unsigned)((total * 100 + dt / 2) / dt));
    if (cores_used > 1) {
      APPEND("(");
      for (int c = 0, first = 1; c < PL_CORES; c++)
        if (d[c] * 100 >= dt) {
          APPEND("%s%d:%u", first ? "" : ",", c, (unsigned)((d[c] * 100 + dt / 2) / dt));
          first = 0;
        }
      APPEND(")");
    }
  }
  APPEND(" | c0=%u c1=%u c2=%u",
         (unsigned)((core_sum[0] * 100 + dt / 2) / dt),
         (unsigned)((core_sum[1] * 100 + dt / 2) / dt),
         (unsigned)((core_sum[2] * 100 + dt / 2) / dt));

  APPEND(" |");
  if (p_vis)
    APPEND(" vis=%u/%u", logic_n ? (unsigned)(vis_sum / logic_n) : 0u, vis_max);
  if (p_lods)
    APPEND(" lods=%d", *(volatile int *)p_lods);
  if (p_lod_scale)
    APPEND(" lod=%.2f", (double)*(volatile float *)p_lod_scale);
  if (p_lowlod_scale)
    APPEND("/%.2f", (double)*(volatile float *)p_lowlod_scale);
  if (p_peds)
    APPEND(" peds=%d", *(volatile int *)p_peds);
  if (p_cars)
    APPEND(" cars=%d", *(volatile int *)p_cars);
  if (g_clk_ok)
    APPEND(" | clk=%u/%u/%u", clk_mhz(&g_clk_cpu), clk_mhz(&g_clk_gpu), clk_mhz(&g_clk_emc));
  APPEND(" %s", appletGetOperationMode() == AppletOperationMode_Console ? "d" : "h");

  fputs(line, g_file);
  fputc('\n', g_file);
  fflush(g_file);
}

static void writer_main(void *arg) {
  (void)arg;
  u64 last = armGetSystemTick();
  for (;;) {
    const Result rc = waitSingle(waiterForUEvent(&g_stop_event), PL_PERIOD_NS);
    const u64 now = armGetSystemTick();
    if (now > last)
      write_line(now, now - last);
    last = now;
    if (R_SUCCEEDED(rc))
      break; // stop requested; the partial window above is the last line
  }
  fputs("=== end\n", g_file);
  fflush(g_file);
}

// ---------------------------------------------------------------------------

void perflog_init(void) {
  if (!config.perf_log || armed())
    return;
  g_file = fopen(PL_FILE, "a");
  if (!g_file) {
    debugPrintf("perflog: cannot open %s\n", PL_FILE);
    return;
  }
  if (!hosversionAtLeast(13, 0, 0))
    g_tick_info = InfoType_ThreadTickCountDeprecated;

  p_vis  = (int *)so_try_find_addr_rx(&game_mod, "_ZN9CRenderer23ms_nNoOfVisibleEntitiesE");
  p_lods = (int *)so_try_find_addr_rx(&game_mod, "_ZN9CRenderer19ms_nNoOfVisibleLodsE");
  p_lod_scale = (float *)so_try_find_addr_rx(&game_mod, "_ZN9CRenderer15ms_lodDistScaleE");
  p_lowlod_scale =
      (float *)so_try_find_addr_rx(&game_mod, "_ZN9CRenderer18ms_lowLodDistScaleE");
  p_peds = (int *)so_try_find_addr_rx(&game_mod, "_ZN11CPopulation13ms_nTotalPedsE");
  p_cars = (int *)so_try_find_addr_rx(&game_mod, "_ZN8CCarCtrl13NumRandomCarsE");

  clocks_init();
  write_header();

  __atomic_store_n(&g_armed, 1, __ATOMIC_RELEASE);
  perflog_register_thread("main");
}

void perflog_start(void) {
  if (!armed() || g_writer_running)
    return;
  ueventCreate(&g_stop_event, false);
  g_t0 = armGetSystemTick();
  // lowest priority every pthread already uses; core 2 keeps it off the
  // logic (0) and RenderQueue (1) cores
  Result rc = threadCreate(&g_writer, writer_main, NULL, NULL, 0x8000, 0x3B, 2);
  if (R_FAILED(rc))
    rc = threadCreate(&g_writer, writer_main, NULL, NULL, 0x8000, 0x3B, -2);
  if (R_FAILED(rc)) {
    debugPrintf("perflog: writer thread failed: %08x\n", rc);
    return;
  }
  if (R_FAILED(threadStart(&g_writer))) {
    threadClose(&g_writer);
    return;
  }
  g_writer_running = 1;
}

void perflog_stop(void) {
  if (!g_writer_running)
    return;
  g_writer_running = 0;
  ueventSignal(&g_stop_event);
  if (R_SUCCEEDED(waitSingleHandle(g_writer.handle, 500000000ULL))) {
    threadClose(&g_writer);
    fclose(g_file);
    g_file = NULL;
  } else {
    // stuck in SD I/O: freeze it like the engine threads
    svcSetThreadActivity(g_writer.handle, ThreadActivity_Paused);
  }
  clocks_exit();
}
