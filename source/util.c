/* util.c -- misc utility functions
 *
 * Copyright (C) 2021 fgsfds, Andy Nguyen
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <switch.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include "util.h"
#include "config.h"

#ifdef DEBUG_LOG

static int s_nxlinkSock = -1;

static void initNxLink(void) {
  if (R_FAILED(socketInitializeDefault()))
    return;
  s_nxlinkSock = nxlinkStdio();
  if (s_nxlinkSock < 0)
    socketExit();
}

static void deinitNxLink(void) {
  if (s_nxlinkSock >= 0) {
    close(s_nxlinkSock);
    socketExit();
    s_nxlinkSock = -1;
  }
}

void userAppInit(void) {
  initNxLink();
}

void userAppExit(void) {
  deinitNxLink();
}

#endif

// the game's `printf` import points here; a no-op with DEBUG_LOG off. The log
// file is kept open for the run (reopening per line on FAT is slow) and flushed
// each line to survive an abrupt exit.
int debugPrintf(char *text, ...) {
#ifdef DEBUG_LOG
  va_list list;
  static FILE *f = NULL;
  if (!f)
    f = fopen(LOG_NAME, "w"); // fresh log each boot
  if (f) {
    va_start(list, text);
    vfprintf(f, text, list);
    va_end(list);
    fflush(f);
  }
  va_start(list, text);
  vprintf(text, list); // also to nxlink stdout, if a host is connected
  va_end(list);
#endif
  return 0;
}

// boost the CPU to 1785MHz while loading
void cpu_boost(int on) {
  appletSetCpuBoostMode(on ? ApmCpuBoostMode_FastLoad : ApmCpuBoostMode_Normal);
}

// The NRO links with -Wl,--wrap=svcSetThreadCoreMask (see the Makefile and
// __wrap_svcSetThreadCoreMask below); our own pinning calls the real SVC.
Result __real_svcSetThreadCoreMask(Handle handle, s32 preferred_core, u32 affinity_mask);

// Cores granted to this process (cores 0..2 for an application; core 3 is the
// system core).
static u64 process_core_mask(void) {
  static u64 mask = 0;
  if (mask == 0)
    svcGetInfo(&mask, InfoType_CoreMask, CUR_PROCESS_HANDLE, 0);
  return mask;
}

// Pin the calling thread to the allowed part of `mask`; an empty result leaves
// the thread on its current core.
void set_thread_core_mask(uint64_t mask) {
  mask &= process_core_mask();
  if (mask == 0)
    return;
  const s32 ideal = __builtin_ctzll(mask);
  Result rc = __real_svcSetThreadCoreMask(CUR_THREAD_HANDLE, ideal, (u32)mask);
  if (R_FAILED(rc))
    debugPrintf("affinity: pin to mask 0x%llx failed: %08x\n",
                (unsigned long long)mask, rc);
}

// pin the calling thread to a single core; an out-of-range request just leaves
// the thread on its default core.
void set_thread_core(int core) {
  if (core < 0 || core > 63)
    return;
  set_thread_core_mask(1ull << core);
}

// Mesa's Horizon port pins every util_queue worker it starts (the glthread
// worker, the disk-cache writer, zink's queues) to cores 1|2 with an ideal core
// of 1 or 2, from the worker itself (src/util/u_queue.c). Upstream's comment
// assumes the app renders on core 0; here the engine's RenderQueue thread owns
// core 1, so producer and glthread consumer end up sharing it. All these threads
// run at libnx's default priority 59, which Horizon round-robins every 10 ms.
// Intercept that exact call so mesa_core_mask / mesa_prio can move the workers.
#define MESA_WORKER_MASK ((1u << 1) | (1u << 2))

Result __wrap_svcSetThreadCoreMask(Handle handle, s32 preferred_core, u32 affinity_mask) {
  if (handle == CUR_THREAD_HANDLE && affinity_mask == MESA_WORKER_MASK) {
    if (config.mesa_prio > 0) {
      Result prc = svcSetThreadPriority(CUR_THREAD_HANDLE, (u32)config.mesa_prio);
      if (R_FAILED(prc))
        debugPrintf("mesa worker: priority %d failed: %08x\n", config.mesa_prio, prc);
    }
    const u64 mask = (u64)(u32)config.mesa_core_mask & process_core_mask();
    if (mask)
      return __real_svcSetThreadCoreMask(handle, __builtin_ctzll(mask), (u32)mask);
  }
  return __real_svcSetThreadCoreMask(handle, preferred_core, affinity_mask);
}

// Android AArch64 stack guards read their canary at TPIDR_EL0+0x28.
#define MAX_GAME_TLS_THREADS 64
#define GAME_TLS_SIZE 0x1000
#define GAME_TLS_GUARD UINT64_C(0x4242424242424242)

static uint8_t g_game_tls[MAX_GAME_TLS_THREADS][GAME_TLS_SIZE]
    __attribute__((aligned(GAME_TLS_SIZE)));
static unsigned g_game_tls_count;

void *game_tls_install(void) {
  unsigned slot = __atomic_fetch_add(&g_game_tls_count, 1, __ATOMIC_RELAXED);
  if (slot >= MAX_GAME_TLS_THREADS) {
    debugPrintf("TLS: exhausted %u dedicated game slots\n",
                MAX_GAME_TLS_THREADS);
    return NULL;
  }

  uint8_t *tls = g_game_tls[slot];
  memset(tls, 0, GAME_TLS_SIZE);
  const uint64_t guard = GAME_TLS_GUARD;
  memcpy(tls + 0x28, &guard, sizeof(guard));
  armSetTlsRw(tls);
  return tls;
}

// --- thread registry (see util.h) -----------------------------------------
#define MAX_TRACKED_THREADS 64
static Handle g_thread_handles[MAX_TRACKED_THREADS];
static int g_thread_count; // grow-only; index reserved with an atomic add

void thread_registry_add(void) {
  int i = __atomic_fetch_add(&g_thread_count, 1, __ATOMIC_RELAXED);
  if (i < MAX_TRACKED_THREADS)
    g_thread_handles[i] = threadGetCurHandle();
}

void thread_registry_pause_others(void) {
  Handle self = threadGetCurHandle();
  int n = g_thread_count;
  if (n > MAX_TRACKED_THREADS)
    n = MAX_TRACKED_THREADS;
  int paused = 0;
  for (int i = 0; i < n; i++) {
    Handle h = g_thread_handles[i];
    if (h && h != self && R_SUCCEEDED(svcSetThreadActivity(h, ThreadActivity_Paused)))
      paused++;
  }
  debugPrintf("EXIT: paused %d/%d engine threads\n", paused, n);
}

int ret0(void) { return 0; }

int retm1(void) { return -1; }
