/* perflog.h -- per-thread CPU time and frame timing log (config.perf_log)
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef __PERFLOG_H__
#define __PERFLOG_H__

#include <stdint.h>

// Arms the log when config.perf_log is set: resolves the engine counters in
// libGame.so, writes the run header and registers the calling (main) thread.
// Call once the game module is loaded and before the engine spawns threads.
void perflog_init(void);

// Starts the background writer (one line every ~2 s). Call right before the
// main loop, once the game module is mapped.
void perflog_start(void);

// Writes the last line and stops the writer; safe to call when not armed.
void perflog_stop(void);

// Registers the calling thread for CPU-time sampling; no-op unless armed.
void perflog_register_thread(const char *name);

// Same, naming the thread prefix0, prefix1, ... from a per-call-site counter.
void perflog_register_thread_numbered(const char *prefix, int *counter);

// Main thread: around implOnDrawFrame.
void perflog_logic_begin(void);
void perflog_logic_end(void);

// RenderQueue thread: around eglSwapBuffers. begin returns its timestamp.
uint64_t perflog_swap_begin(void);
void perflog_swap_end(uint64_t t0);

#endif
