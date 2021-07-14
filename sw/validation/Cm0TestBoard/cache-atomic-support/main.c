/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <support.h>

/* ------ Macros ------------------------------------------------------------*/

/* ------ Types -------------------------------------------------------------*/
struct UndoLogEntry {
  unsigned address;
  uint8_t data[8];  // TODO get the linewidth from config
};

struct CacheControllerStruct {
  volatile unsigned CSR;
  volatile unsigned CRNTDIRTY;
  volatile unsigned MAXDIRTY;
  volatile unsigned LOGADDR;
}

/* ------ Extern functions --------------------------------------------------*/

/* ------ Function Prototypes -----------------------------------------------*/

/* ------ Variable Declarations ---------------------------------------------*/

const uint8_t before_data[] =
    " Lorem ipsum dolor sit amet, consectetur adipiscing elit. Suspendisse "
    "sagittis justo ut sem lacinia imperdiet. Donec in diam erat. Sed ornare "
    "orci nunc, finibus feugiat nibh mollis et. Phasellus pharetra ante vitae "
    "elit porta dignissim. Mauris malesuada massa dolor, eget dictum neque "
    "condimentum eu. Nullam sit amet placerat mauris, nec pulvinar sapien. "
    "Morbi porttitor, odio ac bibendum finibus, justo nisl porta mauris, at "
    "fermentum sapien ante vel dui. Morbi tincidunt mauris interdum congue "
    "vestibulum. Quisque magna elit, pulvinar vel odio ut, bibendum eleifend "
    "ligula. Phasellus non nulla felis. Suspendisse elementum mauris lorem, a "
    "laoreet lacus pretium nec. Sed dapibus consectetur mi, volutpat consequat "
    "nisi cursus nec. Donec at nibh eget elit viverra porta ac ullamcorper "
    "justo. In porta metus lectus, id pulvinar dolor dictum quis.  Praesent "
    "vitae ornare ligula. Maecenas eleifend sodales rutrum. Aenean quam orci, "
    "mattis sed eleifend a, viverra ut lorem. Curabitur consectetur efficitur "
    "tellus, vitae erat curae. ";

const uint8_t fase_data[] =
    "At vero eos et accusamus et iusto odio dignissimos ducimus qui blanditiis "
    "praesentium voluptatum deleniti atque corrupti quos dolores et quas "
    "molestias excepturi sint occaecati cupiditate non provident, similique "
    "sunt in culpa qui officia deserunt mollitia animi, id est laborum et "
    "dolorum fuga.";

static uint8_t nvram_data[sizeof(before_data)];

static struct UndoLogEntry undo_log[64];

static struct CacheControllerStruct* const CacheController =
    (struct CacheControllerStruct* const)DCACHE_CTRL_BASE;

/* ------ Function Declarations ---------------------------------------------*/

void fused_assert(bool c) {
  if (!c) {
    indicate_test_fail();
  }
}

int main(void) {
  // Copy over before_data
  memcpy(nvram_data, before_data, sizeof(before_data));

  // Start Failure-Atomic Section
  CacheController->LOGADDR = (volatile unsigned int)&undo_log;
  CacheController->CSR |= DCACHE_CTRL_FASE;

  // Copy over fase_data
  memcpy(nvram_data, fase_data, sizeof(fase_data));

  // End Failure-Atomic Section -> Simulate reboot by software-resetting cache

  // Restore NVM state from undo-log

  for (unsigned addr = &undo_log; addr < CacheController->LOGADDR;
       addr += sizeof(struct UndoLogEntry)) {
    struct UndoLogEntry* entry = (struct UndoLogEntry*)addr;
    memcpy(entry->address, entry->data, sizeof(entry->data));
  }

  end_experiment();
  return 0;
}
