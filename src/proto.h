// src/proto.h — compile-time protocol selector.
//
// One of PROTOCOL_HURRA or PROTOCOL_FERRUM must be defined (via the
// Makefile's PROTOCOL variable). This header aliases proto_* to the
// selected parser's symbols so kmbox.c can call into the protocol
// without #ifdefs at every call site.
#pragma once
#include <stdint.h>

#if defined(PROTOCOL_HURRA)
  #include "hurra.h"
  #define proto_init           hurra_init
  #define proto_feed_byte      hurra_feed_byte
  static inline void proto_feed(const uint8_t *b, uint16_t n) { hurra_feed(b, n); }
  #define proto_reset          hurra_reset
  #define proto_tick           hurra_tick
  #define proto_set_tx         hurra_set_tx
  typedef hurra_tx_fn proto_tx_fn;
  #define proto_notify_buttons hurra_notify_buttons
  #define proto_notify_axes    hurra_notify_axes
  #define proto_notify_keys    hurra_notify_keys
  #define PROTO_NAME "Hurra"
#elif defined(PROTOCOL_FERRUM)
  #include "ferrum.h"
  #define proto_init           ferrum_init
  #define proto_feed_byte      ferrum_feed_byte
  static inline void proto_feed(const uint8_t *b, uint16_t n) {
      for (uint16_t i = 0; i < n; i++) ferrum_feed_byte(b[i]);
  }
  #define proto_reset          ferrum_reset
  #define proto_tick           ferrum_tick
  #define proto_set_tx         ferrum_set_tx
  typedef ferrum_tx_fn proto_tx_fn;
  #define proto_notify_buttons ferrum_notify_buttons
  #define proto_notify_axes    ferrum_notify_axes
  #define proto_notify_keys    ferrum_notify_keys
  #define PROTO_NAME "Ferrum"
#else
  #error "Define PROTOCOL_FERRUM or PROTOCOL_HURRA (set PROTOCOL in Makefile)"
#endif
