#ifndef CARTEST_RX_FASTPATH_H
#define CARTEST_RX_FASTPATH_H

/* Keep the 2 Mbaud receive success path in the IRQ translation unit even in
   Debug builds. At 64 MHz one incoming 8N1 byte allows only 320 CPU cycles.
   Other controller and output code retains its configured optimization. */
#if defined(__GNUC__) && (defined(__arm__) || defined(__thumb__))
#define CAR_RX_INLINE inline __attribute__((always_inline, optimize("O2")))
#define CAR_RX_IRQ_OPTIMIZE __attribute__((optimize("O2")))
#elif defined(_MSC_VER)
#define CAR_RX_INLINE __forceinline
#define CAR_RX_IRQ_OPTIMIZE
#else
#define CAR_RX_INLINE inline
#define CAR_RX_IRQ_OPTIMIZE
#endif

#endif
