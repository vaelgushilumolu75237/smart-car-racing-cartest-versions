# OpenMV v5.0.1 JPEG preview codec guard patch

This is series patch 0005, applied after the unchanged 0004 path-acceleration patch. Its bytes are generated from the root-reviewed `jpeg_preview_guard/overlay` and pinned v5.0.1 base source.

The overlay adds a default 1000 ms encode timeout getter/setter, with a 1..1000 ms valid range. Decode waits retain their existing timeout. Encoder NLR paths release owned resources after safe cleanup and rethrow the original exception. The fail-closed abort path does not free DMA-visible buffers until HAL confirms stop.

The runtime caller may request a 5 ms encode budget, but HAL abort cleanup can still take about 1.013 seconds in the nominal running-tick case. This is not a 5 ms hard bound, not a hardware recovery guarantee, and not a hardware validation result.
