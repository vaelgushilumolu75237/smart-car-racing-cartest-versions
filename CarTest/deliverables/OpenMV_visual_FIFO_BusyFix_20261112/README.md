# FIFO busy reply correction

Read-only ST-Link evidence matched the installed visual DebugRingFast ELF flash segments. Handshake and parameter ACK worked, vision frames advanced, but no STATE was cached. No flash/reset/output action was performed.

The primary 16-byte FIFO packet is queued immediately before sideband service. An immediate idle test can skip every fast iteration, starving replies. Sideband now polls completion for at most 2000 us (16 bytes at 115200 8N1 take 1389 us). A stuck FIFO preserves the queued packet and returns. Existing whole-frame ordinary writes handle packets over 16 bytes. No frame fragments may interleave.

Tests include initially busy FIFO completing and real HELLO/GET replies, stuck FIFO bounded return with packet retention, and full camera regressions. Hardware acceptance is pending. The bounded wait and synchronous long packets add latency; actual frame rate must be measured after installation. H743 is unchanged and paired ELFs retain their existing build evidence.

Install the matching OpenMV/wireless_sideband.py and OpenMV/main.py in the active camera directory. Preserve calibration files. H743 need not be flashed if the matching entry firmware is installed. Both Python files are required if the earlier FIFO-length fix was not installed.
