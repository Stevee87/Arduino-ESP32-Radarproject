# Patch for Stevee87/Radar-project-Uconsole

This directory holds a fix for a different repository by the same author,
`Radar-project-Uconsole`, kept here because this fork has no fork of that repo
to push to. The patch changes one line in `Firmware/uconsole_radar_receiver.py`.

**Bug.** The XIAO transmitter sends a packed `UdpPacket` whose `struct Target`
is not packed; the compiler pads each target to 8 bytes, so the datagram is
32 bytes. The Python receiver unpacks it with `"<II" + "hhhB" * 3` (29 bytes),
which reads target 2 one byte early and target 3 two bytes early. With one
person in view (the usual case, after the firmware's 1 m clustering) nothing
looks wrong; with two or three, targets 2 and 3 are garbage.

**Fix.** `PACKET_FMT = "<II" + "hhhBx" * MAX_TARGETS` (32 bytes).

To open the pull request:

```bash
# 1. fork https://github.com/Stevee87/Radar-project-Uconsole on GitHub, then
git clone https://github.com/<you>/Radar-project-Uconsole
cd Radar-project-Uconsole
git checkout -b fix/receiver-packet-format
git am /path/to/Arduino-ESP32-Radarproject/upstream-patches/Radar-project-Uconsole/0001-*.patch
git push -u origin fix/receiver-packet-format
# 2. open the PR against Stevee87/Radar-project-Uconsole from the GitHub UI
```

Verification used for the patch: `python3 -c 'import struct; print(struct.calcsize("<II"+"hhhBx"*3))'`
prints 32, matching `sizeof(UdpPacket)` from the XIAO sketch compiled for the ESP32-S3.
