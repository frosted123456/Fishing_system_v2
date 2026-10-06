# test/

Host-side unit tests for `lib/IceMesh` (no ESP32 needed). One folder per suite: `test_<name>/test_main.cpp`.

- PlatformIO (real Unity): `pio test -e native`
- Plain g++ (what I use to verify): `make -C test` — builds with `-std=c++11 -pedantic -Werror`
  because the ESP32 Arduino core 2.0.17 compiles C++ as gnu++11. `shim/unity.h` is a tiny
  Unity-compatible stand-in used only by this Makefile.

Sonar suites: `test_sonar_codec` (round trip, escapes, truncation, fuzz, measured sizes on the fake
scene), `test_sonar_link` (hub outbox, chalet store, relay keeps whole blocks, FOCUS allowance);
`test_tdma_sim` also streams FOCUS from a remote hub through a relay at 0/10/20 % loss.
`test_sonar_proc`: C++ scene + processing vs the prototype (double, bit-exact, fixture from
`tools/proto_reference.js`); `test_sonar_proc_float`: same chain in float (as on the ESP32) + hole variants.
