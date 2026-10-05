# test/

Host-side unit tests for `lib/IceMesh` (no ESP32 needed). One folder per suite: `test_<name>/test_main.cpp`.

- PlatformIO (real Unity): `pio test -e native`
- Plain g++ (what I use to verify): `make -C test` — builds with `-std=c++11 -pedantic -Werror`
  because the ESP32 Arduino core 2.0.17 compiles C++ as gnu++11. `shim/unity.h` is a tiny
  Unity-compatible stand-in used only by this Makefile.
