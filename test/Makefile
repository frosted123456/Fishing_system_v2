# Host unit tests with plain g++ (no PlatformIO needed): make -C test
# The same files run under PlatformIO with the real Unity: pio test -e native
# -std=c++11 on purpose: Arduino-ESP32 core 2.0.17 builds with gnu++11, so lib/IceMesh must stay C++11.
CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -pedantic -O1 -g -pthread
INC      := -I../lib/IceMesh/src -Ishim
TESTS    := $(patsubst %/,%,$(wildcard test_*/))
BINS     := $(addprefix bin/,$(TESTS))
HDRS     := $(wildcard ../lib/IceMesh/src/*.h) shim/unity.h

.PHONY: all clean sanitize
all: $(BINS)
	@set -e; for t in $(BINS); do echo "=== $$t"; ./$$t; done

bin/%: %/test_main.cpp $(HDRS)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(INC) $< -o $@

# Same tests at -O0 with AddressSanitizer + UndefinedBehaviorSanitizer
sanitize:
	@mkdir -p bin/san; set -e; for t in $(TESTS); do \
	  $(CXX) -std=c++11 -Wall -Wextra -Werror -pedantic -O0 -g -fsanitize=address,undefined \
	    -fno-sanitize-recover=all -pthread $(INC) $$t/test_main.cpp -o bin/san/$$t; \
	  echo "=== san/$$t"; ./bin/san/$$t | tail -1; done

clean:
	rm -rf bin
