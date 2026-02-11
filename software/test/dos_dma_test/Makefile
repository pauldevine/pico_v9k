# Makefile for DOS DMA test programs (macOS cross-compilation)
#
# Usage: make [target]
#

# OpenWatcom paths - adjust if needed
WATCOM_PATH = /Users/pauldevine/projects/open-watcom-v2
WATCOM_INC = $(WATCOM_PATH)/rel/h
WATCOM_LIB = $(WATCOM_PATH)/rel/lib286/dos

# Compiler and linker for macOS
CC = bwcc
LINK = bwlink

# Compiler flags
CFLAGS = -bt=dos -ms -0 -os -zq -za99 -d0 -I$(WATCOM_INC)
CFLAGS_DEBUG = -bt=dos -ms -0 -od -zq -za99 -d2 -I$(WATCOM_INC)

# Linker flags
# Use 'format dos' instead of 'system dos' to avoid W1107 warning
# Include both DOS-specific and general 286 library paths for math libraries
LFLAGS = format dos LIBPATH $(WATCOM_LIB) LIBPATH $(WATCOM_PATH)/rel/lib286

# Target executables
TARGETS = dmatest.exe addrtest.exe addr_mid.exe minimal.exe read.exe victor_boot_trace.exe sasi_perf_test.exe dma_verify.exe boot_sequence_test.exe hd_stress.exe

# Source and object files
SOURCES = dmatest.c addrtest.c addr_mid.c minimal.c read.c victor_boot_trace.c sasi_perf_test.c dma_verify.c boot_sequence_test.c hd_stress.c
OBJECTS = dmatest.o addrtest.o addr_mid.o minimal.o read.o victor_boot_trace.o sasi_perf_test.o dma_verify.o boot_sequence_test.o hd_stress.o

# Default target
.PHONY: all
all: $(TARGETS)

# Build rules for executables
dmatest.exe: dmatest.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

addrtest.exe: addrtest.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

addr_mid.exe: addr_mid.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

minimal.exe: minimal.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

read.exe: read.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

victor_boot_trace.exe: victor_boot_trace.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

sasi_perf_test.exe: sasi_perf_test.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

dma_verify.exe: dma_verify.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

boot_sequence_test.exe: boot_sequence_test.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

hd_stress.exe: hd_stress.o
	@echo "Linking $@..."
	@$(LINK) $(LFLAGS) name $@ file $<

# Compile rule for C files
%.o: %.c
	@echo "Compiling $<..."
	@$(CC) $(CFLAGS) $<

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	@rm -f *.o *.obj *.err *.map *.exe
	@echo "Clean complete"

# Debug build
.PHONY: debug
debug: CFLAGS = $(CFLAGS_DEBUG)
debug: clean all
	@echo "Debug build complete"

# Run tests (requires DOS emulator)
.PHONY: test
test: all
	@echo "Test executables built:"
	@ls -la *.exe
	@echo ""
	@echo "To run tests, copy *.exe files to DOS environment or use DOSBox:"
	@echo "  dosbox -c 'mount c .' -c 'c:' -c 'dmatest.exe'"

# Run address test only
.PHONY: test-addr
test-addr: addrtest.exe
	@echo "Address test built: addrtest.exe"
	@echo "Copy to DOS environment to run"

# Run minimal test only
.PHONY: test-min
test-min: minimal.exe
	@echo "Minimal test built: minimal.exe"
	@echo "Copy to DOS environment to run"

# Run read test only
.PHONY: test-read
test-read: read.exe
	@echo "Read test built: read.exe"
	@echo "Copy to DOS environment to run"

# Run boot sequence test only
.PHONY: test-bootseq
test-bootseq: boot_sequence_test.exe
	@echo "Boot sequence test built: boot_sequence_test.exe"
	@echo "Copy to DOS environment to run"

# Show help
.PHONY: help
help:
	@echo "DOS DMA Test Programs - macOS Build"
	@echo ""
	@echo "Targets:"
	@echo "  all        - Build all test programs (default)"
	@echo "  clean      - Remove all build artifacts"
	@echo "  debug      - Build with debug symbols"
	@echo "  test       - Build and show run instructions"
	@echo "  test-addr  - Build address test only"
	@echo "  test-bootseq - Build boot sequence harness only"
	@echo "  help       - Show this help message"
	@echo ""
	@echo "Individual programs:"
	@echo "  dmatest.exe  - Full DMA board test suite"
	@echo "  addrtest.exe - Address register focused test"
	@echo "  addr_mid.exe - Minimal DMA_ADDR_MID write/read tester"
	@echo "  minimal.exe  - Minimal single register test (REG_ADDR_H)"
	@echo "  victor_boot_trace.exe - Replay first 20 BIOS accesses from MAME log"
	@echo "  sasi_perf_test.exe - SASI state machine perf test (250 ops from MAME)"
	@echo "  dma_verify.exe - DMA transfer CRC8 verification (10 sectors)"
	@echo "  boot_sequence_test.exe - Replay full observed boot SASI sequence with divergence checks"
	@echo "  hd_stress.exe  - DOS file I/O stress test (25 small + 1 large file)"
	@echo ""
	@echo "Environment:"
	@echo "  WATCOM_PATH = $(WATCOM_PATH)"
	@echo "  Compiler    = $(CC)"
	@echo "  Linker      = $(LINK)"

# Verbose build
.PHONY: verbose
verbose:
	$(MAKE) --no-print-directory VERBOSE=1 all

# Check dependencies
.PHONY: check
check:
	@echo "Checking build environment..."
	@which $(CC) > /dev/null 2>&1 && echo "✓ Compiler found: $$(which $(CC))" || echo "✗ Compiler not found: $(CC)"
	@which $(LINK) > /dev/null 2>&1 && echo "✓ Linker found: $$(which $(LINK))" || echo "✗ Linker not found: $(LINK)"
	@test -d "$(WATCOM_INC)" && echo "✓ Include path exists: $(WATCOM_INC)" || echo "✗ Include path missing: $(WATCOM_INC)"
	@test -d "$(WATCOM_LIB)" && echo "✓ Library path exists: $(WATCOM_LIB)" || echo "✗ Library path missing: $(WATCOM_LIB)"

# Install to DOS directory (customize the path)
DOS_DIR = ~/dos_programs
.PHONY: install
install: all
	@mkdir -p $(DOS_DIR)
	@cp *.exe $(DOS_DIR)/
	@echo "Installed to $(DOS_DIR):"
	@ls -la $(DOS_DIR)/*.exe
