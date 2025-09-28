# Building DOS DMA Test Programs on macOS

This directory contains DOS test programs that can be cross-compiled on macOS using OpenWatcom.

## Prerequisites

1. **OpenWatcom for macOS** - Install the macOS version of OpenWatcom
2. **Ensure tools are in PATH** - `bwcc` and `bwlink` should be accessible

## Build Methods

### Method 1: Using build.sh (Recommended)

```bash
# Build all programs
./build.sh

# Build specific program
./build.sh dmatest
./build.sh addrtest

# Build with debug symbols
./build.sh debug

# Clean build artifacts
./build.sh clean

# Show help
./build.sh help
```

### Method 2: Using Make

```bash
# Build all programs
make

# Build specific program
make dmatest.exe
make addrtest.exe

# Build with debug symbols
make debug

# Clean build artifacts
make clean

# Check build environment
make check

# Show help
make help
```

### Method 3: Manual Compilation

```bash
# Compile
bwcc -bt=dos -ms -0 -os -zq -d0 -I/Users/pauldevine/projects/open-watcom-v2/rel/h dmatest.c

# Link
bwlink system dos LIBPATH /Users/pauldevine/projects/open-watcom-v2/rel/lib286/dos name dmatest.exe file dmatest.o
```

## Configuration

If your OpenWatcom installation is in a different location, update the paths:

1. In `build.sh`, modify:
   ```bash
   WATCOM_PATH="/Users/pauldevine/projects/open-watcom-v2"
   ```

2. In `Makefile`, modify:
   ```make
   WATCOM_PATH = /Users/pauldevine/projects/open-watcom-v2
   ```

## Output Files

The build process creates:
- `*.exe` - DOS executable files
- `*.o` - Object files (intermediate)
- `*.map` - Linker map files (if debug enabled)

## Running the Programs

After building, copy the `.exe` files to your DOS environment:

1. **Real Hardware**: Copy to floppy/CF card/network share
2. **DOSBox**:
   ```bash
   dosbox -c "mount c ." -c "c:" -c "dmatest.exe"
   ```
3. **QEMU/VMware**: Copy to virtual disk image

## Troubleshooting

### "bwcc: command not found"
- Ensure OpenWatcom for macOS is installed
- Add OpenWatcom bin directory to PATH:
  ```bash
  export PATH="/path/to/openwatcom/binm:$PATH"
  ```

### Include/Library path errors
- Update `WATCOM_PATH` in build scripts
- Run `make check` to verify paths

### Build errors
- Ensure source files have DOS line endings (CRLF)
- Check that you're using the DOS target libraries (`lib286/dos`)

## Programs

- **dmatest.exe** - Comprehensive DMA board test suite
- **addrtest.exe** - Focused address register test

## Notes

- The programs are compiled for 8088/8086 compatibility (`-0` flag)
- Small memory model is used (`-ms` flag)
- Optimization for size (`-os` flag) to keep executables small
- Debug builds use `-od` (disable optimization) and `-d2` (full debug info)