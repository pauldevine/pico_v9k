#!/bin/bash
#
# build.sh - macOS build script for DOS DMA test programs
# Cross-compiles DOS executables using OpenWatcom on macOS
#

# OpenWatcom paths - adjust these if your installation is different
WATCOM_PATH="/Users/pauldevine/projects/open-watcom-v2"
WATCOM_INC="${WATCOM_PATH}/rel/h"
WATCOM_LIB="${WATCOM_PATH}/rel/lib286/dos"

# Compiler and linker commands for macOS
CC="${WATCOM_PATH}/build/binbuild/bwcc"
LINK="${WATCOM_PATH}/build/binbuild/bwlink"

# Compiler flags
# -bt=dos    : Build for DOS target
# -ms        : Small memory model
# -0         : 8088/8086 instructions only
# -os        : Optimize for size
# -zq        : Quiet mode
# -za99      : Enable C99 standard
# -d0        : No debug info (change to -d2 for debug)
CFLAGS="-bt=dos -ms -0 -os -zq -za99 -d0 -I${WATCOM_INC}"
CFLAGS_DEBUG="-bt=dos -ms -0 -od -zq -za99 -d2 -I${WATCOM_INC}"

# Linker flags
LFLAGS="system dos LIBPATH ${WATCOM_LIB}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[BUILD]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Function to compile a C file
compile() {
    local source=$1
    local object="${source%.c}.o"

    print_status "Compiling ${source}..."
    if ${CC} ${CFLAGS} ${source}; then
        print_status "  -> ${object} created"
        return 0
    else
        print_error "Failed to compile ${source}"
        return 1
    fi
}

# Function to link an executable
link() {
    local exe=$1
    local obj=$2

    print_status "Linking ${exe}..."
    if ${LINK} ${LFLAGS} name ${exe} file ${obj}; then
        print_status "  -> ${exe} created"
        return 0
    else
        print_error "Failed to link ${exe}"
        return 1
    fi
}

# Function to clean build artifacts
clean() {
    print_status "Cleaning build artifacts..."
    rm -f *.o *.obj *.err *.map *.exe
    print_status "Clean complete"
}

# Function to build all programs
build_all() {
    local success=true

    # Build dmatest.exe
    if compile dmatest.c; then
        if ! link dmatest.exe dmatest.o; then
            success=false
        fi
    else
        success=false
    fi

    # Build addrtest.exe
    if compile addrtest.c; then
        if ! link addrtest.exe addrtest.o; then
            success=false
        fi
    else
        success=false
    fi

    # Build minimal.exe
    if compile minimal.c; then
        if ! link minimal.exe minimal.o; then
            success=false
        fi
    else
        success=false
    fi

    if $success; then
        print_status "Build complete! DOS executables ready:"
        ls -la *.exe
    else
        print_error "Build failed with errors"
        return 1
    fi
}

# Function to build with debug symbols
build_debug() {
    CFLAGS="${CFLAGS_DEBUG}"
    print_warning "Building with debug symbols..."
    build_all
}

# Main script logic
main() {
    # Check if OpenWatcom tools are available
    if ! command -v ${CC} &> /dev/null; then
        print_error "OpenWatcom compiler (${CC}) not found in PATH"
        print_error "Please ensure OpenWatcom for macOS is installed and in PATH"
        exit 1
    fi

    if ! command -v ${LINK} &> /dev/null; then
        print_error "OpenWatcom linker (${LINK}) not found in PATH"
        print_error "Please ensure OpenWatcom for macOS is installed and in PATH"
        exit 1
    fi

    # Check if OpenWatcom paths exist
    if [ ! -d "${WATCOM_INC}" ]; then
        print_error "OpenWatcom include path not found: ${WATCOM_INC}"
        print_error "Please adjust WATCOM_PATH in this script"
        exit 1
    fi

    if [ ! -d "${WATCOM_LIB}" ]; then
        print_error "OpenWatcom library path not found: ${WATCOM_LIB}"
        print_error "Please adjust WATCOM_PATH in this script"
        exit 1
    fi

    # Parse command line arguments
    case "${1:-all}" in
        all)
            build_all
            ;;
        clean)
            clean
            ;;
        debug)
            build_debug
            ;;
        dmatest)
            if compile dmatest.c; then
                link dmatest.exe dmatest.o
            fi
            ;;
        addrtest)
            if compile addrtest.c; then
                link addrtest.exe addrtest.o
            fi
            ;;
        minimal)
            if compile minimal.c; then
                link minimal.exe minimal.o
            fi
            ;;
        help|--help|-h)
            echo "Usage: $0 [command]"
            echo ""
            echo "Commands:"
            echo "  all      - Build all test programs (default)"
            echo "  clean    - Remove all build artifacts"
            echo "  debug    - Build with debug symbols"
            echo "  dmatest  - Build only dmatest.exe"
            echo "  addrtest - Build only addrtest.exe"
            echo "  minimal  - Build only minimal.exe (single register test)"
            echo "  help     - Show this help message"
            echo ""
            echo "Environment:"
            echo "  WATCOM_PATH = ${WATCOM_PATH}"
            echo "  Compiler    = ${CC}"
            echo "  Linker      = ${LINK}"
            ;;
        *)
            print_error "Unknown command: $1"
            echo "Run '$0 help' for usage information"
            exit 1
            ;;
    esac
}

# Run the main function with all arguments
main "$@"