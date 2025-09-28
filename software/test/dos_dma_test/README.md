# DOS DMA Board Test Program

This is a DOS test program for testing the DMA board registers on the Victor 9000/Sirius 1.

## Overview

The test program accesses the DMA board registers at memory segment 0xEF30 (physical address 0xEF300-0xEF3FF) and performs various read/write tests to verify proper operation.

## Register Map

- **0x00-0x0F**: Control Register (write-only)
- **0x10-0x1F**: Data Register (read/write)
- **0x20-0x2F**: Status Register (read-only)
- **0x80-0x9F**: DMA Address Low Byte
- **0xA0-0xBF**: DMA Address Mid Byte
- **0xC0-0xDF**: DMA Address High Byte (4 bits)

## Building

### Requirements
- OpenWatcom C compiler (tested with 1.9 and 2.0)
- DOS or DOSBox environment

### Using wmake
```
wmake -f makefile.wc
```

### Using batch file
```
build.bat
```

### Manual compilation
```
wcc -bt=dos -ms -0 -os -zq dmatest.c
wlink system dos name dmatest.exe file dmatest.obj
```

## Running

### Automated test mode
```
DMATEST.EXE
```
This runs all tests automatically and provides a summary.

### Interactive mode
```
DMATEST.EXE -i
```
This allows manual register reads/writes for debugging.

## Test Coverage

1. **Control Register Tests**: Writes various bit patterns to control register
2. **Data Register Tests**: Read/write verification
3. **Status Register Tests**: Reads status and decodes SASI bus signals
4. **DMA Address Tests**: Tests 20-bit address register read/write
5. **Timing Tests**: Measures register access performance
6. **Burst Access Tests**: Tests rapid sequential register access

## Interactive Mode Commands

- `r <offset>` - Read from register at offset
- `w <offset> <value>` - Write value to register at offset
- `s` - Read status register
- `c <value>` - Write to control register
- `d` - Dump all readable registers
- `q` - Quit interactive mode

## Expected Results

### Normal Operation
- Control register writes should succeed (no readback verification possible)
- Status register should show current SASI bus state
- DMA address registers should retain written values
- Data register behavior depends on SASI bus state

### Common Issues
- If all reads return 0xFF: Board not present or not at expected address
- If addresses don't retain values: Possible bus timing issue
- If status always shows busy: SASI bus may be stuck

## Debugging Tips

1. Start with interactive mode to manually test each register
2. Use a logic analyzer to verify 8088 bus timing
3. Check that Pico debug output shows register accesses
4. Verify DMA board is properly initialized before running tests

## Memory Models

The program uses the small memory model (-ms) for compatibility with all DOS systems. The DMA registers are accessed using far pointers to segment 0xEF30.

## Compatibility

- 8088/8086 processors and above
- MS-DOS 2.0 or later
- Compatible with Victor 9000/Sirius 1 hardware