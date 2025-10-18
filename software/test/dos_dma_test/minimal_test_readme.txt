MINIMAL.EXE - DMA Address Register Test
========================================

This test focuses specifically on the REG_ADDR_H register (0xC0) timing issue
where data arrives 100ns too late for the 8088 to read it properly.

Test Sequence:
--------------
1. Initial Test Pattern
   - Writes 0xAA to REG_ADDR_H (0xEF3C0)
   - Immediately reads back
   - Expected: 0xAA, but gets register offset (0xC0) due to timing

2. Read with Delay
   - Writes 0x55 to REG_ADDR_H
   - Small delay before reading
   - Tests if delay helps with timing

3. Multiple Sequential Reads
   - Tests if first read fails but subsequent reads succeed
   - Important for understanding PIO/ISR interaction

4. Walking Bit Pattern
   - Tests all bit positions (0x01, 0x02, 0x04, etc.)
   - Helps identify any bit-specific issues

5. Address Boundary Test
   - Tests values near 20-bit address boundaries
   - Important for DMA address calculations

Running the Test:
-----------------
Copy minimal.exe to your Victor 9000 and run it.
The test will display all register writes and reads,
showing expected vs actual values.

Key Observations to Note:
-------------------------
- Does the first read always fail but subsequent reads succeed?
- Do any patterns work better than others?
- Does adding delay between write and read help?
- Are there specific bit patterns that fail?

This minimal test isolates the core timing problem without
the complexity of full DMA operations or SASI commands.