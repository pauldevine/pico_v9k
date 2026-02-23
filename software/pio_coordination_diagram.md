# PIO State Machine Coordination Diagrams

This document shows the T-state timing and signaling between the three PIO state machines for different bus cycles.

## Register Read Cycle (8088 reading from 0xEF300, single-pass)

```mermaid
sequenceDiagram
    participant BR as board_registers<br/>SM
    participant ARM as ARM Core
    participant BUS as 8088 Bus

    Note over BUS: T0: CLK5 goes low
    BUS->>BR: ALE goes high
    Note over BR: T1: Address latched
    BR->>BR: Detect 0xEF300 match
    BR->>BR: Store full 20-bit address

    Note over BR: T2: Assert XACK (open-drain), hold READY low
    BR->>BR: Check RD pin (low=read)

    Note over BR: T3_READ: Single FIFO request/response
    BR->>ARM: FIFO RX: [type=0][address]
    BR->>BR: pull block (wait for ARM response)

    Note over ARM: register_read_irq_isr
    ARM->>ARM: Decode address + compute data + apply read side effects
    ARM->>BR: FIFO TX: [0xFF pindirs][8-bit data]

    Note over BR: Drive response and release wait state
    BR->>BUS: out pins 8 (BD0-BD7 = data)
    BR->>BUS: out pindirs 8 (BD0-BD7 outputs, XACK released)

    Note over BUS: T3: 8088 samples data
    BUS->>BUS: RD goes high
    BR->>BR: wait RD high (end of cycle)

    Note over BR: T4/T0: Cleanup
    BR->>BUS: Release EXTIO (side-set back to input)
    BR->>BUS: Tri-state BD0-BD7 on next cycle setup

    Note over BR,BUS: Cycle complete, back to T0
```

## DMA Read Cycle (Pico reading from 8088 memory)

```mermaid
sequenceDiagram
    participant ARM as ARM Core
    participant DR as dma_read_write<br/>SM
    participant BH as bus_output_helper<br/>SM
    participant BUS as 8088 Bus

    Note over ARM: Prepare DMA read
    ARM->>BH: FIFO TX: 0xFFF00 (pindirs for read)
    ARM->>DR: FIFO TX: [addr<<1 | 0] (R/W=0)
    ARM->>DR: FIFO TX: [0xFFF][0x00] (pindirs+data)

    Note over DR: T0: Request bus
    DR->>DR: pull block (get addr+R/W)
    DR->>DR: out x,1 (x=0 for read)
    DR->>BUS: Assert HOLD/, set IO/M input
    DR->>DR: wait HLDA high

    Note over DR: T1: Address phase
    DR->>BH: IRQ 1 (Output address)
    DR->>BUS: Set ALE high, DEN high

    Note over BH: Output address
    BH->>BH: wait irq 1
    BH->>BH: pull block (get addr)
    BH->>BH: jmp pin (HLDA=1, DMA cycle)
    BH->>BH: mov isr, osr (save address)
    BH->>BUS: out pins 20 (A0-A19 = address)
    BH->>BUS: mov pindirs ~null (outputs)
    BH->>BH: pull block (get pindirs+data)

    DR->>DR: wait CLK5 high, then low

    Note over DR: T2: Switch to data phase
    DR->>BUS: ALE low, DEN low
    DR->>DR: wait CLK15B high
    DR->>BH: IRQ 2 (Address→Data transition)
    DR->>DR: jmp !x T2_READ (x=0, take jump)
    DR->>BUS: RD/ low, DT/R low

    Note over BH: Configure for read
    BH->>BH: wait irq 2
    BH->>BH: out x, 1 (x=0 for read)
    BH->>BUS: out pins 20 (A8-A19 on upper bits)
    BH->>BUS: mov pindirs, y (0xFFF00: A8-A19 out, BD0-BD7 in)
    BH->>BH: wait irq 3 (for read capture)

    Note over DR: T3: Data capture
    DR->>DR: wait READY high
    DR->>DR: wait CLK5 high, then low
    DR->>BUS: Release HOLD/, keep IO/M output
    DR->>DR: jmp x-- T4 (x=0, fall through)
    DR->>BH: IRQ 3 (Capture data now)

    Note over BH: Read data
    BH->>BUS: in pins 8 (read BD0-BD7)
    BH->>ARM: FIFO RX: push (send data to ARM)

    Note over DR: T4: End cycle
    DR->>DR: wait CLK5 low, CLK15B high
    DR->>BUS: WR/ high, RD/ high
    DR->>DR: wait CLK5 high
    DR->>BUS: DEN high
    DR->>DR: wait HLDA low
    DR->>BUS: Control pins to input, release HOLD/
    DR->>BH: IRQ 4 (Tri-state bus)

    Note over BH: Release bus
    BH->>BH: wait irq 4
    BH->>BUS: mov pindirs null (tri-state all)

    Note over DR,BUS: Cycle complete
```

## DMA Write Cycle (Pico writing to 8088 memory)

```mermaid
sequenceDiagram
    participant ARM as ARM Core
    participant DR as dma_read_write<br/>SM
    participant BH as bus_output_helper<br/>SM
    participant BUS as 8088 Bus

    Note over ARM: Prepare DMA write
    ARM->>DR: FIFO TX: [addr<<1 | 1] (R/W=1)
    ARM->>DR: FIFO TX: [A19-A8][data][1]

    Note over DR: T0: Request bus
    DR->>DR: pull block (get addr+R/W)
    DR->>DR: out x,1 (x=1 for write)
    DR->>BUS: Assert HOLD/, set IO/M input
    DR->>DR: wait HLDA high

    Note over DR: T1: Address phase
    DR->>BH: IRQ 1 (Output address)
    DR->>BUS: Set ALE high, DEN high

    Note over BH: Output address
    BH->>BH: wait irq 1
    BH->>BH: pull block (get addr)
    BH->>BH: jmp pin (HLDA=1, DMA cycle)
    BH->>BH: mov isr, osr (save address)
    BH->>BUS: out pins 20 (A0-A19 = address)
    BH->>BUS: mov pindirs ~null (all outputs)
    BH->>BH: pull block (get data packet)

    DR->>DR: wait CLK5 high, then low

    Note over DR: T2: Switch to data phase
    DR->>BUS: ALE low, DEN low
    DR->>DR: wait CLK15B high
    DR->>BH: IRQ 2 (Address→Data transition)
    DR->>DR: jmp !x T2_WRITE (x=1, fall through)
    DR->>BUS: WR/ low

    Note over BH: Output data
    BH->>BH: wait irq 2
    BH->>BH: out x, 1 (x=1 for write)
    BH->>BUS: out pins 20 (BD0-BD7=data, A8-A19)
    BH->>BH: jmp !x DMA_T2_READ (x=1, fall through)
    BH->>BUS: mov pindirs ~null (all outputs)
    BH->>BH: jmp T4_END (skip read logic)

    Note over DR: T3: Write data
    DR->>DR: wait READY high
    DR->>DR: wait CLK5 high, then low
    DR->>BUS: Release HOLD/, keep IO/M output
    DR->>DR: jmp x-- T4 (x=1, decrement & jump)

    Note over DR: T4: End cycle
    DR->>DR: wait CLK5 low, CLK15B high
    DR->>BUS: WR/ high, RD/ high
    DR->>DR: wait CLK5 high
    DR->>BUS: DEN high
    DR->>DR: wait HLDA low
    DR->>BUS: Control pins to input, release HOLD/
    DR->>BH: IRQ 4 (Tri-state bus)

    Note over BH: Release bus
    BH->>BH: wait irq 4
    BH->>BUS: mov pindirs null (tri-state all)

    Note over DR,BUS: Cycle complete
```

## IRQ Protocol Summary

| IRQ | Source | Destination | Phase | Purpose |
|-----|--------|-------------|-------|---------|
| **1** | dma_read_write | bus_output_helper | T1 | Output DMA address phase |
| **2** | dma_read_write | bus_output_helper | T2 | Switch from address to data phase |
| **3** | dma_read_write | bus_output_helper | T3 | Capture read data from BD0-BD7 |
| **4** | dma_read_write | bus_output_helper | T4 | Cycle complete, tri-state bus |

`board_registers` no longer uses inter-SM IRQ handshakes for register reads; it performs a direct FIFO round-trip with ARM Core 0 while XACK holds READY low.

## FIFO Protocol Summary

### board_registers → ARM (RX FIFO)
- **Read**: `[type=0][address 20-bit]` (address in bits 30:11)
- **Write**: `[type=1][data 8-bit][address 20-bit]` (data in bits 30:23, address in bits 22:3)

### ARM → board_registers (TX FIFO)
- **Register read response**: `[pindirs 0xFF][data 8-bit]` (16-bit pull consumed by `board_registers`)

### ARM → bus_output_helper (TX FIFO)
- **DMA cycle**: First pulled in preamble (pindirs value 0xFFF00), then address pulled at runtime

### ARM → dma_read_write (TX FIFO)
- **First word**: [padding] + 20-bit address + 1-bit R/W flag
- **Second word**: [A19-A8 or pindirs] + [data or 0x00] + [R/W bit]

### bus_output_helper → ARM (RX FIFO)
- **DMA read data**: 8-bit value read from BD0-BD7

## Key Timing Points

1. **Register reads are now deterministic**: XACK (open-drain) holds READY low until Core 0 returns a byte via FIFO.
2. **Single-pass register semantics**: one read payload does both data return and read side effects; no commit payload.
3. **DMA cycles remain tightly timed**: ARM must preload FIFOs before initiating DMA.
4. **bus_output_helper remains DMA arbiter**: it coordinates DMA bus direction/tri-state transitions via IRQs 1/2/3/4.
