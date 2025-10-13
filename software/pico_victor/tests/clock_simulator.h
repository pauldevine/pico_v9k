#ifndef CLOCK_SIMULATOR_H
#define CLOCK_SIMULATOR_H

// Initialize the clock simulator (starts generating CLK5 and CLK15B)
void clock_simulator_init(void);

// Stop the clock simulator
void clock_simulator_stop(void);

// Test the clock simulator output
void clock_simulator_test(void);

#endif // CLOCK_SIMULATOR_H