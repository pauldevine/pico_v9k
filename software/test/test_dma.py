import RPi.GPIO as GPIO
import time
import threading

# Define GPIO pins based on the dma.h definitions
SSO_PIN = 9
DEN_PIN = 11
ALE_PIN = 13
IO_M_PIN = 19
RD_PIN = 5
DT_R_PIN = 22
WR_PIN = 10

HOLD_PIN = 2
HLDA_PIN = 3
EXTIO_PIN = 0

READY_PIN = 27
CLOCK_15B_PIN = 4
CLOCK_5_PIN = 17



def setup_pins():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(HOLD_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(HLDA_PIN, GPIO.OUT)
    GPIO.output(HLDA_PIN, GPIO.LOW)
    GPIO.setup(EXTIO_PIN, GPIO.IN)
    GPIO.setup(IO_M_PIN, GPIO.IN)
    GPIO.setup(DT_R_PIN, GPIO.IN)
    GPIO.setup(SSO_PIN, GPIO.IN)
    GPIO.setup(ALE_PIN, GPIO.IN)
    GPIO.setup(READY_PIN, GPIO.OUT)
    GPIO.setup(CLOCK_15B_PIN, GPIO.OUT)
    GPIO.setup(CLOCK_5_PIN, GPIO.OUT)
    GPIO.setup(DEN_PIN, GPIO.IN)
    GPIO.setup(WR_PIN, GPIO.IN)
    GPIO.setup(RD_PIN, GPIO.IN)

def clock_simulation(pin, frequency):
    while True:
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(1 / (2 * frequency))
        GPIO.output(pin, GPIO.LOW)
        time.sleep(1 / (2 * frequency))

def hlda_simulation():
    while True:
        if GPIO.input(HOLD_PIN) == GPIO.LOW:
            GPIO.output(HLDA_PIN, GPIO.HIGH)
            print("HOLD requested")
        else:
            GPIO.output(HLDA_PIN, GPIO.LOW)
            #print("HOLD NOT requested")

def ready_simulation():
    GPIO.setup(READY_PIN, GPIO.OUT)
    while True:
        GPIO.output(READY_PIN, GPIO.HIGH)   
        time.sleep(0.1)
        GPIO.output(READY_PIN, GPIO.LOW)
        time.sleep(0.1)

def test_dma_write():
    #print("Testing DMA Write...")
    if GPIO.input(HOLD_PIN) == GPIO.LOW:
        print("HOLD requested")


def test_dma_read():
    print("Testing DMA Read...")

def cleanup():
    GPIO.cleanup()

if __name__ == "__main__":
    setup_pins()
    try:
        threading.Thread(target=clock_simulation, args=(CLOCK_15B_PIN, 15)).start()
        threading.Thread(target=clock_simulation, args=(CLOCK_5_PIN, 5)).start()
        threading.Thread(target=hlda_simulation).start()
        threading.Thread(target=ready_simulation).start()
        for i in range(3000000000000000):
            print(f"{i+1}")
            time.sleep(1)
            test_dma_write()
    finally:
        cleanup()    print("DT/R: ", GPIO.input(DT_R_PIN))
    print("WR", GPIO.input(RD_PIN))
    print("EXTIO", GPIO.input(EXTIO_PIN))
    print("READY", GPIO.input(READY_PIN))
   
    time.sleep(0.1)
    print("3")
    print("SSO: ", GPIO.input(SSO_PIN))
    print("DEN: ", GPIO.input(EXTIO_PIN))
    print("ALE: ", GPIO.input(ALE_PIN))
    print("IO/M: ", GPIO.input(IO_M_PIN))
    print("RD: ", GPIO.input(RD_PIN))
    print("DT/R: ", GPIO.input(DT_R_PIN))
    print("WR", GPIO.input(RD_PIN))
    print("EXTIO", GPIO.input(EXTIO_PIN))
    print("READY", GPIO.input(READY_PIN))
    print("DMA Write test completed")

def test_dma_read():
    print("Testing DMA Read...")
    
    time.sleep(0.1)
    if GPIO.input(HOLD_PIN):
        print("HOLD requested")
        GPIO.output(HLDA_PIN, GPIO.HIGH)
    else:
        print("HOLD not requested")
        GPIO.output(HLDA_PIN, GPIO.LOW)
    print("SSO: ", GPIO.input(SSO_PIN))
    print("DEN: ", GPIO.input(EXTIO_PIN))
    print("ALE: ", GPIO.input(ALE_PIN))
    print("IO/M: ", GPIO.input(IO_M_PIN))
    print("RD: ", GPIO.input(RD_PIN))
    print("DT/R: ", GPIO.input(DT_R_PIN))
    print("WR", GPIO.input(RD_PIN))
    print("EXTIO", GPIO.input(EXTIO_PIN))
    print("READY", GPIO.input(READY_PIN))
    
    time.sleep(0.1)
    print("2")
    print("SSO: ", GPIO.input(SSO_PIN))
    print("DEN: ", GPIO.input(EXTIO_PIN))
    print("ALE: ", GPIO.input(ALE_PIN))
    print("IO/M: ", GPIO.input(IO_M_PIN))
    print("RD: ", GPIO.input(RD_PIN))
    print("DT/R: ", GPIO.input(DT_R_PIN))
    print("WR", GPIO.input(RD_PIN))
    print("EXTIO", GPIO.input(EXTIO_PIN))
    print("READY", GPIO.input(READY_PIN))
   
    time.sleep(0.1)
    print("3")
    print("SSO: ", GPIO.input(SSO_PIN))
    print("DEN: ", GPIO.input(EXTIO_PIN))
    print("ALE: ", GPIO.input(ALE_PIN))
    print("IO/M: ", GPIO.input(IO_M_PIN))
    print("RD: ", GPIO.input(RD_PIN))
    print("DT/R: ", GPIO.input(DT_R_PIN))
    print("WR", GPIO.input(RD_PIN))
    print("EXTIO", GPIO.input(EXTIO_PIN))
    print("READY", GPIO.input(READY_PIN))

def cleanup():
    GPIO.cleanup()

if __name__ == "__main__":
    setup_pins()
    try:
        threading.Thread(target=clock_simulation, args=(CLOCK_15B_PIN, 15)).start()
        threading.Thread(target=clock_simulation, args=(CLOCK_5_PIN, 5)).start()
        threading.Thread(target=hlda_simulation).start()
        threading.Thread(target=ready_simulation).start()
        test_dma_write()
        test_dma_read()
    finally:
        cleanup()
