#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#include "spi.h"
#include "fujiCmd.h"

#define SPI_PHASE_TIMEOUT_US 10000000
#define VICTOR_SECTOR_COUNT_SINGLE 1

uint8_t calculate_checksum_buf(const uint8_t *buf, size_t len) {
    unsigned int chk = 0;
    for (size_t i = 0; i < len; i++) {
        chk = ((chk + buf[i]) >> 8) + ((chk + buf[i]) & 0xff);
    }
    return (uint8_t)chk;
}

// Simple checksum function (matches your ESP32 code)
uint8_t calculate_checksum_cmd(cmdFrame_t *cmd)
{
    uint8_t cksum = 0;
    uint8_t *p = (uint8_t*)cmd;
    
    // Calculate checksum over frame (excluding cksum field)
    for (int i = 0; i < offsetof(cmdFrame_t, cksum); i++) {
        cksum ^= p[i];
    }
    
    return cksum;
}

// returns a newly-malloc’d buffer, zero-padded to 4-byte align
uint8_t *pad_to_4(const uint8_t *in, size_t len, size_t *out_len) {
    size_t pad = pad_multiple_4(len);
    size_t n   = len + pad;
    uint8_t *buf = malloc(n);
    if(!buf) return NULL;
    memcpy(buf, in, len);
    if(pad) memset(buf + len, 0, pad);
    if(out_len) *out_len = n;
    return buf;
}

static bool spi_start_transaction_phase(const char *phase, uint32_t timeout_us)
{
    printf("Pico SPI [%s]: CS LOW\n", phase);
    gpio_put(PIN_CS, 0);

    uint32_t waited = 0;
    while (gpio_get(PIN_SPI_HANDSHAKE)) {
        sleep_us(10);
        waited += 10;
        if (waited >= timeout_us) {
            printf("Pico SPI [%s]: timeout waiting for handshake low\n", phase);
            gpio_put(PIN_CS, 1);
            return false;
        }
    }

    printf("Pico SPI [%s]: handshake low after %u us\n", phase, waited);
    return true;
}

static void spi_end_transaction_phase(const char *phase)
{
    gpio_put(PIN_CS, 1);
    printf("Pico SPI [%s]: CS HIGH (handshake=%d)\n", phase, gpio_get(PIN_SPI_HANDSHAKE));
}

static bool receive_status_byte(const char *label, uint8_t *status_out) {
    if (!status_out) return false;

    if (!spi_start_transaction_phase(label, SPI_PHASE_TIMEOUT_US)) {
        return false;
    }

    int read = spi_read_blocking(SPI_PORT, 0x00, status_out, 1);
    spi_end_transaction_phase(label);

    if (read != 1) {
        printf("%s: failed to read status byte (read=%d)\n", label, read);
        return false;
    }

    printf("%s: 0x%02X\n", label, *status_out);
    return true;
}

static bool finish_simple_command(const char *label) {
    uint8_t status = 0;
    if (!receive_status_byte(label, &status)) {
        return false;
    }

    if (status == 'A') {
        char completion_label[64];
        snprintf(completion_label, sizeof(completion_label), "%s completion", label);
        if (!receive_status_byte(completion_label, &status)) {
            return false;
        }
    }

    if (status != 'C') {
        printf("%s: unexpected status 0x%02X\n", label, status);
        return false;
    }

    printf("%s: COMPLETE\n", label);
    return true;
}

// Send a command frame to ESP32
spi_transaction_result send_command_frame(cmdFrame_t cmd) {
    
    cmd.sync1 = 0xAA;  // Sync byte 1
    cmd.sync2 = 0x55;  // Sync byte 2
    cmd.cksum = calculate_checksum_cmd(&cmd);  // Calculate checksum
    printf("Command frame: sync1=0x%02X, sync2=0x%02X, device=0x%02X, command=0x%02X, aux1=0x%02X, aux2=0x%02X, aux3=0x%02X, aux4=0x%02X, checksum=0x%02X\n",
           cmd.sync1, cmd.sync2, cmd.device, cmd.comnd, cmd.aux1, cmd.aux2, cmd.aux3, cmd.aux4, cmd.cksum);
    uint8_t frame[sizeof(cmdFrame_t)] = {0};
    memcpy(frame, &cmd, sizeof(cmdFrame_t));

    printf("Sending command frame: ");
    for (int i = 0; i < sizeof(frame); i++) {
        printf("%02X ", frame[i]);
    }
    printf("\n");
    
    if (!spi_start_transaction_phase("CMD_TX", SPI_PHASE_TIMEOUT_US)) {
        return SPI_GENERAL_ERROR;
    }

    int bytes_written = spi_write_blocking(SPI_PORT, frame, sizeof(frame));

    spi_end_transaction_phase("CMD_TX");
    printf("Bytes written: %d\n", bytes_written);

     // now immediately read the ACK
    if (!spi_start_transaction_phase("CMD_ACK_RX", SPI_PHASE_TIMEOUT_US)) {
        return SPI_GENERAL_ERROR;
    }
    uint8_t ack_byte = 0;
    spi_read_blocking (SPI_PORT, 0xFF, &ack_byte, 1);

    //end transaction
    spi_end_transaction_phase("CMD_ACK_RX");
    printf("ACK received: %02X\n", ack_byte);
    spi_transaction_result ack = (spi_transaction_result)ack_byte;
    if (ack != SPI_ACK) {
        printf("Error: Received ACK 0x%02X instead of 0x%02X\n", ack, SPI_ACK);
        return ack;
    } else {
        printf("Command frame sent successfully, ACK received: %02X\n", ack);
    }

    return ack;
}

// Send data with checksum (binary-safe)
spi_transaction_result send_data_frame(const uint8_t *data, size_t len) {
    if (!data && len > 0) {
        printf("send_data_frame: null data pointer\n");
        return SPI_GENERAL_ERROR;
    }

    uint8_t checksum = calculate_checksum_buf(data, len);

    size_t combined_len = len + 1;
    uint8_t *combined = malloc(combined_len);
    if (!combined) {
        printf("send_data_frame: failed to allocate combined buffer\n");
        return SPI_GENERAL_ERROR;
    }

    if (len > 0) memcpy(combined, data, len);
    combined[len] = checksum;

    size_t padded_len;
    uint8_t *padded_frame = pad_to_4(combined, combined_len, &padded_len);
    free(combined);
    if (!padded_frame) {
        printf("send_data_frame: failed to allocate padded frame\n");
        return SPI_GENERAL_ERROR;
    }

    printf("Sending data frame (%zu bytes, padded to %zu), checksum: 0x%02X\n",
           len, padded_len, checksum);

    if (!spi_start_transaction_phase("DATA_TX", SPI_PHASE_TIMEOUT_US)) {
        free(padded_frame);
        return SPI_GENERAL_ERROR;
    }
    int bytes_written = spi_write_blocking(SPI_PORT, padded_frame, padded_len);
    spi_end_transaction_phase("DATA_TX");

    free(padded_frame);

    if (!spi_start_transaction_phase("DATA_ACK_RX", SPI_PHASE_TIMEOUT_US)) {
        return SPI_GENERAL_ERROR;
    }
    uint8_t ack = 0;
    spi_read_blocking(SPI_PORT, 0x00, &ack, 1);
    spi_end_transaction_phase("DATA_ACK_RX");

    printf("Data sent: %d bytes (padded), ACK received: 0x%02X\n", bytes_written, ack);

    spi_transaction_result ack_result = (spi_transaction_result)ack;
    if (ack_result != SPI_ACK) {
        printf("Error: Received ACK 0x%02X instead of 0x%02X\n", ack_result, SPI_ACK);
    }

    return ack_result;
}


// Read data frame with checksum verification
bool read_data_frame(uint8_t *buffer, size_t expected_len) {
    if (!spi_start_transaction_phase("DATA_RX", SPI_PHASE_TIMEOUT_US)) {
        return false;
    }

    // Read data
    int data_read = spi_read_blocking(SPI_PORT, 0x00, buffer, expected_len);
    
    // Read checksum
    uint8_t received_checksum;
    int chk_read = spi_read_blocking(SPI_PORT, 0x00, &received_checksum, 1);
    
    spi_end_transaction_phase("DATA_RX");
    
    if (data_read != expected_len || chk_read != 1) {
        printf("Read failed: data=%d/%d, checksum=%d/1\n", data_read, expected_len, chk_read);
        return false;
    }
    
    // Verify checksum
    uint8_t calculated_checksum = calculate_checksum_buf(buffer, expected_len);
    if (received_checksum != calculated_checksum) {
        printf("Checksum mismatch: received=0x%02X, calculated=0x%02X\n", 
               received_checksum, calculated_checksum);
        return false;
    }
    
    printf("Received data (%zu bytes): ", expected_len);
    for (size_t i = 0; i < expected_len; i++) {
        printf("%02X ", buffer[i]);
    }
    printf(" checksum: 0x%02X\n", received_checksum);
    
    return true;
}

static bool send_sector_payload(uint8_t device, uint8_t command, const uint8_t *buffer, size_t len) {
    cmdFrame_t payload_cmd = {0};
    payload_cmd.device = device;
    payload_cmd.comnd = command;
    payload_cmd.aux = (uint32_t)len;

    printf("Queueing sector payload: device=0x%02X command=0x%02X length=%zu\n",
           device, command, len);

    spi_transaction_result ack = send_command_frame(payload_cmd);
    if (ack != SPI_ACK) {
        printf("Sector payload command NAK: 0x%02X\n", ack);
        return false;
    }

    ack = send_data_frame(buffer, len);
    if (ack != SPI_ACK) {
        printf("Sector payload data NAK: 0x%02X\n", ack);
        return false;
    }

    return true;
}

// Simple hello world transaction
void hello_world_transaction() {
    printf("\n=== Starting Hello World Transaction ===\n");
    const char *hello_msg = "Hello World!";
    printf("Sending message: '%s'\n", hello_msg);

    cmdFrame_t cmd={0};
    cmd.device = 0x01;
    cmd.comnd = 0x02;  //status command
    cmd.aux = (uint32_t) strlen(hello_msg);  // Length of data to send

    // Step 1: Send command frame
    if (send_command_frame(cmd) != SPI_ACK) {  // Device 0x31, Hello command, 12 byte data length
        printf("Failed to send command frame\n");
        return;
    }
    
    // Step 3: Send data frame

    if (send_data_frame((const uint8_t *)hello_msg, strlen(hello_msg)) != SPI_ACK) {
        printf("Failed to send data frame\n");
        return;
    }
}

// Test bidirectional communication
void echo_test() {
    printf("\n=== Starting Echo Test ===\n");
    const char *test_data = "ANOTHER TEST!";
    printf("Sending message: '%s'\n", test_data);

    cmdFrame_t cmd={0};
    cmd.device = 0x31;  // Device ID for ESP32
    cmd.comnd = 0x02;   // Echo command
    cmd.aux = (uint32_t) strlen(test_data);  // Length of data to send


    // Send command for echo test
    if (send_command_frame(cmd) != SPI_ACK) {  // Echo command, expect 5 bytes back
        printf("Failed to send echo command\n");
        return;
    }
    
    // Send test data

    if (send_data_frame((const uint8_t *)test_data, strlen(test_data)) != SPI_ACK) {
        printf("Failed to send test data\n");
        return;
    }

}

void spi_bus_init() {
    // Initialize SPI
    spi_init(SPI_PORT, 1000000);  // 1MHz clock speed
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Initialize GPIO pins
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Handshake pin
    gpio_init(PIN_SPI_HANDSHAKE);
    gpio_set_dir(PIN_SPI_HANDSHAKE, GPIO_IN);
    gpio_pull_up(PIN_SPI_HANDSHAKE);

    // CS is manual control
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);  // CS idle high
}

bool fujinet_read_sector(uint8_t device, uint32_t lba, uint8_t *buffer, size_t len) {
    if (len != 512) {
        printf("fujinet_read_sector: invalid length %u (expected 512)\n", (unsigned)len);
        return false;
    }

    victor_disk_rw_payload_t header = {
        .lba = lba,
        .sector_count = VICTOR_SECTOR_COUNT_SINGLE,
        .flags = 0,
    };

    cmdFrame_t cmd = {0};
    cmd.device = device;
    cmd.comnd  = CMD_DISK_READ;
    cmd.aux    = 0;

    spi_transaction_result ack = send_command_frame(cmd);
    if (ack != SPI_ACK) {
        printf("FujiNet read command NAK: 0x%02X\n", ack);
        return false;
    }

    ack = send_data_frame((const uint8_t *)&header, sizeof(header));
    if (ack != SPI_ACK) {
        printf("FujiNet read payload NAK: 0x%02X\n", ack);
        return false;
    }

    uint8_t status = 0;
    if (!receive_status_byte("FujiNet command status", &status)) {
        return false;
    }
    if (status == 'A') {
        if (!receive_status_byte("FujiNet command completion", &status)) {
            return false;
        }
    }
    if (status != 'C') {
        printf("FujiNet reported error status 0x%02X\n", status);
        return false;
    }

    // Read back 512 bytes + checksum
    if (!read_data_frame(buffer, len)) {
        printf("FujiNet read data failed for LBA %lu\n", (unsigned long)lba);
        return false;
    }

    return true;
}
bool fujinet_write_sector(uint8_t device, uint32_t lba, const uint8_t *buffer, size_t len) {
    if (len != 512) {
        printf("fujinet_write_sector: invalid length %u (expected 512)\n", (unsigned)len);
        return false;
    }

    victor_disk_rw_payload_t header = {
        .lba = lba,
        .sector_count = VICTOR_SECTOR_COUNT_SINGLE,
        .flags = 0,
    };

    cmdFrame_t cmd = {0};
    cmd.device = device;
    cmd.comnd  = CMD_DISK_WRITE;
    cmd.aux    = 0;

    spi_transaction_result ack = send_command_frame(cmd);
    if (ack != SPI_ACK) {
        printf("FujiNet write command NAK: 0x%02X\n", ack);
        return false;
    }

    ack = send_data_frame((const uint8_t *)&header, sizeof(header));
    if (ack != SPI_ACK) {
        printf("FujiNet write header NAK for LBA %lu, ack=0x%02X\n", (unsigned long)lba, ack);
        return false;
    }

    if (!send_sector_payload(device, CMD_DISK_WRITE, buffer, len)) {
        printf("FujiNet write sector payload failed for LBA %lu\n", (unsigned long)lba);
        return false;
    }

    uint8_t status = 0;
    if (!receive_status_byte("FujiNet write status", &status)) {
        return false;
    }
    if (status == 'A') {
        if (!receive_status_byte("FujiNet write completion", &status)) {
            return false;
        }
    }
    if (status != 'C') {
        printf("FujiNet write status 0x%02X for LBA %lu\n", status, (unsigned long)lba);
        return false;
    }
    return true;
}

bool fujinet_config_boot(bool enable) {
    cmdFrame_t cmd = {0};
    cmd.device = DEVICE_FUJINET_CONTROL;
    cmd.comnd = FUJICMD_CONFIG_BOOT;
    cmd.aux1 = enable ? 1 : 0;

    if (send_command_frame(cmd) != SPI_ACK) {
        return false;
    }

    return finish_simple_command("FujiNet config boot");
}

bool fujinet_mount_host(uint8_t host_slot, uint8_t access_mode) {
    cmdFrame_t cmd = {0};
    cmd.device = DEVICE_FUJINET_CONTROL;
    cmd.comnd = FUJICMD_MOUNT_HOST;
    cmd.aux1 = host_slot;
    cmd.aux2 = access_mode;

    if (send_command_frame(cmd) != SPI_ACK) {
        return false;
    }

    return finish_simple_command("FujiNet mount host");
}

bool fujinet_mount_disk_slot(uint8_t slot, uint8_t access_mode) {
    cmdFrame_t cmd = {0};
    cmd.device = DEVICE_FUJINET_CONTROL;
    cmd.comnd = FUJICMD_MOUNT_IMAGE;
    cmd.aux1 = slot;
    cmd.aux2 = access_mode;

    if (send_command_frame(cmd) != SPI_ACK) {
        return false;
    }

    return finish_simple_command("FujiNet mount image");
}
