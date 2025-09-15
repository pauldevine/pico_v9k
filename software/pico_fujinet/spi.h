// SPI defines
#define SPI_PORT spi0
#define PIN_MISO 32
#define PIN_CS   33
#define PIN_SCK  34
#define PIN_MOSI 35
#define PIN_SPI_HANDSHAKE 36

// Protocol defines
#define CMD_HELLO 0x01

typedef struct
{
    uint8_t sync1;  // 0xAA bytes to make sure we're in sync
    uint8_t sync2;  // 0x55
    uint8_t device;
    uint8_t comnd;
    union {
        struct {
            uint8_t aux1;
            uint8_t aux2;
            uint8_t aux3;
            uint8_t aux4;
        };
        struct {
            uint16_t aux12;
            uint16_t aux34;
        };
        uint32_t aux;
    };
    uint8_t reserved[3]; // reserved for future use, should be 0
    uint8_t cksum;
} __attribute__((packed)) cmdFrame_t;

typedef enum 
{
    SPI_ACK = 0x06,  // Successful transaction 6=ASCII ACK
    SPI_SYNC_ERROR = 0x14, // Sync error 20=ASCII DC4 Device Control 4
    SPI_CHECKSUM_ERROR = 0x15, // Checksum error 21=ASCII NAK Negative Acknowledge
    SPI_GENERAL_ERROR = 0x04 // General transaction error, 4=ASCII End of Transmission (EOT)
} spi_transaction_result;

// compute how many bytes to add so total is a multiple of 4
// This is used to ensure DMA transfers are aligned correctly
// Returns 0 if length is already a multiple of 4
// Returns 1, 2, or 3 to indicate how many bytes to add
static inline size_t pad_multiple_4(size_t len) {
    // If length is already a multiple of 4, no padding needed
    if (len % 4 == 0) return 0;

    // Calculate padding needed to make length a multiple of 4
    // (4 - (len % 4)) gives the number of bytes to add
    // & 3 ensures we only add 0, 1, 2, or 3 bytes
    return (4 - (len % 4)) & 3;
}