#include "nrf24_radio.h"
#include "config.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#ifndef DEBUG_PRINT
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

void NRF24::writeReg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {(uint8_t)(W_REG | reg), value};
    csn_low();
    spi_write_blocking(spi, buf, 2);
    csn_high();
}

uint8_t NRF24::readReg(uint8_t reg) {
    uint8_t buf = (R_REG | reg);
    uint8_t response;
    csn_low();
    spi_write_blocking(spi, &buf, 1);
    spi_read_blocking(spi, 0x00, &response, 1);
    csn_high();
    return response;
}

void NRF24::writeCmd(uint8_t cmd) {
    csn_low();
    spi_write_blocking(spi, &cmd, 1);
    csn_high();
}

bool NRF24::isSending() {
    if (!_is_sending) return false;
    uint8_t status = readReg(STATUS);
    if (status & 0x30) {
        _is_sending = false;
        return false;
    }
    return true;
}

bool NRF24::isReady() {
    if (isSending()) return false;
    return true;
}

NRF24::NRF24(spi_inst_t* spi_port, uint ce, uint csn, uint sck, uint mosi, uint miso)
: spi(spi_port), ce_pin(ce), csn_pin(csn) {
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(miso, GPIO_FUNC_SPI);

    gpio_init(ce_pin);
    gpio_set_dir(ce_pin, GPIO_OUT);
    ce_low();

    gpio_init(csn_pin);
    gpio_set_dir(csn_pin, GPIO_OUT);
    csn_high();
}

bool NRF24::checkConnection() {
    spi_init(spi, 2000 * 1000);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    sleep_ms(10);

    uint8_t test_channel = 88;
    writeReg(RF_CH, test_channel);
    sleep_ms(2);
    uint8_t response = readReg(RF_CH);

    if (response == test_channel) return true;

    DEBUG_PRINT("[NRF24 ERROR] SPI Verification Failed! Expected: %d, Got: %d\n", test_channel, response);
    return false;
}

void NRF24::init() {
    spi_init(spi, 2000 * 1000);
    spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    sleep_ms(100);

    // Basic config: enable CRC, power up, set as PTX initially
    writeReg(CONFIG, 0x0A);
    sleep_ms(5);

    // Flush FIFOs
    writeCmd(FLUSH_RX);
    writeCmd(FLUSH_TX);

    // Enable Auto‑ACK on all data pipes (0‑5) for reliable delivery
    writeReg(EN_AA, 0x3F);
    // Enable RX addresses on pipes 0 and 1 (used for responses)
    writeReg(EN_RXADDR, 0x03);
    // Auto‑retransmit: 500 µs delay, 15 retries (max robustness)
    writeReg(SETUP_RETR, 0x2F);
    // RF channel (chosen to avoid Wi‑Fi interference)
    writeReg(RF_CH, 76);
    // RF_SETUP: 1 Mbps data rate, 0 dBm output power, LNA gain off
    writeReg(RF_SETUP, 0x06);
    // Payload size for pipe 0 (response packets)
    writeReg(RX_PW_P0, 32);
    // Payload size for pipe 1 (future use)
    writeReg(RX_PW_P1, 32);

    // Disable dynamic features to enforce strict, static payload handling
    writeReg(FEATURE, 0x00);
}

void NRF24::setAddresses(const uint8_t* tx_addr, const uint8_t* rx_addr) {
    csn_low();
    uint8_t cmd_tx = W_REG | TX_ADDR;
    spi_write_blocking(spi, &cmd_tx, 1);
    spi_write_blocking(spi, tx_addr, 5);
    csn_high();

    csn_low();
    uint8_t cmd_rx0 = W_REG | RX_ADDR_P0;
    spi_write_blocking(spi, &cmd_rx0, 1);
    spi_write_blocking(spi, tx_addr, 5);
    csn_high();

    csn_low();
    uint8_t cmd_rx1 = W_REG | 0x0B;
    spi_write_blocking(spi, &cmd_rx1, 1);
    spi_write_blocking(spi, rx_addr, 5);
    csn_high();
}

void NRF24::startListening() {
    writeReg(CONFIG, readReg(CONFIG) | 0x01);
    writeReg(STATUS, 0x70); // Clear interrupt flags
    ce_high();
    sleep_us(130);
}

void NRF24::stopListening() {
    ce_low();
    writeReg(CONFIG, readReg(CONFIG) & ~0x01);
    writeCmd(FLUSH_TX);
}

// --- NEW: Hardware Soft Reset ---
void NRF24::restart() {
    ce_low();

    // 1. Power down the chip (Clear PWR_UP bit 1)
    uint8_t config = readReg(CONFIG);
    writeReg(CONFIG, config & ~0x02);
    sleep_ms(10); // Allow internal state machine to fully reset

    // 2. Power back up
    writeReg(CONFIG, config | 0x02);
    sleep_ms(5); // Oscillator start-up time

    // 3. Flush any stuck data and clear all error flags
    writeCmd(FLUSH_RX);
    writeCmd(FLUSH_TX);
    writeReg(STATUS, 0x70);

    // 4. Reset our C++ state tracker
    _is_sending = false;

    // 5. Resume normal operations
    startListening();
}

bool NRF24::dataAvailable() {
    uint8_t status = readReg(STATUS);
    bool has_data = ((status & 0x0E) != 0x0E);
    if (status & 0x40) {
        writeReg(STATUS, 0x40);
    }
    return has_data;
}

// ==========================================================
// DRONE FUNCTIONS (Drone -> Ground)
// ==========================================================

bool NRF24::sendTelemetry(TelemetryPacket* data) {
    if (!isReady()) return false;

    data->header1 = TELEMETRY_HEADER_1;
    data->header2 = TELEMETRY_HEADER_2;

    uint8_t calc_cs = 0;
    uint8_t* ptr = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) calc_cs ^= ptr[i];
    data->checksum = calc_cs;

    stopListening();
    _is_sending = true;

    // BUG FIX: Temporarily drop retries to 0 for Telemetry so it doesn't block the PIDs
    uint8_t retr_bak = readReg(SETUP_RETR);
    writeReg(SETUP_RETR, 0x00);

    csn_low();
    uint8_t cmd = TX_PAYLOAD; // Use STANDARD payload instead of toxic NOACK command
    spi_write_blocking(spi, &cmd, 1);
    spi_write_blocking(spi, (uint8_t*)data, 32);
    csn_high();

    ce_high();

    uint32_t start_time = time_us_32();
    uint8_t status = 0;
    while (true) {
        status = readReg(STATUS);
        if (status & 0x30) break;
        if (time_us_32() - start_time > 5000) break; // 5ms safety timeout
    }

    ce_low();

    writeReg(STATUS, 0x30); // Clear TX_DS and MAX_RT flags
    if (status & 0x10) writeCmd(FLUSH_TX);

    // BUG FIX: Restore normal retries before listening for PID packets
    writeReg(SETUP_RETR, retr_bak);

    startListening();

    _is_sending = false;
    return (status & 0x20) != 0;
}

bool NRF24::readCommand(RadioCommandPacket* data) {
    uint8_t buffer[32] = {0};

    csn_low();
    uint8_t cmd = RX_PAYLOAD;
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, buffer, 32);
    csn_high();

    memcpy(data, buffer, sizeof(RadioCommandPacket));

    // --- AUTO-RECOVERY FIX ---
    if (data->header1 != CMD_HEADER_1 || data->header2 != CMD_HEADER_2) {
        DEBUG_PRINT("[NRF24 ERR] readCommand: bad header 0x%02X 0x%02X (expected 0x%02X 0x%02X)\n",
                    data->header1, data->header2, CMD_HEADER_1, CMD_HEADER_2);
        return false;
    }

    uint8_t calc_cs = 0;
    uint8_t* ptr = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(RadioCommandPacket) - 1; i++) calc_cs ^= ptr[i];

    if (calc_cs != data->checksum) {
        DEBUG_PRINT("[NRF24 ERROR] Command Checksum Mismatch!\n");
        return false;
    }

    return true;
}

bool NRF24::sendResponse(RadioResponsePacket* data) {
    if (!isReady()) return false;

    data->header1 = RESP_HEADER_1;
    data->header2 = RESP_HEADER_2;

    uint8_t calc_cs = 0;
    uint8_t* ptr = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(RadioResponsePacket) - 1; i++) calc_cs ^= ptr[i];
    data->checksum = calc_cs;

    stopListening();
    _is_sending = true;

    csn_low();
    uint8_t cmd = TX_PAYLOAD;
    spi_write_blocking(spi, &cmd, 1);
    spi_write_blocking(spi, (uint8_t*)data, 32);
    csn_high();

    ce_high();

    uint32_t start_time = time_us_32();
    uint8_t status = 0;
    while (true) {
        status = readReg(STATUS);
        if (status & 0x30) break;
        if (time_us_32() - start_time > 25000) break;
    }

    ce_low();

    if (status & 0x10) {
        DEBUG_PRINT("[NRF24 ERR] sendResponse: MAX_RT hit, GS did not ACK\n");
        writeCmd(FLUSH_TX);
    }

    startListening();

    _is_sending = false;
    bool ok = (status & 0x20) != 0;
    if (!ok) {
        DEBUG_PRINT("[NRF24 ERR] sendResponse: TX_DS not set (status=0x%02X)\n", status);
    }
    return ok;
}

// ==========================================================
// GROUND STATION FUNCTIONS (Ground -> Drone)
// ==========================================================

bool NRF24::sendCommand(RadioCommandPacket* data) {
    if (!isReady()) {
        DEBUG_PRINT("[NRF24 ERROR] Radio busy, skipping command packet\n");
        return false;
    }

    data->header1 = CMD_HEADER_1;
    data->header2 = CMD_HEADER_2;

    uint8_t calc_cs = 0;
    uint8_t* ptr = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(RadioCommandPacket) - 1; i++) calc_cs ^= ptr[i];
    data->checksum = calc_cs;

    stopListening();
    _is_sending = true;

    csn_low();
    uint8_t cmd = TX_PAYLOAD;
    spi_write_blocking(spi, &cmd, 1);
    spi_write_blocking(spi, (uint8_t*)data, 32);
    csn_high();

    ce_high();

    uint32_t start_time = time_us_32();
    uint8_t status = 0;
    while(true) {
        status = readReg(STATUS);
        if (status & 0x30) break;

        if (time_us_32() - start_time > 25000) {
            DEBUG_PRINT("[NRF24 ERROR] TX Timeout!\n");
            break;
        }
    }

    ce_low();

    writeReg(STATUS, 0x30);

    if (status & 0x10) {
        DEBUG_PRINT("[NRF24 ERROR] Command TX MAX_RT (Max Retries) hit! Drone offline.\n");
        writeCmd(FLUSH_TX);
    }

    startListening();
    _is_sending = false;

    return (status & 0x20) != 0;
}

bool NRF24::readResponse(RadioResponsePacket* data) {
    uint8_t buffer[32] = {0};

    csn_low();
    uint8_t cmd = RX_PAYLOAD;
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, buffer, 32);
    csn_high();

    memcpy(data, buffer, sizeof(RadioResponsePacket));

    if (data->header1 != RESP_HEADER_1 || data->header2 != RESP_HEADER_2) {
        DEBUG_PRINT("[NRF24 ERR] readResponse: bad header 0x%02X 0x%02X (expected 0x%02X 0x%02X)\n",
                    data->header1, data->header2, RESP_HEADER_1, RESP_HEADER_2);
        return false;
    }

    uint8_t calc_cs = 0;
    uint8_t* ptr = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(RadioResponsePacket) - 1; i++) calc_cs ^= ptr[i];

    if (calc_cs != data->checksum) {
        DEBUG_PRINT("[NRF24 ERR] readResponse: checksum mismatch (calc=0x%02X, got=0x%02X)\n", calc_cs, data->checksum);
        return false;
    }

    return true;
}

bool NRF24::readTelemetry(TelemetryPacket* data) {
    uint8_t buffer[32] = {0};

    csn_low();
    uint8_t cmd = RX_PAYLOAD;
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, 0x00, buffer, 32);
    csn_high();

    memcpy(data, buffer, sizeof(TelemetryPacket));

    // --- AUTO-RECOVERY FIX ---
    if (data->header1 != TELEMETRY_HEADER_1 || data->header2 != TELEMETRY_HEADER_2) {
        return false;
    }

    uint8_t calc_cs = 0;
    uint8_t* ptr = (uint8_t*)data;
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) calc_cs ^= ptr[i];

    if (calc_cs != data->checksum) {
        return false;
    }

    return true;
}
