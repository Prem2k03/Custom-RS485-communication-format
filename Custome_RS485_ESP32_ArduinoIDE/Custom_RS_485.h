/*
 * Custom_RS_485.h  —  Revised to match protocol diagram & data-byte spec
 *
 * ─── COMMAND BYTE (Master → All Slaves) ──────────────────────────────────────
 *  Bit  7   : Data width       — 1 = 16-bit transfer, 0 = 8-bit transfer
 *  Bit 6-4  : Register address (0–7, up to 8 registers)
 *  Bit  3   : Operation        — 1 = Read, 0 = Write
 *  Bit 2-0  : Slave ID         (1–7)
 *
 * ─── HANDSHAKE ────────────────────────────────────────────────────────────────
 *  Master sends command byte
 *    → Addressed slave  : replies 'Y', then handles read or waits for write data
 *    → All other slaves : go DEAF — they watch the D-bit of each incoming byte
 *                         and return to IDLE once they see D-bit = 1 (last byte)
 *
 *  On READ  : addressed slave sends data bytes immediately after 'Y'
 *  On WRITE : master sends data bytes after receiving 'Y' from slave
 *
 * ─── DATA BYTE FORMAT (D-bit protocol) ───────────────────────────────────────
 *
 *  Every data byte on the bus looks like:
 *
 *      bit 7  = D-bit  →  0 = "more data bytes follow"
 *                          1 = "this is the LAST data byte"
 *      bits 6-0 = payload bits (7 bits of actual data per byte)
 *
 *  8-bit value  → 2 data bytes  (1 + 7 = 8 payload bits)
 *  ┌─────────────────────────────────────────────────────────┐
 *  │ Byte 1:  D=0 | 0 0 0 0 0 0 | val[7]                   │
 *  │          [7]   [6 5 4 3 2 1]  [0]                      │
 *  │ Byte 2:  D=1 | val[6] val[5] val[4] val[3] val[2] val[1] val[0] │
 *  │          [7]   [6      5      4      3      2      1    0]       │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  16-bit value → 3 data bytes  (1 + 7 + 7 = 15 bits... but we need 16)
 *  Actually: Byte1 carries bits[15:15] (1 bit), Byte2 carries bits[14:8] (7 bits),
 *            Byte3 carries bits[7:1] (7 bits) — wait, that leaves bit0 unaccounted.
 *
 *  Correct split for 16-bit (same 7-payload-bits-per-byte rule):
 *  ┌─────────────────────────────────────────────────────────┐
 *  │ Byte 1:  D=0 | 0 0 0 0 0 0 | val[15]                  │
 *  │ Byte 2:  D=0 | val[14..8]   (7 bits)                   │
 *  │ Byte 3:  D=1 | val[7..1]    (7 bits)  ← NOTE: val[0] lost │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  To preserve all 16 bits we extend to full 7-bit chunks:
 *  ┌─────────────────────────────────────────────────────────┐
 *  │ Byte 1:  D=0 | 0 0 0 0 0 | val[15] val[14]             │  ← 2 MSBs
 *  │ Byte 2:  D=0 | val[13..7]  (7 bits)                    │
 *  │ Byte 3:  D=1 | val[6..0]   (7 bits)  ← last            │
 *  └─────────────────────────────────────────────────────────┘
 *  This packs 2+7+7 = 16 bits exactly.
 *
 * ─── DEAF STATE ───────────────────────────────────────────────────────────────
 *  Non-addressed slaves watch the D-bit of every byte they receive.
 *  They stay DEAF while D-bit = 0, and return to IDLE when D-bit = 1.
 *  This is self-synchronising — no byte-count table needed.
 *
 *  Deaf entry: on seeing a command byte not addressed to us, the slave
 *  immediately goes DEAF. It will exit when it sees a byte with D=1.
 *
 *  Special case: if the command byte's bit-7 (16-bit flag) is 0 AND the
 *  operation is a read, the slave's reply bytes have D-bits — the deaf slave
 *  just treats them the same way (watches D-bit to know when transaction ends).
 */

#ifndef CUSTOM_RS_485_H
#define CUSTOM_RS_485_H

#include <Arduino.h>
//#include <SoftwareSerial.h>

// ── Tunables ──────────────────────────────────────────────────────────────────
#define MAX_REGISTERS   8
#define MAX_SLAVE_ID    7
#define RX_TIMEOUT_MS   100

// ── Port & pin ────────────────────────────────────────────────────────────────
static HardwareSerial*  _rs485          = nullptr;
static int              _re_de_pin      = -1;
static unsigned long    _byte_period_us = 1042;   // default 9600 baud

// ── Slave identity & registers ────────────────────────────────────────────────
static uint8_t  _my_slave_id              = 0;
static uint16_t  _registers[MAX_REGISTERS] = {0};
static uint8_t  _reg_count                = MAX_REGISTERS;

// ── State machine ─────────────────────────────────────────────────────────────
typedef enum {
    STATE_IDLE,        // waiting for a command byte
    STATE_WRITE_BYTE1, // addressed for 8-bit write: waiting for first data byte
    STATE_WRITE_BYTE2, // waiting for second data byte (8-bit or 16-bit mid)
    STATE_WRITE_BYTE3, // waiting for third data byte (16-bit only)
    STATE_DEAF         // not our transaction: watching D-bit to know when done
} SlaveState;

static SlaveState _state         = STATE_IDLE;
static uint8_t    _pending_reg   = 0;
static bool       _pending_16bit = false;
static uint16_t   _pending_val   = 0;   // accumulates incoming write data

// ═════════════════════════════════════════════════════════════════════════════
//  LOW-LEVEL HELPERS
// ═════════════════════════════════════════════════════════════════════════════

static void _send_byte(uint8_t b) {
    digitalWrite(_re_de_pin, HIGH);
    delayMicroseconds(10);
    _rs485->write(b);
    _rs485->flush();
    delayMicroseconds(_byte_period_us);
    digitalWrite(_re_de_pin, LOW);
    delayMicroseconds(10);
}

static int _recv_byte(void) {
    unsigned long start = millis();
    while (!_rs485->available()) {
        if ((millis() - start) >= RX_TIMEOUT_MS) return -1;
    }
    return _rs485->read();
}

static void _flush_echo(uint8_t n) {
    for (uint8_t i = 0; i < n; i++) _recv_byte();
}

// ─── Build data bytes from a value ──────────────────────────────────────────
//
//  8-bit encoding  → 2 bytes
//    byte1: D=0, bits[6:1]=0, bit0 = val[7]
//    byte2: D=1, bits[6:0]  = val[6:0]
//
//  16-bit encoding → 3 bytes
//    byte1: D=0, bits[6:2]=0, bits[1:0] = val[15:14]
//    byte2: D=0, bits[6:0]  = val[13:7]
//    byte3: D=1, bits[6:0]  = val[6:0]

static void _encode_8(uint8_t val, uint8_t *b1, uint8_t *b2) {
    *b1 = (uint8_t)((val >> 7) & 0x01);          // D=0, bit0=val[7]
    *b2 = (uint8_t)((val & 0x7F) | 0x80);         // D=1, bits[6:0]=val[6:0]
}

static void _encode_16(uint16_t val, uint8_t *b1, uint8_t *b2, uint8_t *b3) {
    *b1 = (uint8_t)((val >> 14) & 0x03);          // D=0, bits[1:0]=val[15:14]
    *b2 = (uint8_t)((val >> 7)  & 0x7F);          // D=0, bits[6:0]=val[13:7]
    *b3 = (uint8_t)((val & 0x7F) | 0x80);         // D=1, bits[6:0]=val[6:0]
}

// ─── Decode data bytes back to a value ──────────────────────────────────────

static uint8_t _decode_8(uint8_t b1, uint8_t b2) {
    return (uint8_t)(((b1 & 0x01) << 7) | (b2 & 0x7F));
}

static uint16_t _decode_16(uint8_t b1, uint8_t b2, uint8_t b3) {
    return (uint16_t)(
        ((uint16_t)(b1 & 0x03) << 14) |
        ((uint16_t)(b2 & 0x7F) << 7)  |
        ((uint16_t)(b3 & 0x7F))
    );
}

// ═════════════════════════════════════════════════════════════════════════════
//  PUBLIC API — COMMON
// ═════════════════════════════════════════════════════════════════════════════

// Modified rs485_begin() for ESP32
void rs485_begin(HardwareSerial &serial, int re_de_pin, long baud, int rxpin, int txpin) {
    _re_de_pin      = re_de_pin;
    _byte_period_us = (10000000UL / (unsigned long)baud) + 1;
    _rs485          = &serial;
    _rs485->begin(baud, SERIAL_8N1, rxpin, txpin);  // ESP32 needs pin numbers
    pinMode(_re_de_pin, OUTPUT);
    digitalWrite(_re_de_pin, LOW);
}


// ═════════════════════════════════════════════════════════════════════════════
//  PUBLIC API — SLAVE SIDE
// ═════════════════════════════════════════════════════════════════════════════

void slave_begin(uint8_t slave_id, uint8_t reg_count) {
    _my_slave_id = slave_id;
    _reg_count   = (reg_count <= MAX_REGISTERS) ? reg_count : MAX_REGISTERS;
    memset(_registers, 0, sizeof(_registers));
    _state = STATE_IDLE;
}

void slave_write_register(uint8_t reg_addr, uint16_t value) {
    if (reg_addr < _reg_count) _registers[reg_addr] = value;
}

uint16_t slave_read_register(uint8_t reg_addr) {
    if (reg_addr < _reg_count) return _registers[reg_addr];
    return 0;
}

void slave_process(void) {
    if (!_rs485->available()) return;

    uint8_t b = (uint8_t)_rs485->read();

    switch (_state) {

    // ── Waiting for a master command byte ────────────────────────────────────
    case STATE_IDLE: {
        uint8_t target_id = (b >> 0) & 0x07;
        bool    is_read   = (b >> 3) & 0x01;
        uint8_t reg_addr  = (b >> 4) & 0x07;
        bool    use_16bit = (b >> 7) & 0x01;

        if (target_id == _my_slave_id) {
            // ── This command is for us ─────────────────────────────────────
            if (reg_addr >= _reg_count) return;   // unknown register — stay silent
            
            _send_byte('Y');
            _pending_reg   = reg_addr;
            _pending_16bit = use_16bit;
            _pending_val   = 0;

            if (is_read) {
                // Send register value back to master using D-bit encoding
                uint16_t val = (uint16_t)_registers[reg_addr];
                if (use_16bit) {
                    uint8_t b1, b2, b3;
                    _encode_16(val, &b1, &b2, &b3);
                    _send_byte(b1);
                    _send_byte(b2);
                    _send_byte(b3);
                } else {
                    uint8_t v8 = (val > 255) ? 255 : (uint8_t)val;
                    uint8_t b1, b2;
                    _encode_8(v8, &b1, &b2);
                    _send_byte(b1);
                    _send_byte(b2);
                }
                _state = STATE_IDLE;
            } else {
                // Wait for master to send data bytes
                _state = STATE_WRITE_BYTE1;
            }

        } else {
            // ── Not our command — go DEAF until we see D-bit = 1 ──────────
            // The command byte itself has no D-bit; just go deaf now.
            _state = STATE_DEAF;
        }
        break;
    }

    // ── Receiving write data from master: first byte ─────────────────────────
    case STATE_WRITE_BYTE1:
        _pending_val = (uint16_t)(b & 0x7F);    // strip D-bit, save payload
        if (b & 0x80) {
            // D=1 already? That would only happen for a 1-byte transfer (not
            // defined in this protocol, but handle gracefully).
            _registers[_pending_reg] = (uint16_t)_pending_val;
            _state = STATE_IDLE;
        } else {
            _state = STATE_WRITE_BYTE2;
        }
        break;

    // ── Receiving write data: second byte ────────────────────────────────────
    case STATE_WRITE_BYTE2:
        if (!_pending_16bit) {
            // 8-bit transfer: byte1 held val[7], byte2 holds val[6:0], D=1
            uint8_t val8 = _decode_8((uint8_t)_pending_val, b);
            _registers[_pending_reg] = (uint16_t)val8;
            _state = STATE_IDLE;
        } else {
            // 16-bit transfer: still expecting a third byte
            _pending_val = (uint16_t)((_pending_val << 7) | (b & 0x7F));
            if (b & 0x80) {
                // D=1 early — treat as complete (shouldn't happen in normal flow)
                _registers[_pending_reg] = (uint16_t)_pending_val;
                _state = STATE_IDLE;
            } else {
                _state = STATE_WRITE_BYTE3;
            }
        }
        break;

    // ── Receiving write data: third byte (16-bit only, D=1 expected) ─────────
    case STATE_WRITE_BYTE3: {
        // b1 bits[1:0] = val[15:14]  already in _pending_val[13:0] from byte2
        // Actually: pending_val after BYTE2 = (b1_payload << 7) | b2_payload
        //           = val[15:14] << 7  | val[13:7]
        // Now byte3 payload = val[6:0]
        uint16_t full = (uint16_t)((_pending_val << 7) | (b & 0x7F));
        // Reconstruct: full = (b1[1:0] << 14) | (b2[6:0] << 7) | b3[6:0]
        // That is: bits[15:14] | bits[13:7] | bits[6:0] = all 16 bits
        _registers[_pending_reg] = (uint16_t)full;
        _state = STATE_IDLE;
        break;
    }

    // ── Not our transaction — wait for D-bit = 1 to return to IDLE ───────────
    case STATE_DEAF:
        if (b & 0x80) {          // D-bit set → last byte of this transaction
            _state = STATE_IDLE;
        }
        // else stay DEAF
        break;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  PUBLIC API — MASTER SIDE
// ═════════════════════════════════════════════════════════════════════════════

static uint8_t _build_cmd(uint8_t slave_id, uint8_t reg_addr,
                           bool is_read, bool use_16bit) {
    return (uint8_t)(
        ( slave_id  & 0x07)       |    // bits 2-0 : slave ID
        ((is_read  ? 1 : 0) << 3) |    // bit  3   : R/W
        ((reg_addr & 0x07)  << 4) |    // bits 6-4 : register address
        ((use_16bit? 1 : 0) << 7)      // bit  7   : 16-bit flag
    );
}

/**
 * Write an 8-bit value to a slave register.
 * Data bytes use D-bit encoding (2 bytes for 8-bit value).
 * @return true on success, false on timeout/NAK.
 */
bool master_write_8(uint8_t slave_id, uint8_t reg_addr, uint8_t value) {
    _send_byte(_build_cmd(slave_id, reg_addr, false, false));
    _flush_echo(1);
    if (_recv_byte() != 'Y') return false;

    uint8_t b1, b2;
    _encode_8(value, &b1, &b2);
    _send_byte(b1);
    _flush_echo(1);
    _send_byte(b2);
    _flush_echo(1);
    return true;
}

/**
 * Write a 16-bit value to a slave register.
 * Data bytes use D-bit encoding (3 bytes for 16-bit value).
 * @return true on success, false on timeout/NAK.
 */
bool master_write_16(uint8_t slave_id, uint8_t reg_addr, uint16_t value) {
    _send_byte(_build_cmd(slave_id, reg_addr, false, true));
    _flush_echo(1);
    if (_recv_byte() != 'Y') return false;

    uint8_t b1, b2, b3;
    _encode_16(value, &b1, &b2, &b3);
    _send_byte(b1);
    _flush_echo(1);
    _send_byte(b2);
    _flush_echo(1);
    _send_byte(b3);
    _flush_echo(1);
    return true;
}

/**
 * Read an 8-bit value from a slave register.
 * Slave sends 2 D-bit-encoded bytes.
 * @return true on success, false on timeout/error.
 */
bool master_read_8(uint8_t slave_id, uint8_t reg_addr, uint8_t *out) {
    _send_byte(_build_cmd(slave_id, reg_addr, true, false));
    _flush_echo(1);
    if (_recv_byte() != 'Y') return false;

    int r1 = _recv_byte();
    int r2 = _recv_byte();
    if (r1 < 0 || r2 < 0) return false;

    *out = _decode_8((uint8_t)r1, (uint8_t)r2);
    return true;
}

/**
 * Read a 16-bit value from a slave register.
 * Slave sends 3 D-bit-encoded bytes.
 * @return true on success, false on timeout/error.
 */
bool master_read_16(uint8_t slave_id, uint8_t reg_addr, uint16_t *out) {
    _send_byte(_build_cmd(slave_id, reg_addr, true, true));
    _flush_echo(1);
    if (_recv_byte() != 'Y') return false;

    int r1 = _recv_byte();
    int r2 = _recv_byte();
    int r3 = _recv_byte();
    if (r1 < 0 || r2 < 0 || r3 < 0) return false;

    *out = _decode_16((uint8_t)r1, (uint8_t)r2, (uint8_t)r3);
    return true;
}

#endif // CUSTOM_RS_485_H

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  USAGE EXAMPLES
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  ── SLAVE SKETCH ────────────────────────────────────────────────────────────
 *  #include "Custom_RS_485.h"
 *
 *  void setup() {
 *      rs485_begin(10, 11, 3, 9600);   // RX=10, TX=11, RE_DE=3
 *      slave_begin(2, 4);               // Slave ID=2, 4 registers
 *  }
 *
 *  void loop() {
 *      slave_write_register(0, (int16_t)analogRead(A0));
 *      slave_process();
 *  }
 *
 *  ── MASTER SKETCH ───────────────────────────────────────────────────────────
 *  #include "Custom_RS_485.h"
 *
 *  void setup() {
 *      rs485_begin(10, 11, 3, 9600);
 *  }
 *
 *  void loop() {
 *      master_write_8(2, 1, 0xAB);
 *
 *      uint8_t val8;
 *      if (master_read_8(2, 0, &val8))   Serial.println(val8);
 *
 *      master_write_16(3, 2, 1234);
 *
 *      uint16_t val16;
 *      if (master_read_16(3, 2, &val16)) Serial.println(val16);
 *
 *      delay(500);
 *  }
 */