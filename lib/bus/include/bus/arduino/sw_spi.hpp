/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2017 Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef BUS_ARDUINO_SW_SPI_HPP
#define BUS_ARDUINO_SW_SPI_HPP

#include "bus/arduino/spi.hpp"
#include <cstdint>
#ifndef TARGET_PLATFORM_ARDUINO
#error "This header is only for the Arduino framework."
#endif

#include <Arduino.h>

#include "bus/ibus.hpp"
#include "log/ilog.hpp"

#if !defined(SPI_INTERFACES_COUNT) || (defined(SPI_INTERFACES_COUNT) && (SPI_INTERFACES_COUNT > 0))
// HW SPI available
#include <SPI.h>
#define BUSIO_HAS_HW_SPI
#else
// SW SPI ONLY
enum { SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3 };
typedef uint8_t SPIClass;
#endif

// some modern SPI definitions don't have BitOrder enum
#if (defined(__AVR__) && !defined(ARDUINO_ARCH_MEGAAVR)) || defined(ESP8266)                       \
    || defined(TEENSYDUINO) || defined(SPARK) || defined(ARDUINO_ARCH_SPRESENSE)                   \
    || defined(MEGATINYCORE) || defined(DXCORE) || defined(ARDUINO_AVR_ATmega4809)                 \
    || defined(ARDUINO_AVR_ATmega4808) || defined(ARDUINO_AVR_ATmega3209)                          \
    || defined(ARDUINO_AVR_ATmega3208) || defined(ARDUINO_AVR_ATmega1609)                          \
    || defined(ARDUINO_AVR_ATmega1608) || defined(ARDUINO_AVR_ATmega809)                           \
    || defined(ARDUINO_AVR_ATmega808) || defined(ARDUINO_ARCH_ARC32) || defined(ARDUINO_ARCH_XMC)

typedef enum _BitOrder {
    SPI_BITORDER_MSBFIRST = MSBFIRST,
    SPI_BITORDER_LSBFIRST = LSBFIRST,
} BusIOBitOrder;

#elif defined(ESP32) || defined(__ASR6501__) || defined(__ASR6502__)

// some modern SPI definitions don't have BitOrder enum and have different SPI
// mode defines
typedef enum _BitOrder {
    SPI_BITORDER_MSBFIRST = SPI_MSBFIRST,
    SPI_BITORDER_LSBFIRST = SPI_LSBFIRST,
} BusIOBitOrder;

#else
// Some platforms have a BitOrder enum but its named MSBFIRST/LSBFIRST
#define SPI_BITORDER_MSBFIRST MSBFIRST
#define SPI_BITORDER_LSBFIRST LSBFIRST
typedef BitOrder BusIOBitOrder;
#endif

#if defined(__IMXRT1062__)  // Teensy 4.x
// *Warning* I disabled the usage of FAST_PINIO as the set/clear operations
// used in the cpp file are not atomic and can effect multiple IO pins
// and if an interrupt happens in between the time the code reads the register
//  and writes out the updated value, that changes one or more other IO pins
// on that same IO port, those change will be clobbered when the updated
// values are written back.  A fast version can be implemented that uses the
// ports set and clear registers which are atomic.
// typedef volatile uint32_t BusIO_PortReg;
// typedef uint32_t BusIO_PortMask;
// #define BUSIO_USE_FAST_PINIO

#elif defined(__MBED__) && defined(ARDUINO_ARCH_NRF52840)
// Arduino mbed core on nRF52840 (Nano 33 BLE): drive pins through the GPIO
// peripheral's atomic OUTSET/OUTCLR and IN registers. On this core digitalWrite
// and digitalRead cost ~1.5 us per call (measured) against ~0.1-0.2 us for the
// registers, and bit-banged SPI makes about four pin operations per bit.
#include "nrf52840.h"
#undef BUSIO_USE_FAST_PINIO
#define BUSIO_USE_NRF_PINIO

#elif defined(__MBED__) || defined(__ZEPHYR__)
// Boards based on RTOS cores like mbed or Zephyr are not going to expose the
// low level registers needed for fast pin manipulation
#undef BUSIO_USE_FAST_PINIO

#elif defined(ARDUINO_ARCH_XMC)
#undef BUSIO_USE_FAST_PINIO

#elif defined(__AVR__) || defined(TEENSYDUINO)
typedef volatile uint8_t BusIO_PortReg;
typedef uint8_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif defined(ESP8266) || defined(ESP32) || defined(__SAM3X8E__) || defined(ARDUINO_ARCH_SAMD)
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif (defined(__arm__) || defined(ARDUINO_FEATHER52)) && !defined(ARDUINO_ARCH_RP2040)            \
    && !defined(ARDUINO_SILABS) && !defined(ARDUINO_UNOR4_MINIMA) && !defined(ARDUINO_UNOR4_WIFI)  \
    && !defined(PORTDUINO)
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#if !defined(__ASR6501__) && !defined(__ASR6502__)
#define BUSIO_USE_FAST_PINIO
#endif

#else
#undef BUSIO_USE_FAST_PINIO
#endif

#ifdef BUSIO_USE_FAST_PINIO
#define BUSIO_SET_CLOCK_LOW()  (*clkPort = *clkPort & ~clkPinMask)
#define BUSIO_SET_CLOCK_HIGH() (*clkPort = *clkPort | clkPinMask)
#define BUSIO_READ_MISO()      (*misoPort & misoPinMask)
#define BUSIO_WRITE_MOSI(value)                                                                    \
    do {                                                                                           \
        if (value)                                                                                 \
            *mosiPort = *mosiPort | mosiPinMask;                                                   \
        else                                                                                       \
            *mosiPort = *mosiPort & ~mosiPinMask;                                                  \
    } while (0)
#elif defined(BUSIO_USE_NRF_PINIO)
#define BUSIO_SET_CLOCK_LOW()  (clkGpio->OUTCLR = clkPinMask)
#define BUSIO_SET_CLOCK_HIGH() (clkGpio->OUTSET = clkPinMask)
#define BUSIO_READ_MISO()      (misoGpio->IN & misoPinMask)
#define BUSIO_WRITE_MOSI(value)                                                                    \
    do {                                                                                           \
        if (value)                                                                                 \
            mosiGpio->OUTSET = mosiPinMask;                                                        \
        else                                                                                       \
            mosiGpio->OUTCLR = mosiPinMask;                                                        \
    } while (0)
#else
#define BUSIO_SET_CLOCK_LOW()   digitalWrite(_sck, LOW)
#define BUSIO_SET_CLOCK_HIGH()  digitalWrite(_sck, HIGH)
#define BUSIO_READ_MISO()       digitalRead(_miso)
#define BUSIO_WRITE_MOSI(value) digitalWrite(_mosi, value)
#endif

namespace bus::arduino {

class SWSPIBus : public ISPIBus {
  public:
    /**
     * @brief  Create an SPI device with the given CS pin and settings
     *
     * @param cspin The arduino pin number to use for chip select
     * @param sckpin The arduino pin number to use for SCK
     * @param misopin The arduino pin number to use for MISO, set to -1 if not used
     * @param mosipin The arduino pin number to use for MOSI, set to -1 if not used
     * @param freq The SPI clock frequency to use
     * @param dataOrder The SPI data order to use for bits within each byte
     * @param dataMode The SPI mode to use, defaults to SPI_MODE0
     */
    SWSPIBus(
        int8_t cs_pin,
        int8_t sck_pin,
        int8_t miso_pin,
        int8_t mosi_pin,
        logger::ILogger *logger,
        uint32_t freq            = 1000000,
        BusIOBitOrder data_order = SPI_BITORDER_MSBFIRST,
        uint8_t data_mode        = SPI_MODE0
    ) noexcept
        : _cs(cs_pin), _sck(sck_pin), _miso(miso_pin), _mosi(mosi_pin), _freq(freq),
          _dataMode(data_mode), _dataOrder(data_order), logger_(logger) {
#ifdef BUSIO_USE_FAST_PINIO
        csPort    = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(cs_pin));
        csPinMask = digitalPinToBitMask(cs_pin);
        if (mosi_pin != -1) {
            mosiPort    = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(mosi_pin));
            mosiPinMask = digitalPinToBitMask(mosi_pin);
        }
        if (miso_pin != -1) {
            misoPort    = (BusIO_PortReg *)portInputRegister(digitalPinToPort(miso_pin));
            misoPinMask = digitalPinToBitMask(miso_pin);
        }
        clkPort    = (BusIO_PortReg *)portOutputRegister(digitalPinToPort(sck_pin));
        clkPinMask = digitalPinToBitMask(sck_pin);
#endif
#ifdef BUSIO_USE_NRF_PINIO
        nrf_pin_(sck_pin, clkGpio, clkPinMask);
        if (mosi_pin != -1) { nrf_pin_(mosi_pin, mosiGpio, mosiPinMask); }
        if (miso_pin != -1) { nrf_pin_(miso_pin, misoGpio, misoPinMask); }
#endif
    }

    /* Bus is not copyable, but it is movable. */
    SWSPIBus(const SWSPIBus &)            = delete;
    SWSPIBus &operator=(const SWSPIBus &) = delete;
    /* */

    SWSPIBus(SWSPIBus &&other) noexcept
        /* Invalidate the other bus' hardware state. */
        : _cs(std::exchange(other._cs, -1)), _sck(std::exchange(other._sck, -1)),
          _miso(std::exchange(other._miso, -1)), _mosi(std::exchange(other._mosi, -1)),
          _freq(std::exchange(other._freq, 0)), _dataMode(std::exchange(other._dataMode, 0)),
          _dataOrder(std::exchange(other._dataOrder, SPI_BITORDER_MSBFIRST)),
          locked_(std::exchange(other.locked_, false)), _begun(std::exchange(other._begun, false)),
          logger_(std::exchange(other.logger_, nullptr)) {
#ifdef BUSIO_USE_FAST_PINIO
        csPort      = std::exchange(other.csPort, nullptr);
        csPinMask   = std::exchange(other.csPinMask, 0);
        mosiPort    = std::exchange(other.mosiPort, nullptr);
        mosiPinMask = std::exchange(other.mosiPinMask, 0);
        misoPort    = std::exchange(other.misoPort, nullptr);
        misoPinMask = std::exchange(other.misoPinMask, 0);
        clkPort     = std::exchange(other.clkPort, nullptr);
        clkPinMask  = std::exchange(other.clkPinMask, 0);
#endif
#ifdef BUSIO_USE_NRF_PINIO
        clkGpio     = std::exchange(other.clkGpio, nullptr);
        clkPinMask  = std::exchange(other.clkPinMask, 0);
        mosiGpio    = std::exchange(other.mosiGpio, nullptr);
        mosiPinMask = std::exchange(other.mosiPinMask, 0);
        misoGpio    = std::exchange(other.misoGpio, nullptr);
        misoPinMask = std::exchange(other.misoPinMask, 0);
#endif
    }

    SWSPIBus &operator=(SWSPIBus &&other) noexcept {
        if (this != &other) {
            /* Invalidate the other bus' hardware state. */
            this->_cs        = std::exchange(other._cs, -1);
            this->_sck       = std::exchange(other._sck, -1);
            this->_miso      = std::exchange(other._miso, -1);
            this->_mosi      = std::exchange(other._mosi, -1);
            this->_freq      = std::exchange(other._freq, 0);
            this->_dataMode  = std::exchange(other._dataMode, 0);
            this->_dataOrder = std::exchange(other._dataOrder, SPI_BITORDER_MSBFIRST);
            this->locked_    = std::exchange(other.locked_, false);
            this->_begun     = std::exchange(other._begun, false);
            this->logger_    = std::exchange(other.logger_, nullptr);

#ifdef BUSIO_USE_FAST_PINIO
            this->csPort      = std::exchange(other.csPort, nullptr);
            this->csPinMask   = std::exchange(other.csPinMask, 0);
            this->mosiPort    = std::exchange(other.mosiPort, nullptr);
            this->mosiPinMask = std::exchange(other.mosiPinMask, 0);
            this->misoPort    = std::exchange(other.misoPort, nullptr);
            this->misoPinMask = std::exchange(other.misoPinMask, 0);
            this->clkPort     = std::exchange(other.clkPort, nullptr);
            this->clkPinMask  = std::exchange(other.clkPinMask, 0);
#endif
#ifdef BUSIO_USE_NRF_PINIO
            this->clkGpio     = std::exchange(other.clkGpio, nullptr);
            this->clkPinMask  = std::exchange(other.clkPinMask, 0);
            this->mosiGpio    = std::exchange(other.mosiGpio, nullptr);
            this->mosiPinMask = std::exchange(other.mosiPinMask, 0);
            this->misoGpio    = std::exchange(other.misoGpio, nullptr);
            this->misoPinMask = std::exchange(other.misoPinMask, 0);
#endif
        }
        return *this;
    }

    ~SWSPIBus() noexcept override = default;

    /**
     * @brief  Initializes SPI bus and sets CS pin high
     *
     */
    void begin() noexcept {
        if (_cs != -1) {
            pinMode(_cs, OUTPUT);
            digitalWrite(_cs, HIGH);
        }

        pinMode(_sck, OUTPUT);

        if ((_dataMode == SPI_MODE0) || (_dataMode == SPI_MODE1)) {
            // idle low on mode 0 and 1
            digitalWrite(_sck, LOW);
        } else {
            // idle high on mode 2 or 3
            digitalWrite(_sck, HIGH);
        }
        if (_mosi != -1) {
            pinMode(_mosi, OUTPUT);
            digitalWrite(_mosi, HIGH);
        }
        if (_miso != -1) { pinMode(_miso, INPUT); }

        _begun = true;
    }

    /**
     * @brief  Transfer (send/receive) a buffer over hard/soft SPI, without transaction management
     * @param  buffer The buffer to send and receive at the same time
     * @param  len    The number of bytes to transfer
     *
     */
    void transfer(uint8_t *buffer, size_t len) noexcept {
        uint8_t startbit;
        if (_dataOrder == SPI_BITORDER_LSBFIRST) {
            startbit = 0x1;
        } else {
            startbit = 0x80;
        }

        bool towrite, lastmosi = !(buffer[0] & startbit);
        uint8_t bitdelay_us = (1000000 / _freq) / 2;

        for (size_t i = 0; i < len; i++) {
            uint8_t reply = 0;
            uint8_t send  = buffer[i];

            /*
            Serial.print("\tSending software SPI byte 0x");
            Serial.print(send, HEX);
            Serial.print(" -> 0x");
            */

            // Serial.print(send, HEX);
            for (uint8_t b = startbit; b != 0;
                 b         = (_dataOrder == SPI_BITORDER_LSBFIRST) ? b << 1 : b >> 1) {

                if (bitdelay_us) { delayMicroseconds(bitdelay_us); }

                if (_dataMode == SPI_MODE0 || _dataMode == SPI_MODE2) {
                    towrite = send & b;
                    if ((_mosi != -1) && (lastmosi != towrite)) {
                        BUSIO_WRITE_MOSI(towrite);
                        lastmosi = towrite;
                    }

                    BUSIO_SET_CLOCK_HIGH();

                    if (bitdelay_us) { delayMicroseconds(bitdelay_us); }

                    if (_miso != -1) {
                        if (BUSIO_READ_MISO()) reply |= b;
                    }

                    BUSIO_SET_CLOCK_LOW();

                } else if (_dataMode == SPI_MODE3) {

                    if (_mosi != -1) {  // transmit on falling edge
                        BUSIO_WRITE_MOSI(send & b);
                    }

                    BUSIO_SET_CLOCK_LOW();

                    if (bitdelay_us) { delayMicroseconds(bitdelay_us); }

                    BUSIO_SET_CLOCK_HIGH();

                    if (bitdelay_us) { delayMicroseconds(bitdelay_us); }

                    if (_miso != -1) {  // read on rising edge
                        if (BUSIO_READ_MISO()) { reply |= b; }
                    }

                } else {  // || _dataMode == SPI_MODE1)

                    BUSIO_SET_CLOCK_HIGH();

                    if (bitdelay_us) { delayMicroseconds(bitdelay_us); }

                    if (_mosi != -1) { BUSIO_WRITE_MOSI(send & b); }

                    BUSIO_SET_CLOCK_LOW();

                    if (_miso != -1) {
                        if (BUSIO_READ_MISO()) { reply |= b; }
                    }
                }
            }
            if (_miso != -1) { buffer[i] = reply; }
        }
        return;
    }

    /**
     * @brief  Transfer (send/receive) one byte over hard/soft SPI, without transaction management
     * @param  send The byte to send
     * @return The byte received while transmitting
     *
     */
    uint8_t transfer(uint8_t send) noexcept {
        uint8_t data = send;
        transfer(&data, 1);
        return data;
    }

    void beginTransaction() noexcept {}
    void endTransaction() noexcept {}

    void lock() noexcept override {
        asm volatile("nop \n nop \n nop");
        this->beginTransaction();
        if (this->_cs != -1) { digitalWrite(this->_cs, !SWSPIBus::spi_active_low_); }
        this->locked_ = true;
        asm volatile("nop \n nop \n nop");
    }

    void unlock() noexcept override {
        asm volatile("nop \n nop \n nop");
        if (this->_cs != -1) { digitalWrite(this->_cs, SWSPIBus::spi_active_low_); }
        this->endTransaction();
        this->locked_ = false;
        asm volatile("nop \n nop \n nop");
    }

    tl::expected<int, bus::BusError> write_register(reg_type reg, span<const element_type> buffer)
        override {
        if (!this->locked_) {
            this->logger_->error("{:d}: attempted to write while bus is not locked", this->_cs);
            return tl::make_unexpected(BusError::NOT_LOCKED);
        }

        this->transfer(reg);
        for (const auto &byte : buffer) {
            this->transfer(byte);
        }

        return buffer.size();
    }

    tl::expected<int, bus::BusError> read_register(reg_type reg, span<element_type> buffer)
        override {
        if (!this->locked_) {
            this->logger_->error("{:d}: attempted to read while bus is not locked", this->_cs);
            return tl::make_unexpected(BusError::NOT_LOCKED);
        }

        this->transfer(reg);
        for (auto &byte : buffer) {
            byte = this->transfer(0x00);
        }

        return buffer.size();
    }

  private:
    int8_t _cs;
    int8_t _sck;
    int8_t _miso;
    int8_t _mosi;
    uint32_t _freq;
    uint8_t _dataMode;
    BusIOBitOrder _dataOrder;

    static constexpr bool spi_active_low_ = true;
    bool locked_                          = false;
    bool _begun                           = false;

#ifdef BUSIO_USE_FAST_PINIO
    BusIO_PortReg *mosiPort, *clkPort, *misoPort, *csPort;
    BusIO_PortMask mosiPinMask, misoPinMask, clkPinMask, csPinMask;
#endif
#ifdef BUSIO_USE_NRF_PINIO
    NRF_GPIO_Type *clkGpio = nullptr, *mosiGpio = nullptr, *misoGpio = nullptr;
    uint32_t clkPinMask = 0, mosiPinMask = 0, misoPinMask = 0;

    /** Resolve an Arduino pin to its nRF GPIO port (P0 or P1) and bit mask. */
    static void nrf_pin_(int8_t pin, NRF_GPIO_Type *&port, uint32_t &mask) noexcept {
        const int name = static_cast<int>(digitalPinToPinName(pin));
        port           = name >= 32 ? NRF_P1 : NRF_P0;
        mask           = 1UL << (name & 31);
    }
#endif

    logger::ILogger *logger_;
};

} /* namespace bus::arduino */

#endif /* BUS_ARDUINO_SW_SPI_HPP */
