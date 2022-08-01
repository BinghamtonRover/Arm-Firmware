
#pragma once

#if defined(ARDUINO_ARCH_AVR)

    #include <Arduino.h>
	#include <SoftwareSerial.h>
    #include <SPI.h>

    #define SW_CAPABLE_PLATFORM true

    namespace TMC_HAL {
        using PinDef = uint8_t;

        struct PinCache {
            PinCache(const uint8_t p, const uint8_t bm, volatile uint8_t* const ptr);
            const uint8_t port;
            const uint8_t bitMask;
            volatile uint8_t* const pPort = nullptr;
        };

        class SWSerial : public SoftwareSerial {
            public:
                SWSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false) :
                    SoftwareSerial(receivePin, transmitPin, inverse_logic),
                    RXTX_pin(receivePin == transmitPin ? receivePin : 0)
                    {}

                virtual size_t write(const uint8_t *buffer, size_t size) {
                    if (RXTX_pin > 0) {
                        digitalWrite(RXTX_pin, HIGH);
                        pinMode(RXTX_pin, OUTPUT);
                    }

                    const size_t bytesOut = SoftwareSerial::write(buffer, size);

                    if (RXTX_pin > 0) {
                        pinMode(RXTX_pin, INPUT_PULLUP);
                    }

                    return bytesOut;
                }

            private:
                const PinDef RXTX_pin; // Half duplex
        };
    }

#elif defined(ARDUINO_ARCH_SAM)

    #include <Arduino.h>
	#include <HardwareSerial.h>
	#include <SPI.h>

    #define SW_CAPABLE_PLATFORM false

    namespace TMC_HAL {
        using PinDef = uint32_t;

        struct PinCache {
            explicit PinCache(const PinDef _pin) :
                pin(_pin) {}
            const PinDef pin;
        };
    }

#elif defined(TARGET_LPC1768)

    #include <Arduino.h>
	#include <HardwareSerial.h>
	#include <SoftwareSerial.h>
	
    #define SW_CAPABLE_PLATFORM true

    namespace TMC_HAL {
        using SWSerial = SoftwareSerial;
        using PinDef = uint16_t;

        struct PinCache {
            explicit PinCache(const PinDef _pin) :
                pin(_pin)
                {}
            uint16_t const pin;
            static constexpr uint_fast8_t pinDelay = 60;
        };
    }

    #define HardwareSerial HardwareSerial<>

    #ifdef __MARLIN_FIRMWARE__
        #include <SPI.h>
    #else
        #include <SoftwareSPI.h>

        struct SPISettings {
            SPISettings(...) {}
        };

        struct SPIClass { // Should be removed when LPC core gets full SPI class implementation
            SPIClass(const uint8_t spi_speed, const pin_t sck_pin, const pin_t miso_pin, const pin_t mosi_pin) :
                mosi(mosi_pin), miso(miso_pin), sck(sck_pin), speed(spi_speed) {
                    swSpiInit(spi_speed, sck_pin, mosi_pin);
                }

            void begin(...) {}

            void beginTransaction(...) const {
                swSpiBegin(sck, miso, mosi);
            }

            uint8_t transfer(uint8_t data) const {
                return swSpiTransfer(data, speed, sck, miso, mosi);
            }

            private:
                const pin_t mosi, miso, sck;
                const uint8_t speed;
        };
    #endif

    namespace TMC_HAL {
        using LPC176x::delay_ns;
    }

#elif defined(ARDUINO)

    #include <Arduino.h>
    #include <SPI.h>
    #include <HardwareSerial.h>

    #if defined(ARDUINO_ARCH_STM32)
        #include <SoftwareSerial.h>
	    #define SW_CAPABLE_PLATFORM true

        namespace TMC_HAL {
            using SWSerial = SoftwareSerial;
        }
    #else
        #define SW_CAPABLE_PLATFORM false
    #endif

    namespace TMC_HAL {
        using PinDef = uint8_t;

        struct PinCache {
            explicit PinCache(const PinDef _pin) :
                pin(_pin)
                {}
            const PinDef pin;
        };
    }

#elif defined(__MBED__)

    #include <mbed.h>

    namespace TMC_HAL {
        using PinDef = PinName;

        struct PinCache {
            explicit PinCache(const PinDef _pin) :
                pin(_pin)
                {}

            /// All other HALs can treat the object as const
            mutable DigitalInOut pin;
        };
    }

    using SPIClass = SPI;
    using HardwareSerial = BufferedSerial;

    inline void delay(size_t ms) { wait_us(1000*ms); }
    namespace TMC_HAL {
        inline void delay_ns(unsigned int ns) { wait_ns(ns); }
    }

#elif (defined(USE_FULL_LL_DRIVER) || defined(USE_HAL_DRIVER))

    #include <cstddef>
    #include <stdint.h>

    #if defined(USE_FULL_LL_DRIVER)
        extern "C" {
          #include "main.h"
        }

        #if defined(STM32F0xx_LL_USART_H) || defined(__STM32F1xx_LL_USART_H) || defined(STM32F3xx_LL_USART_H) || defined(__STM32F4xx_LL_USART_H) || defined(STM32F7xx_LL_USART_H)
            #define STM_HAS_LL_UART
        #endif
        #if defined(STM32F0xx_LL_SPI_H) || defined(STM32F1xx_LL_SPI_H) || defined(STM32F3xx_LL_SPI_H) || defined(STM32F4xx_LL_SPI_H) || defined(STM32F7xx_LL_SPI_H)
            #define STM_HAS_LL_SPI
        #endif

    #elif defined(USE_HAL_DRIVER)
        #if defined(STM32F3xx)
            #include <stm32f3xx_hal.h>
        #elif defined(STM32F4xx)
            #include <stm32f4xx_hal.h>
        #else
            #include "main.h"
        #endif
    #else
        #include "main.h"
    #endif

    namespace TMC_HAL {

        struct PinDef {
            GPIO_TypeDef* const port;
            uint32_t const bm;
        };

        struct PinCache {
            explicit PinCache(const PinDef _pin) :
                pin(_pin)
                {}

            const PinDef pin;

            bool operator ==(const PinDef &p2) {
                return (p2.port == pin.port) && (p2.bm == pin.bm);
            }
        };
    }

        #if defined(STM_HAS_LL_SPI)
            using SPIClass = SPI_TypeDef;
        #elif defined(HAL_SPI_MODULE_ENABLED)
            using SPIClass = SPI_HandleTypeDef;
        #else
            using SPIClass = void*;
        #endif

        #if defined(STM_HAS_LL_UART)
            using HardwareSerial = USART_TypeDef;
        #elif defined(HAL_UART_MODULE_ENABLED)
            using HardwareSerial = UART_HandleTypeDef;
        #else
            using HardwareSerial = void*;
        #endif

        static constexpr uint32_t timeout = 1000;

        void delay(uint32_t ms);

#elif defined(__linux__) // Tested with RaspberryPi 3B

    #include <stdio.h>
    #include <stdlib.h>
    #include <stdint.h>
    #include <iostream>
    #include <string.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <errno.h>
    #include <sys/time.h>
    #include <termios.h>
    #include <sys/ioctl.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <gpiod.hpp>
    #include <cstdlib>
    #include <getopt.h>
    #include <linux/ioctl.h>
    #include <linux/types.h>
    #include <linux/spi/spidev.h>
    #include <sys/time.h>

    namespace TMC_HAL {
        using PinDef = ::gpiod::line;

        struct PinCache {
            explicit PinCache(const PinDef _pin) :
                pin(_pin)
                {}
            const PinDef pin;
            static constexpr bool LOW = 0;
            static constexpr bool HIGH = 1;
        };

        struct HW_port {
            HW_port(std::string_view port);
            ~HW_port() { ::close(fd); }
            int fd = -1;
        };

        // Ensure CS pin timings requirements
        inline void delay_ns(unsigned int ns) {
            const uint_fast16_t us = ns / 1000 + 1; // No API for ns sleep
            usleep(us);
        }
    }

    using SPIClass = TMC_HAL::HW_port;
    using HardwareSerial = TMC_HAL::HW_port;

#elif defined(IDF_VER)

    #include <cstdint>
    #include <driver/gpio.h>
    #include <driver/spi_master.h>
    #include <driver/uart.h>
    #include <esp32/clk.h>
    #include <hal/cpu_ll.h>

    #define SW_CAPABLE_PLATFORM false

    namespace TMC_HAL {
        using PinDef = gpio_num_t;

        struct PinCache {
            explicit PinCache(const PinDef _pin) :
                pin(_pin)
                {}
            const PinDef pin;
            static constexpr bool LOW = 0;
            static constexpr bool HIGH = 1;
        };
    }

    using SPIClass = spi_device_handle_t;
    using HardwareSerial = uart_port_t;

    inline void delay(const uint16_t ms) {
        ets_delay_us( ms * 1000 );
    }

    namespace TMC_HAL {
        // Ensure CS pin timings requirements
        inline void delay_ns(unsigned int ns) {
            uint_fast16_t cycles = ((esp_clk_cpu_freq()>>16) * ns) / (1000000000UL>>16) + 1;

            uint32_t start = cpu_ll_get_cycle_count();

            while(cpu_ll_get_cycle_count() - start < cycles);
        }
    }

#elif defined(PICO_BOARD)

    #include <pico/stdlib.h>
    #include <hardware/spi.h>

    namespace TMC_HAL {
        using PinDef = uint;

        struct PinCache {
            explicit PinCache(const PinDef _pin) :
                pin(_pin)
                {}

            const PinDef pin;
            static constexpr bool LOW = 0;
            static constexpr bool HIGH = 1;
        };
    }

    using SPIClass = spi_inst_t * const;
    using HardwareSerial = uart_inst_t * const;

    // inline void delay(size_t ms) { /*wait_us(1000*ms);*/ }
    namespace TMC_HAL {
        inline void delay_ns(unsigned int ns) { /*wait_ns(ns);*/ }
    }

#elif defined(UNIT_TEST)

    #include "../examples/UnitTests/include/Mocks.h"

#endif

namespace TMC_HAL {

    class InputPin : PinCache {
    public:
        InputPin(const PinDef _pin);

        void setMode() const;

        bool read() const;

        operator bool() const {
            return read();
        }
    };

    class OutputPin : PinCache {
    public:
        OutputPin(const PinDef _pin);

        void setMode() const;

        void write(const bool state) const {
            state ? set() : reset();
        }
        void operator =(const bool state) const {
            write(state);
        }

        void set() const;
        void reset() const;
    };
}

#ifndef HIGH
    #define HIGH 1
#endif
#ifndef LOW
    #define LOW 0
#endif
#ifndef SW_CAPABLE_PLATFORM
    #define SW_CAPABLE_PLATFORM false
#endif

#ifdef UNIT_TEST
	#define TMC_WEAK_FUNCTION
#else
	#define TMC_WEAK_FUNCTION __attribute__((weak))
#endif

#if defined(F_CPU) && !defined(TARGET_LPC1768)

    namespace TMC_HAL {
        // Ensure CS pin timings requirements
        inline void delay_ns(unsigned int ns) {
            uint_fast16_t cycles = ((F_CPU>>16) * ns) / (1000000000UL>>14) + 1;

            while(--cycles) {
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
            }
        }
    }

#elif defined(TARGET_RP2040)

    #include "hardware/clocks.h"

    namespace TMC_HAL {
        inline void delay_ns(unsigned int ns) {
            const uint32_t F_CPU = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) * 1000;

            uint_fast16_t cycles = ((F_CPU>>16) * ns) / (1000000000UL>>14) + 1;

            while(--cycles) {
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
                asm("nop");
            }
        }
    }

#endif
