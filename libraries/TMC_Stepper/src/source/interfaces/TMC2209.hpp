#pragma once

#include "../../TMCStepper.h"
#include "TMC2130.hpp"

namespace TMC2209_n {

    using namespace TMC2208_n;
    using TMC2130_n::TCOOLTHRS_t;
    using TMC2130_n::TCOOLTHRS_i;

    // 0x06 R: IOIN
    #pragma pack(push, 1)
    struct IOIN_t {
        constexpr static uint8_t address = 0x06;
        union {
            uint32_t sr;
            struct {
                bool enn : 1,
                    : 1,
                    ms1 : 1,
                    ms2 : 1,
                    diag : 1,
                    : 1,
                    pdn_uart : 1,
                    step : 1,
                    spread_en : 1,
                    dir : 1;
                uint16_t : 14;
                uint8_t version : 8;
            };
        };
    };
    #pragma pack(pop)

    template<typename TYPE>
    struct IOIN_i {
        uint32_t IOIN() {
            return static_cast<TYPE*>(this)->read(IOIN_t::address);
        }
        bool enn()          { return IOIN_t{ IOIN() }.enn;         }
        bool ms1()          { return IOIN_t{ IOIN() }.ms1;         }
        bool ms2()          { return IOIN_t{ IOIN() }.ms2;         }
        bool diag()         { return IOIN_t{ IOIN() }.diag;        }
        bool pdn_uart()     { return IOIN_t{ IOIN() }.pdn_uart;    }
        bool step()         { return IOIN_t{ IOIN() }.step;        }
        bool spread_en()    { return IOIN_t{ IOIN() }.spread_en;   }
        bool dir()          { return IOIN_t{ IOIN() }.dir;         }
        uint8_t version()   { return IOIN_t{ IOIN() }.version;     }
    };

    // 0x40 W: SGTHRS
    struct SGTHRS_t {
        constexpr static uint8_t address = 0x40;
        uint8_t sr : 8;
    };

    template<typename TYPE>
    struct SGTHRS_i {
        void SGTHRS(const uint8_t input) {
            r.sr = input;
            static_cast<TYPE*>(this)->write(r.address, r.sr);
        }
        uint8_t SGTHRS() const {
            return r.sr;
        }
    protected:
        SGTHRS_t r{};
    };

    // 0x41 R: SG_RESULT
    struct SG_RESULT_t {
        constexpr static uint8_t address = 0x41;
    };

    template<typename TYPE>
    struct SG_RESULT_i {
        uint16_t SG_RESULT() {
            return static_cast<TYPE*>(this)->read(SG_RESULT_t::address);
        }
    };

    // 0x42 W: COOLCONF
    #pragma pack(push, 1)
    struct COOLCONF_t {
        constexpr static uint8_t address = 0x42;
        union {
            uint16_t sr;
            struct {
                uint8_t semin : 4,
                        : 1,
                        seup : 2,
                        : 1,
                        semax : 4,
                        : 1,
                        sedn : 2;
                bool    seimin : 1;
            };
        };
    };
    #pragma pack(pop)

    template<typename TYPE>
    struct COOLCONF_i {
        uint16_t COOLCONF() const { return r.sr; }
        void COOLCONF(const uint16_t input) {
            r.sr = input;
            static_cast<TYPE*>(this)->write(r.address, r.sr);
        }

        void semin( const uint8_t B ) { r.semin = B;  COOLCONF(r.sr); }
        void seup(  const uint8_t B ) { r.seup = B;   COOLCONF(r.sr); }
        void semax( const uint8_t B ) { r.semax = B;  COOLCONF(r.sr); }
        void sedn(  const uint8_t B ) { r.sedn = B;   COOLCONF(r.sr); }
        void seimin(const bool    B ) { r.seimin = B; COOLCONF(r.sr); }

        uint8_t semin()  const { return r.semin;   }
        uint8_t seup()   const { return r.seup;    }
        uint8_t semax()  const { return r.semax;   }
        uint8_t sedn()   const { return r.sedn;    }
        bool    seimin() const { return r.seimin;  }

        protected:
            COOLCONF_t r{};
    };
}

namespace TMC2224_n {
    #pragma pack(push, 1)
    struct IOIN_t {
        constexpr static uint8_t address = 0x06;
        union {
            uint32_t sr;
            struct {
                bool  : 1,
                    pdn_uart : 1,
                    spread : 1,
                    dir : 1,
                    enn : 1,
                    step : 1,
                    ms1 : 1,
                    ms2 : 1,
                    sel_a : 1;
                uint16_t : 15;
                uint8_t version : 8;
            };
        };
    };
    #pragma pack(pop)

    template<typename TYPE>
    struct IOIN_i {
        uint32_t IOIN() {
            return static_cast<TYPE*>(this)->read(IOIN_t::address);
        }

        bool enn()          { return IOIN_t{ IOIN() }.enn;       }
        bool ms1()          { return IOIN_t{ IOIN() }.ms1;       }
        bool ms2()          { return IOIN_t{ IOIN() }.ms2;       }
        bool pdn_uart()     { return IOIN_t{ IOIN() }.pdn_uart;  }
        bool spread()       { return IOIN_t{ IOIN() }.spread;    }
        bool step()         { return IOIN_t{ IOIN() }.step;      }
        bool sel_a()        { return IOIN_t{ IOIN() }.sel_a;     }
        bool dir()          { return IOIN_t{ IOIN() }.dir;       }
        uint8_t version()   { return IOIN_t{ IOIN() }.version;   }
    };
}
