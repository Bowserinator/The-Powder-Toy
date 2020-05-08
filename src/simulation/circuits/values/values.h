#ifndef CIRCUIT_VALUES_H
#define CIRCUIT_VALUES_H

#include <unordered_map>
#include "simulation/circuits/circuit_core.h"

#define POSITIVE_TERMINAL    0x01
#define NEGATIVE_TERMINAL    0x02
#define NEGATIVE_RESISTANCE  0x04
#define DYNAMIC_RESISTANCE   0x08  // Resistance can change per frame
#define DYNAMIC_PARTICLE     0x10  // Requires recalculation and constant updates
#define IS_CHIP              0x20
#define IS_VOLTAGE_SOURCE    0x40
#define REQUIRES_INTEGRATION 0x80
#define DONT_HEAT_UP         0x100 // Doesn't heat up with current

class CircuitData {
public:
    const Ohms resistance;
    const int flags;
    CircuitData(Ohms res=0.0, int flags=0) : resistance(res), flags(flags) {}
};

const std::unordered_map<ElementType, CircuitData> circuit_data({
    { PT_VOLT, CircuitData( REALLY_BIG_RESISTANCE, IS_VOLTAGE_SOURCE | DONT_HEAT_UP )}, // If wired improperly turns into big resistor
    { PT_GRND, CircuitData( SUPERCONDUCTING_RESISTANCE )},
    { PT_COPR, CircuitData( 1.72e-8, POSITIVE_TERMINAL )},
    { PT_PSCN, CircuitData( 30, POSITIVE_TERMINAL )},
    { PT_ZINC, CircuitData( 5.5e-8, NEGATIVE_TERMINAL )},
    { PT_NSCN, CircuitData( 30, NEGATIVE_TERMINAL )},

    { PT_NEON, CircuitData( 10e3, NEGATIVE_RESISTANCE )},  // Neon
    { PT_NBLE, CircuitData( 539, NEGATIVE_RESISTANCE )},   // NBLE = Helium
    { PT_HELM, CircuitData( 539, NEGATIVE_RESISTANCE )},
    { PT_PLSM, CircuitData( 539, NEGATIVE_RESISTANCE )},

    { PT_TRST, CircuitData( 0, DYNAMIC_RESISTANCE )},
    { PT_NTCT, CircuitData( 0, DYNAMIC_RESISTANCE | DYNAMIC_PARTICLE )},
    { PT_PTCT, CircuitData( 0, DYNAMIC_RESISTANCE | DYNAMIC_PARTICLE )},
    { PT_PQRT, CircuitData( 0, DYNAMIC_RESISTANCE )},
    { PT_QRTZ, CircuitData( 0, DYNAMIC_RESISTANCE )},
    { PT_MERC, CircuitData( 0, DYNAMIC_RESISTANCE )},
    { PT_CRBN, CircuitData( 0, DYNAMIC_RESISTANCE )},
    { PT_TIN,  CircuitData( 0, DYNAMIC_RESISTANCE )},
    { PT_RSTR, CircuitData( 0 )},

    { PT_SWCH, CircuitData( 1e-8, DYNAMIC_PARTICLE | DONT_HEAT_UP )},
    // Is a really big resistor at steady state
    { PT_CAPR, CircuitData( REALLY_BIG_RESISTANCE,
        DYNAMIC_PARTICLE | IS_VOLTAGE_SOURCE | REQUIRES_INTEGRATION | DONT_HEAT_UP )},
    // Is a wire at steady state
    { PT_INDC, CircuitData( 0, DYNAMIC_PARTICLE | REQUIRES_INTEGRATION )},

    { PT_LOGC, CircuitData( REALLY_BIG_RESISTANCE, IS_CHIP )},

    { PT_SICN, CircuitData( 30 )},
    { PT_METL, CircuitData( 1e-7 )}, // Metal is "steel"
    { PT_BMTL, CircuitData( 1e-7 )},
    { PT_BRMT, CircuitData( 1e-7 )},
    { PT_TRUS, CircuitData( 1e-7 )},
    { PT_INWR, CircuitData( 1e-7 )},
    { PT_BREC, CircuitData( 1e-7 )},
    { PT_NICH, CircuitData( 1e-7 )},
    { PT_TUNG, CircuitData( 5.6e-8 )},
    { PT_LEAD, CircuitData( 2.2e-8 )},
    { PT_DSTW, CircuitData( 1820000000.0 )},
    { PT_WATR, CircuitData( 1000 )},
    { PT_SLTW, CircuitData( 30 )},
    { PT_SWTR, CircuitData( 30 )},
    { PT_IOSL, CircuitData( 30 )},
    { PT_IRON, CircuitData( 9.7e-8 )},
    { PT_THRM, CircuitData( 9.7e-8 )},
    { PT_GOLD, CircuitData( 2.44e-8 )},
    { PT_TTAN, CircuitData( 4.2e-7 )},
    { PT_RBDM, CircuitData( 1.2e-7 )},
    { PT_LRBD, CircuitData( 1.2e-7 )},
    { PT_CESM, CircuitData( 2e-7 )},
    { PT_LCSM, CircuitData( 2e-7 )},
    { PT_INST, CircuitData( SUPERCONDUCTING_RESISTANCE )},
    { PT_BRAS, CircuitData( 0.6e-7 )},
    { PT_BSMH, CircuitData( 1.3e-6 )},
    { PT_ALMN, CircuitData( 2.65e-8 )},
    { PT_PLUT, CircuitData( 141.4e-8 )},
    { PT_URAN, CircuitData( 2.8e-7 )},
    { PT_POLO, CircuitData( 40e-8 )},
    { PT_THOR, CircuitData( 1.5e-7 )},
    { PT_PHSP, CircuitData( 1e-7 )},
    { PT_TMRM, CircuitData( 1e-7 )},
    { PT_IRDM, CircuitData( 4.71e-8 )},
    { PT_PTNM, CircuitData( 10.6e-8 )},
    { PT_BRNZ, CircuitData( 2.58e-7 )}
});

inline int get_flags(int type) {
    if (!circuit_data.count(type)) return 0;
    return circuit_data.at(type).flags;
}

#endif
