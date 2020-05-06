#include "resistance.h"
#include "framework.h"
#include "circuit_core.h"
#include "values/values.h"

#include <exception>
#include <cmath>
#include <iostream>

// ParticleId i is unused but is reserved in case a particle has multiple states
// (only some of which are conductive) determined by a variable (ie tmp). Note that if
// a particle can easily transition between conductive and non-conductive states (ie SWCH)
// it should be marked as conductive, the above comment is for, ie variants of an element
// based on tmp that can only be set via console

bool valid_conductor(ElementType typ, Simulation *sim, ParticleId i) {
    if (i < 0 || !typ) return false;

    // Note: valid conductor only means if RSPK can exist on it, not if its currently capable of conducting
    // Ie, off SWCH should still return true because it has an ON state that can conduct
    return sim->elements[typ].Properties & PROP_CONDUCTS || circuit_data.count(typ);
}

/**
 * Use get_resistance or get_effective_resistance?
 * If your element updates its resistance every frame like a thermoresistor (or could, like a switch)
 * then use get_effective_resistance, otherwise use get_resistance, which updates only every couple of frames
 * 
 * If you set a resistance in effective_resistance, set the resistance in get_resistance to 0.0
 * Also dynamic resistance elements should be added to framework.cpp:is_dynamic_resistor
 */

double get_resistance(ElementType type, Particle *parts, ParticleId i, Simulation *sim) {
    if (type < 0 || type >= PT_NUM) // Should never happen, the throw below is just in case you need to debug
        throw "Error: Invalid particle type found in get_resistance in circuit simulation";

    // Insulators or ideal chips, set to really high value
    if (!valid_conductor(type, sim, i) || is_chip(type)) 
        return REALLY_BIG_RESISTANCE;

    // Dynamic (includes negative resistances) update every frame
    // so they have a 0 base resistance and a "dynamic" resistance
    // that's recalculated every frame that's added on
    if (is_dynamic_resistor(type))
        return 0.0;

    auto itr = circuit_data.find(type);
    if (itr != circuit_data.end())
        return itr->second.resistance;

    return DEFAULT_RESISTANCE;
}

// Not base resistance, but actual resistance it behaves as in the circuit
// (ie, a switch can have either really low or really high resistance depending on state)
// This updates every frame

double get_effective_resistance(ElementType type, Particle *parts, ParticleId i, Simulation *sim) {
    switch(type) {
        case PT_SWCH:
            return parts[i].life >= 4 ? circuit_data.at(type).resistance : REALLY_BIG_RESISTANCE;

        // Resistor stores resistance in pavg0
        case PT_RSTR:
            return parts[i].pavg[0];

        // Quartz
        case PT_QRTZ:
        case PT_PQRT:
            return parts[i].temp < 173.15f ? 50 : REALLY_BIG_RESISTANCE;

        // Superconductors
        case PT_MERC:
            return parts[i].temp < 4 ? SUPERCONDUCTING_RESISTANCE : 9.6e-7;
        case PT_CRBN: // Unrealistic critical temp, but matches behavior for SPRK
            return parts[i].temp < 100 ? SUPERCONDUCTING_RESISTANCE : 1e-5;
        case PT_TIN:
            return parts[i].temp < 3.72f ? SUPERCONDUCTING_RESISTANCE : 1.1e-7;
            
        // Semiconductors
        case PT_PTCT: // Resistance goes to 1e-7 above 100 C
            if (parts[i].temp <= 373.15f)
                return REALLY_BIG_RESISTANCE;
            return std::max(-(double)log((parts[i].temp - 373.15f) / 10.0f), 1e-7);
        case PT_NTCT: // Resistance goes to 1e-7 below 100 C
            if (parts[i].temp >= 373.15f)
                return REALLY_BIG_RESISTANCE;
            return std::max(-(double)log(-(parts[i].temp - 373.15f) / 10.0f), 1e-7);

        // Thermoresistor
        case PT_TRST:
            return std::max(1e-7, (double)(parts[i].pavg[0] + parts[i].temp * parts[i].pavg[1]));
    }

    // Negative resistance conductors
    if (has_negative_resistance(type)) {
        Ohms base_resistance = circuit_data.at(type).resistance;
        Amps current = 0.0;
        int r = sim->photons[(int)(parts[i].y + 0.5f)][(int)(parts[i].x + 0.5f)];

        if (r && TYP(r) == PT_RSPK)
            current = 1000 * fabs(parts[ID(r)].pavg[1]);
        return base_resistance / (1 + current);
    }

    return get_resistance(type, parts, i, sim);
}