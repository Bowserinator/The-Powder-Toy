#ifndef CIRCUITS_FRAMEWORK_H
#define CIRCUITS_FRAMEWORK_H

#include "simulation/CoordStack.h"
#include "simulation/ElementCommon.h"
#include "circuit_core.h"
#include "values/values.h"

#include <vector>

class Circuits;

bool allow_conduction(int totype, int fromtype);
bool can_be_skeleton(int i, Simulation * sim);
bool can_be_node(int i, Simulation * sim);

coord_vec floodfill(Simulation *sim, int x, int y);
coord_vec coord_vec_to_skeleton(Simulation *sim, const coord_vec &floodfill, bool &has_chip);

// Type classifications
inline bool is_positive_terminal(int type)    { return get_flags(type) & POSITIVE_TERMINAL; }
inline bool is_negative_terminal(int type)    { return get_flags(type) & NEGATIVE_TERMINAL; }
inline bool is_terminal(int type)             { return is_positive_terminal(type) || is_negative_terminal(type); }
inline bool has_negative_resistance(int type) { return get_flags(type) & NEGATIVE_RESISTANCE; }
inline bool is_chip(int type)                 { return get_flags(type) & IS_CHIP; }
inline bool is_voltage_source(int type)       { return get_flags(type) & IS_VOLTAGE_SOURCE; }
inline bool is_integration_particle(int type) { return get_flags(type) & REQUIRES_INTEGRATION; }
inline bool is_dynamic_resistor(int type)     { return has_negative_resistance(type) || get_flags(type) & DYNAMIC_RESISTANCE; }
inline bool is_dynamic_particle(int type)     { return get_flags(type) & DYNAMIC_PARTICLE || is_chip(type); }
inline bool doesnt_heat_up(int type)          { return get_flags(type) & DONT_HEAT_UP; }
inline bool ignores_non_skeleton(int type)    { return get_flags(type) & IGNORE_NON_SKELETON; }

#endif
