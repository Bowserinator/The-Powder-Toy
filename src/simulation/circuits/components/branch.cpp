#include "simulation/circuits/components/branch.h"
#include "simulation/circuits/components/circuit.h"

#include "simulation/ElementCommon.h"
#include "simulation/circuits/circuit_core.h"
#include "simulation/circuits/resistance.h"

#include <iostream>


void Branch::print() {
    #ifdef DEBUG
        std::cout << "Branch from " << node1 << " -> " << node2 << " R: " << resistance << " V: " << voltage_gain << 
            "  I source: " << current_gain << "    I: " << current << " | diode: " << diode << " | " <<
            V1 << " to " << V2 << " | Node1, node2 id: " << node1_id << " -> " << node2_id << "\n";
    #endif
}

bool Branch::switches_on(Simulation * sim) {
    if (!recompute_switches) return switches_on_;
    switches_on_ = true;
    for (size_t j = 0; j < switches.size(); j++)
        if (sim->parts[switches[j]].life != 10) {
            switches_on_ = false;
            break;
        }
    recompute_switches = false;
    return switches_on_;
}

void Branch::set_special_type(bool is_capacitor, bool is_inductor, bool is_chip) {
    if (diode)             branch_type = DIODE;
    else if (is_capacitor) branch_type = CAPACITOR;
    else if (is_inductor)  branch_type = INDUCTOR;
    else if (is_chip)      branch_type = CHIP;
    else if (voltage_gain) branch_type = VOLTAGE_SOURCE;

    obeys_ohms_law_ = !(is_capacitor || is_inductor || diode || voltage_gain || is_chip);
}

void Branch::set_to_steady_state() {
    branch_type = RESISTOR;
    voltage_gain = 0.0;
    current_gain = 0.0;
    obeys_ohms_law_ = true;
}

void Branch::compute_dynamic_resistances(Simulation * sim, Circuit * c) {
    resistance = base_resistance;
    if (!switches_on(sim)) // Increase effective resistances of switches that are off
        resistance += REALLY_BIG_RESISTANCE * switches.size();
    for (auto id : dynamic_resistors)
        resistance += get_effective_resistance(sim->parts[id].type, sim->parts, id, sim);
}

void Branch::compute_dynamic_voltages(Simulation *sim, Circuit * c) {
    if (branch_type == CAPACITOR) {
        voltage_gain = 0.0;
        for (auto id : ids)
            voltage_gain += sim->parts[id].pavg[1];
    }
    else if (branch_type == CHIP) {
        float output_voltage = 0.0;
        if (ids.size())
            output_voltage = sim->parts[ids[0]].pavg[1];
        if (c->constrained_nodes.count(node1))
            c->constrained_nodes[node1] = output_voltage;
        else if (c->constrained_nodes.count(node2))
            c->constrained_nodes[node2] = output_voltage;
    }
}

void Branch::compute_dynamic_currents(Simulation * sim, Circuit * c) {
    if (branch_type == INDUCTOR && ids.size())
        current_gain = sim->parts[ids[0]].pavg[1];
}