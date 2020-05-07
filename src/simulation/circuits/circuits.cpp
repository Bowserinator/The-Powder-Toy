#include "simulation/circuits/circuits.h"
#include "simulation/ElementCommon.h"

#include <iomanip>
#include <iostream>

Circuit * circuit_map[NPART];
std::vector<Circuit *> all_circuits;

// Functions
void CIRCUITS::add_circuit(int x, int y, Simulation * sim) {
    int r = sim->photons[y][x];
    if (circuit_map[ID(r)])
        return; // Already part of a circuit

    Circuit * c = new Circuit(x, y, sim);
    all_circuits.push_back(c);
}

void CIRCUITS::delete_circuit(int i) {
    delete all_circuits[i];
    all_circuits.erase(all_circuits.begin() + i);
}

void CIRCUITS::update_all_circuits() {
    for (int i = all_circuits.size() - 1; i >= 0; i--) {
        Circuit * c = all_circuits[i];
        if (c->should_recalc()) {
            c->reset();
            c->generate();
        }
        c->reset_effective_resistances();
        c->solve();
        c->update_sim();
    }
}

void CIRCUITS::clear_circuits() {
    for (auto c : all_circuits)
        delete c;
    all_circuits.clear();
    std::fill(&circuit_map[0], &circuit_map[NPART], nullptr);
}
