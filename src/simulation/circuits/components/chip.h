#ifndef CIRCUIT_CHIP_CLASS
#define CIRCUIT_CHIP_CLASS

#include "simulation/circuits/circuit_core.h"
#include "simulation/circuits/framework.h"

#include <vector>
#include <set>
#include <unordered_map>

enum ChipTypes {
    AND = 0,
    OR = 1,
    NOT = 2,
    XOR = 3,
    NAND = 4,
    NOR = 5,
    XNOR = 6,
    BUFFER = 7,
    OPAMP = 8
};

class ChipData {
public:
    Volts voltage;
    Amps current;
    ElementType ptype;
    ParticleId pid;

    ChipData(Volts voltage, Amps current, ElementType ptype, ParticleId pid) :
        voltage(voltage), current(current), ptype(ptype), pid(pid) {}
};

class Chip {
private:
    ChipType chip_type = 0;
    std::set<ParticleId> input_particle_ids,
                         output_particle_ids;
    std::vector<ChipData> input_data;
    std::vector<NodeId> output_nodes;

    Volts output_value = 0.0; // Output voltage, can vary
    Volts output_high = 5.0f; // Output when true
public:
    Chip() {};

    void assign_values(ChipType type, Volts output);
    void add_input(ParticleId id, Volts voltage, Amps current, ElementType type);
    void add_output(ParticleId id, NodeId node_id);
    void compute_output(std::unordered_map<NodeId, Volts> &constrained_nodes);
    void clear();
};

#endif
