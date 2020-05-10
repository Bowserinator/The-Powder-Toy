#include "simulation/circuits/components/chip.h"
#include "simulation/circuits/circuit_core.h"
#include "simulation/circuits/components/circuit.h"

#include <iostream>

void Chip::add_input(ParticleId id, Volts voltage, Amps current, ElementType type) {
    if (input_particle_ids.find(id) != input_particle_ids.end())
        return;
    input_data.push_back(ChipData(voltage, current, type, id));
    input_particle_ids.insert(id);
}

void Chip::add_output(ParticleId id, NodeId node_id) {
    if (output_particle_ids.find(id) != output_particle_ids.end())
        return;
    if (node_id < CircuitParams::START_NODE_ID)
        return;
    output_nodes.push_back(node_id);
    output_particle_ids.insert(id);
}

void Chip::compute_output(std::unordered_map<NodeId, Volts> &constrained_nodes) {
    output_value = output_high;

    switch(chip_type) {
        // Logic
        case AND:
        case NAND:
            output_value = output_high;
            for (auto &d : input_data) {
                if (fabs(d.voltage) < MIN_CHIP_TRUE_VOLTAGE) {
                    output_value = 0.0;
                    break;
                }
            }
            if (chip_type == NAND)
                output_value = output_high - output_value;
            break;
        case OR:
        case NOR:
            output_value = 0.0;
            for (auto &d : input_data) {
                if (fabs(d.voltage) >= MIN_CHIP_TRUE_VOLTAGE) {
                    output_value = output_high;
                    break;
                }
            }
            if (chip_type == NOR)
                output_value = output_high - output_value;
            break;
        case NOT:
            output_value = input_data.size() ? (
                fabs(input_data[0].voltage) >= MIN_CHIP_TRUE_VOLTAGE ? 0.0 : output_high) : 0.0;
            break;
        case XOR:
        case XNOR: {
                output_value = 0.0;
                int true_count = 0;
                for (auto &d : input_data) {
                    if (fabs(d.voltage) >= MIN_CHIP_TRUE_VOLTAGE)
                        true_count++;
                    if (true_count > 1)
                        break;
                }
                output_value = true_count == 1 ? output_high : 0.0;
                if (chip_type == XNOR)
                    output_value = output_high - output_value;
            break;
        }

        // Op amps
        case BUFFER:
            output_value = input_data.size() ? input_data[0].voltage : 0.0;
            break;
    }

    for (NodeId id : output_nodes) {
        constrained_nodes[id] = output_value;
    }
}

void Chip::clear() {
    input_data.clear();
    output_nodes.clear();
    input_particle_ids.clear();
    output_particle_ids.clear();
}

void Chip::assign_values(ChipType type, Volts output) {
    chip_type = type;
    output_high = output;
}
