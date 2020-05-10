#ifndef CIRCUIT_CIRCUIT_CLASS
#define CIRCUIT_CIRCUIT_CLASS

#include "simulation/CoordStack.h"
#include "simulation/ElementCommon.h"
#include "simulation/circuits/circuit_core.h"
#include "simulation/circuits/framework.h"

#include <vector>
#include <unordered_map>

const std::vector<std::pair<int, int> > ADJACENT_PRIORITY({
    std::make_pair(-1, 0),
    std::make_pair(0, -1),
    std::make_pair(1, 0),
    std::make_pair(0, 1),
    std::make_pair(-1, 1),
    std::make_pair(-1, -1),
    std::make_pair(1, -1),
    std::make_pair(-1, 1)
});

class Branch;
class Chip;

class Circuit {
    friend Branch;
private:
    Simulation * sim;
    NodeId * node_skeleton_map = nullptr;
    char * immutable_node_map = nullptr; // 1 = directly adjacent, 2 = diagonally adjacent
    Circuit * copy = nullptr;                   // Copy of circuit to solve for divergence check

    bool requires_divergence_checking = false; // Steady state bounding for numeric integration (capacitors / inductors)
    bool computed_divergence = false;          // One time flag for creation
    bool recalc_next_frame = false;            // Flag for regeneration
    bool contains_dynamic = false;             // Contains dynamic components such as capacitors (update every frame)
    bool solution_computed = false;            // Already computed solution for non-dynamic systems
    
    NodeId highest_node_id = 0;
    int startx, starty;
    int recalc_cooldown_timer = 0;
    int circuit_size = 0;

    // Solution solving cache
    bool new_solution_exists = true;
    std::vector<double> prev_solution;

    // These can contain duplicate node-node id pairs, and follow same order, ie if connection map
    // for node 5 at index 2 is node 3, then the branch at node 5 index 2 is for node 3
    // Note: duplicate node-node pairs indicate multiple connections from 1 node to another
    // and represent different branches
    std::unordered_map<NodeId, std::vector<NodeId> > connection_map;      // Node ID: Ids that connect
    std::unordered_map<NodeId, std::vector<Branch *> > branch_map;        // Node ID: branches with start_node = id
    std::unordered_map<NodeId, std::vector<Branch *> > floating_branches; // Node ID, branches with start_node = id
    std::vector<Branch *> branch_cache;                               // Stores pointers so we can delete them later
    std::vector<ParticleId> global_rspk_ids;
    std::unordered_map<NodeId, Volts> constrained_nodes; // Reference for nodes = 0 V

    // Chip data
    std::vector<Chip> chips;

    void process_skeleton(coord_vec &skeleton);
    void mark_nodes(const coord_vec &skeleton, coord_vec &nodes);
    void trim_nodes(const coord_vec &nodes);
    void add_branches(const coord_vec &skeleton);

    void add_branch_from_skeleton(const coord_vec &skeleton, Pos start_node_pos, Pos start_offset_pos, NodeId start_node);
    void add_immutable_node(NodeId node_id, Pos position, bool is_diagonal_connection);
    void delete_node(Pos position);

    void delete_maps();
public:
    void generate();
    void solve(bool allow_recursion = true);
    void update_sim();
    
    void reset_effective_resistances();
    void flag_recalc(bool force=false);
    bool should_recalc() { return recalc_next_frame; }
    void reset();
    void debug();

    void update_chip_io(ParticleId chip_pid, ParticleId terminal_id, bool positive_terminal);

    size_t branch_cache_size() const { return branch_cache.size(); }
    const std::vector<int> &get_global_rspk_ids() const { return global_rspk_ids; }

    Circuit(int x, int y, Simulation *sim);
    Circuit(const Circuit &other);
    ~Circuit();
};

struct NodeAndIndex {
    NodeId node;
    size_t index;
};

struct SuperNode {
    NodeId node1;
    NodeId node2;
    double voltage;
};

#endif