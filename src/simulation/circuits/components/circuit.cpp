#include "simulation/circuits/components/circuit.h"
#include "simulation/circuits/components/branch.h"
#include "simulation/circuits/components/chip.h"

#include "simulation/ElementCommon.h"
#include "simulation/circuits/circuit_core.h"
#include "simulation/circuits/circuits.h"
#include "simulation/circuits/resistance.h"
#include "simulation/circuits/util.h"

#include "eigen/Core"
#include "eigen/Dense"

#include <iostream>
#include <cmath>

#define YX(y, x) ((y) * XRES + (x))

void Circuit::add_immutable_node(NodeId node_id, Pos position, bool is_diagonal_connection) {
    node_skeleton_map[YX(position.y, position.x)] = node_id;
    immutable_node_map[YX(position.y, position.x)] = is_diagonal_connection ?
        CircuitParams::DIAGONALLY_ADJACENT : CircuitParams::DIRECTLY_ADJACENT;
}

void Circuit::delete_node(Pos position) {
    immutable_node_map[YX(position.y, position.x)] = CircuitParams::NOSKELETON;
    node_skeleton_map[YX(position.y, position.x)] = CircuitParams::SKELETON;
}

/**
 * Pre-node generation processing for a circuit skeleton. Does the following:
 * 1. Reset node_skeleton_map, immutable_node_map
 * 2. Update global reference in circuit_map
 * 3. Add RSPK ids to circuit
 * 4. Set circuit RSPK tmp to 1 if on skeleton, else 0
 * 5. Set node_skeleton_map to 1 in parts of the circuit that are the skeleton
 * 
 * Note that the skeleton coord_vec is only actually a skeleton when this is complete running
 * @param coord_vec skeleton - Input floodfill data that's edited to be a skeleton
 */
void Circuit::process_skeleton(coord_vec &skeleton) {
    delete_maps();
    node_skeleton_map = new NodeId[XRES * YRES];
    immutable_node_map = new char[XRES * YRES];
    std::fill(node_skeleton_map, node_skeleton_map + XRES * YRES, CircuitParams::NOSKELETON);
    std::fill(immutable_node_map, immutable_node_map + XRES * YRES, CircuitParams::NOSKELETON);

    for (auto &pos : skeleton) {
        // It's safe to assume the id exists because
        // floodfill creates missing RSPK
        int id = ID(sim->photons[pos.y][pos.x]);
        sim->parts[id].tmp = CircuitParams::NOSKELETON; // Clear for setting later after nodes are assigned
        circuit_map[id] = this;
        global_rspk_ids.push_back(id);
    }

    skeleton = coord_vec_to_skeleton(sim, skeleton);
    for (auto &pos : skeleton) {
        node_skeleton_map[YX(pos.y, pos.x)] = CircuitParams::SKELETON;
        sim->parts[ID(sim->photons[pos.y][pos.x])].tmp = CircuitParams::SKELETON;
    }
}

/**
 * Preliminary node marking, can cause duplicates that will be later trimmed.
 * 
 * @param coord_vec skeleton  Circuit skeleton
 * @param coord_vec nodes     Cleared on function call. Locations of nodes will be written to this.
 */
void Circuit::mark_nodes(const coord_vec &skeleton, coord_vec &nodes) {
    NodeId node_id = CircuitParams::START_NODE_ID;
    nodes.clear();

    for (const auto &pos : skeleton) {
        int count = 0,
            x = pos.x,
            y = pos.y;
        int type = TYP(sim->pmap[y][x]);

        if (node_skeleton_map[YX(y, x)] > CircuitParams::SKELETON) // Already marked as node
            continue;

        /* Special Node Type 1
         * These elements have the node on the element itself
         * (For instance, ground). The node can only be assigned if it's touching
         * another conductor that's not itself to avoid excessive useless nodes */

        if (type == PT_GRND) {
            for (auto &rxy : ADJACENT_PRIORITY) {
                int rx = rxy.first, ry = rxy.second;
                if (node_skeleton_map[YX(y + ry, x + rx)] && TYP(sim->pmap[y + ry][x + rx]) != type) {
                    add_immutable_node(node_id, pos, false);
                    nodes.push_back(pos);
                    node_id++;
                    goto end_node_type1;
                }
            }
            end_node_type1:;
        }

        /* Special Node Type 2
         * These elements DO NOT need terminals, ALL particles of a different type that
         * border the element will be a node (for instance, an inductor): -H- will have the wires
         * (-) around it marked as nodes */

        else if (type == PT_INDC) {
            for (auto &rxy : ADJACENT_PRIORITY) {
                int rx = rxy.first, ry = rxy.second;
                if (node_skeleton_map[YX(y + ry, x + rx)] && TYP(sim->pmap[y + ry][x + rx]) != type) {
                    Pos npos(x + rx, y + ry);
                    add_immutable_node(node_id, npos, rx && ry);
                    nodes.push_back(npos);
                    node_id++;
                }
            }
        }

        /* Special Node Type 3
         * Elements with polarity need terminal type elements. The node will be on the terminal element, ie
         *      PSCN - VOLT => NODE - VOLT
         * There is also special handling code for diodes that does a check for change from PSCN to NSCN
         * and vice versa */

        else if (is_terminal(type)) {
            bool t_is_silicon = type == PT_PSCN || type == PT_NSCN;
            int other_silicon_type = type == PT_PSCN ? PT_NSCN : PT_PSCN;

            for (auto &rxy : ADJACENT_PRIORITY) {
                int rx = rxy.first, ry = rxy.second;
                int t = TYP(sim->pmap[y + ry][x + rx]);
                if (is_voltage_source(t) || is_chip(t) || (t_is_silicon && t == other_silicon_type)) {
                    add_immutable_node(node_id, pos, rx && ry);
                    nodes.push_back(pos);
                    node_id++;
                    goto end_node_type3;
                }
            }
            end_node_type3:;
        }

        // Can't be a non-special node (special nodes are exempt from this check)
        // This category mostly includes gases / liquids which disperse lot and create
        // a lot of unnecessary node calculations
        if (!can_be_node(ID(sim->pmap[y][x]), sim))
            continue;

        /* Count surrounding disjoint connections, if > 2 then it must be a junction, ie:
         * #YY
         *   X##
         *   #
         * The pixel marked x has 3 connections going into it, even though it has 4 surrounding
         * pixels, as the surrounding pixels marked with Y are touching, so are only counted once.
         * By convention, the code below ignores directly adjacent pixels (up, down, left, right) if a diagonal
         * touching that pixel is already filled; this allows for more diagonal connections to be considered. */

        for (int rx = -1; rx <= 1; rx++)
        for (int ry = -1; ry <= 1; ry++)
            if ((rx || ry) && node_skeleton_map[YX(y + ry, x + rx)]) {
                if (!rx && ry && (node_skeleton_map[YX(y + ry, x - 1)] || node_skeleton_map[YX(y + ry, x + 1)]))
                    continue;
                if (rx && !ry && (node_skeleton_map[YX(y + 1, x + rx)] || node_skeleton_map[YX(y - 1, x + rx)]))
                    continue;
                count++;
            }
        if (count > 2) {
            node_skeleton_map[YX(y, x)] = node_id;
            nodes.push_back(pos);
            node_id++;
        }
    }
}

/* Trim adjacent nodes, nodes often become grouped in arrangments like
 *    N
 *   NNN
 *    N
 * In which case we take only the center (above, the center node)
 * Also reassigns node ids as it deletes nodes, leaving empty gaps
 * in ids */
void Circuit::trim_nodes(const coord_vec &nodes) {
    NodeId new_node_id = 2;
    bool visited[YRES][XRES];
    std::fill(&visited[0][0], &visited[YRES][0], 0);

    for (const auto &pos : nodes) {
        if (node_skeleton_map[YX(pos.y, pos.x)] <= CircuitParams::SKELETON || visited[pos.y][pos.x]) // Already cleared, skip
            continue;
        if (immutable_node_map[YX(pos.y, pos.x)]) { // Node is not allowed to be condensed
            int type = 0;
            if (immutable_node_map[YX(pos.y, pos.x)] == CircuitParams::DIAGONALLY_ADJACENT) {
                for (int rx = -1; rx <= 1; ++rx)
                for (int ry = -1; ry <= 1; ++ry)
                    // Exception: if diagonal node is touching non-adjacent immutable node
                    // it can be deleted (redundant node)
                    if ((rx || ry) && immutable_node_map[YX(pos.y + ry, pos.x + rx)] == CircuitParams::DIRECTLY_ADJACENT) {
                        delete_node(pos);
                        goto end;
                    }
            }

            // Ground is constrained to 0
            type = TYP(sim->pmap[pos.y][pos.x]);
            if (type == PT_GRND)
                constrained_nodes[new_node_id] = 0.0;

            node_skeleton_map[YX(pos.y, pos.x)] = new_node_id;
            visited[pos.y][pos.x] = true;
            new_node_id++;

            end:;
            continue;
        }

        int count = 0, x, y;
        float avgx = 0.0f, avgy = 0.0f;
        bool x_all_same = true, y_all_same = true;

        CoordStack coords;
        coord_vec node_cluster;
        coords.push(pos.x, pos.y);

        while (coords.getSize()) {
            coords.pop(x, y);

            // Remove all nodes (except immutable ones) for now, reassign later
            if (!immutable_node_map[YX(y, x)])
                node_skeleton_map[YX(y, x)] = CircuitParams::SKELETON;

            node_cluster.push_back(Pos(x, y));
            avgx += x, avgy += y, count++;

            if (node_cluster.back().x != node_cluster[0].x)
                x_all_same = false;
            if (node_cluster.back().y != node_cluster[0].y)
                y_all_same = false;

            for (int rx = -1; rx <= 1; ++rx)
            for (int ry = -1; ry <= 1; ++ry)
                if ((rx || ry) &&
                        node_skeleton_map[YX(y + ry, x + rx)] > CircuitParams::SKELETON &&
                        !immutable_node_map[YX(y + ry, x + rx)]) {
                    coords.push(x + rx, y + ry);
                    node_skeleton_map[YX(y + ry, x + rx)] = CircuitParams::SKELETON;
                }
        }

        x = (int)std::round(avgx / count), y = (int)std::round(avgy / count);

        /* Check: special case where 3 repeated nodes lie in the same line
         * Instead of averaging to center, we trim the center one. For example,
         * NNN should be trimmed as N-N instead of -N- as averaging would */

        if (node_cluster.size() == 3 && (x_all_same || y_all_same)) {
            for (auto p : node_cluster) {
                if (p.x == x && p.y == y) // Skip center instead
                    continue;
                node_skeleton_map[YX(p.y, p.x)] = new_node_id;
                visited[p.y][p.x] = true;
                new_node_id++;
            }
        }

        /* A node in a T-shape like this:
         * X       The averaging code produces the wrong result and selects
         * NNXXXX  the right node, which can create a branch with 2 outputs!
         * X       Since a branch can't have 2 outputs it would create weird behavior */
        
        /* Large clusters (>5, which is size of star:)
         *  N    Might indicate a mesh of 1px branches, in which case
         * NNN   we only consider directly adjacent counts
         *  N */

        /* The code to solve both is the same */

        else if ((node_cluster.size() > 5) ||
                (node_cluster.size() == 2 && (x_all_same || y_all_same))) {
            for (auto p : node_cluster) {
                int x = p.x, y = p.y;
                int count = 0;

                for (int rx = -1; rx <= 1; ++rx)
                for (int ry = -1; ry <= 1; ++ry) // Count directly adjacent nodes
                    if ((rx || ry) && (!rx || !ry) && node_skeleton_map[YX(y + ry, x + rx)])
                        count++;
                if (count > 2) {
                    node_skeleton_map[YX(p.y, p.x)] = new_node_id;
                    visited[p.y][p.x] = true;
                    new_node_id++;
                }
            }
        }

        /* No need to floodfill, assign new ID */
        else if (node_skeleton_map[YX(y, x)] == 1) {
            node_skeleton_map[YX(y, x)] = new_node_id;
            visited[y][x] = true;
            new_node_id++;
        }

        /* Re-add closest node to average in the cluster */
        else {
            float closest = -1.0f, distance;
            auto closest_p = node_cluster[0];
            for (auto p : node_cluster) {
                distance = abs(x - p.x) + fabs(y - p.y);
                if (closest < 0 || distance < closest)
                    closest = distance, closest_p = p;
            }
            node_skeleton_map[YX(closest_p.y, closest_p.x)] = new_node_id;
            visited[closest_p.y][closest_p.x] = true;
            new_node_id++;
        }
    }

    // Last used node_id is new_node_id - 1 because new_node_id++ after each assignment
    highest_node_id = new_node_id - 1;
}

/**
 * Starting from a point next to a node (the node is at start_node_pos, the start position is start_offset_pos
 * which should be adjacent to the start_node_pos), floodfill a branch and add self to circuit
 */
void Circuit::add_branch_from_skeleton(const coord_vec &skeleton, Pos start_node_pos, Pos start_offset_pos, NodeId start_node) {
    if (!node_skeleton_map[YX(start_offset_pos.y, start_offset_pos.x)] || (start_offset_pos == start_node_pos))
        return;

    int x = start_offset_pos.x,
        y = start_offset_pos.y,
        sx = start_node_pos.x,
        sy = start_node_pos.y;
    int r, rt,
        px = sx, py = sy, ox = x, oy = y;

    BranchConstructionData data;
    NodeId end_node = CircuitParams::NOSKELETON;

    double current_voltage = 0.0;
    int source_count = 0;
    char current_polarity = CircuitParams::NEUTRAL_POLARITY; // Changed by terminals

    // Initial polarity check
    if (is_positive_terminal(TYP(sim->pmap[sy][sx])))
        current_polarity = CircuitParams::POSITIVE_POLARITY;
    else if (is_negative_terminal(TYP(sim->pmap[sy][sx])))
        current_polarity = CircuitParams::NEGATIVE_POLARITY;

    // Keep flood filling until we find another node
    while (true) {
        r = sim->pmap[y][x];
        if (!r) {
            // This should never happen because we should only be traversing
            // elements on the skeleton, which must contain a conductor. If this
            // happens, this means the skeleton id map has been desynced with
            // the actual particle map (ie, a particle was deleted but circuit
            // still thinks it exists). We'll ignore this for the production build
            // as the circuit *should* trigger a re-construction
            #ifndef DEBUG
                return;
            #endif
            throw CircuitParticleMapDesync();
        }
        rt = TYP(r);


        if (is_dynamic_particle(rt))
            this->contains_dynamic = true;
        if (is_integration_particle(rt))
            this->requires_divergence_checking = true;


        // Polarity handling for voltage drops
        // Handle voltage drops. Voltage across a blob of a voltage source
        // is the average of the voltage of each pixel (on the skeleton only)

        // Negative -> Positive is positive voltage:
        // -  ----.|.| ---- + going left to right is a voltage gain (positive)
        
        // If transitioning directly from PSCN to NSCN the branch is a diode

        if (is_positive_terminal(rt)) {
            if (current_polarity == CircuitParams::NEGATIVE_POLARITY)
                data.voltage_gain += current_voltage / std::max(1, source_count);
            if (rt == PT_PSCN && TYP(sim->pmap[py][px]) == PT_NSCN) // NSCN -> PSCN
                data.diode_type = CircuitParams::NEGATIVE_POLARITY;

            current_voltage = 0.0f;
            source_count = 0;
            current_polarity = CircuitParams::POSITIVE_POLARITY;
        }
        else if (is_negative_terminal(rt)) {
            // If changing this behavior also change in branch/compute_dynamic_voltages
            if (current_polarity == CircuitParams::POSITIVE_POLARITY)
                data.voltage_gain += current_voltage / std::max(1, source_count);
            if (TYP(r) == PT_NSCN && TYP(sim->pmap[py][px]) == PT_PSCN) // PSCN -> NSCN
                data.diode_type = CircuitParams::POSITIVE_POLARITY;

            current_voltage = 0.0f;
            current_polarity = CircuitParams::NEGATIVE_POLARITY;
            source_count = 0;
        }
        // We left the voltage source without encountering the opposite terminal
        // Examples: PSCN - VOLT - PSCN,  PSCN - VOLT - METL
        else if (!is_voltage_source(rt))
            current_polarity = 0, current_voltage = 0.0f, source_count = 0;


        // Found an end node
        if (node_skeleton_map[YX(y, x)] > CircuitParams::SKELETON && node_skeleton_map[YX(y, x)] != start_node) {
            end_node = node_skeleton_map[YX(y, x)];
            data.total_resistance += get_resistance(TYP(r), sim->parts, ID(r), sim);
            r = sim->pmap[sy][sx];
            data.total_resistance += get_resistance(TYP(r), sim->parts, ID(r), sim);

            // Make sure that points around a node are never deleted as points around nodes
            // might be part of 2 seperate branches, so if one is deleted then an entire branch
            // could be missed
            if (node_skeleton_map[YX(py, px)] == CircuitParams::NOSKELETON)
                node_skeleton_map[YX(py, px)] = CircuitParams::SKELETON;
            break;
        }
        
        if (node_skeleton_map[YX(y, x)] == CircuitParams::SKELETON) { // Non-node
            // Delete skeleton so we don't traverse over this again for
            // another branch (resulting in duplicate branches)
            node_skeleton_map[YX(y, x)] = CircuitParams::NOSKELETON;
            data.total_resistance += get_resistance(TYP(r), sim->parts, ID(r), sim);
            data.ids.push_back(ID(r));
            data.rspk_ids.push_back(ID(sim->photons[y][x]));

            // Switches have 1 set "off state" resistance (a line of OFF SWCH has the
            // same resistance as a single pixel of OFF SWCH) so they're considered
            // seperate from a dynamic resistor
            if (rt == PT_SWCH)
                data.switches.push_back(ID(r));

            // Current sources go here (currently inductor is only one)
            // Inductor has no polarity checks so we just set current gain to
            // whatever the current inductor is
            else if (rt == PT_INDC)
                data.current_gain = sim->parts[ID(r)].pavg[1];
            else if (is_dynamic_resistor(TYP(r)))
                data.dynamic_resistors.push_back(ID(r));
            else if (is_voltage_source(TYP(r))) {
                if (TYP(r) == PT_VOLT) // Pavg0 is voltage
                    current_voltage -= sim->parts[ID(r)].pavg[0] * current_polarity;
                else if (TYP(r) == PT_CAPR) // Pavg1 is "effective" voltage
                    current_voltage -= sim->parts[ID(r)].pavg[1] * current_polarity;
                source_count++;
            }
        }

        bool found_next = false; // Is there a new node to connect to?
        px = x, py = y;

        // If ignore_startnode_distance = 0, the new node must not be within 1 px of the startnode (on any side)
        // this is to avoid floodfill loops, where ie, the branch just makes a circle around the start node
        // blocking off any other branches. However, sometimes a connection must make a connection somewhere
        // within 1px, so if all searches fail the constraint is relaxed for a 2nd pass
        for (size_t ignore_startnode_distance = 0; ignore_startnode_distance <= 1; ignore_startnode_distance++) {
            // Check directly adjacent nodes (Highest priority)
            for (int rx = -1; rx <= 1; rx++)
            for (int ry = -1; ry <= 1; ry++)
                if ((rx == 0 || ry == 0) && (rx || ry) &&
                    node_skeleton_map[YX(y + ry, x + rx)] > CircuitParams::SKELETON &&
                    node_skeleton_map[YX(y + ry, x + rx)] != start_node)
                {
                    x += rx, y += ry, found_next = true;
                    goto end;
                }
            // Check directly adjacent
            for (int rx = -1; rx <= 1; rx++)
            for (int ry = -1; ry <= 1; ry++)
                if ((rx == 0 || ry == 0) && (rx || ry) &&
                    node_skeleton_map[YX(y + ry, x + rx)] &&
                    node_skeleton_map[YX(y + ry, x + rx)] != start_node &&
                    (ignore_startnode_distance || abs(x + rx - sx) > 1 || abs(y + ry - sy) > 1))
                {
                    x += rx, y += ry, found_next = true;
                    goto end;
                }
            // Check for nodes (3rd highest priority)
            for (int rx = -1; rx <= 1; ++rx)
            for (int ry = -1; ry <= 1; ++ry)
                if ((rx || ry) && node_skeleton_map[YX(y + ry, x + rx)] != start_node &&
                    node_skeleton_map[YX(y + ry, x + rx)] > CircuitParams::SKELETON &&
                    (ignore_startnode_distance || abs(x + rx - sx) > 1 || abs(y + ry - sy) > 1))
                {
                    x += rx, y += ry, found_next = true;
                    goto end;
                }
            // Check non-adjacent
            for (int rx = -1; rx <= 1; ++rx)
            for (int ry = -1; ry <= 1; ++ry)
                if ((rx && ry) && node_skeleton_map[YX(y + ry, x + rx)] &&
                    node_skeleton_map[YX(y + ry, x + rx)] != start_node &&
                    (ignore_startnode_distance || abs(x + rx - sx) > 1 || abs(y + ry - sy) > 1))
                {
                    x += rx, y += ry, found_next = true;
                    goto end;
                }
        }
        end:;
        if (!found_next)
            break; // No end node found, terminate
    }

    data.node1_id = ID(sim->photons[sy][sx]);
    data.node2_id = ID(sim->photons[y][x]);

    if (start_node > end_node) { // Convention: smaller node goes first
        std::swap(start_node, end_node);
        std::swap(data.node1_id, data.node2_id);
        std::reverse(data.rspk_ids.begin(), data.rspk_ids.end());
        data.voltage_gain *= -1;
    }

    // Make sure that points around a node are never deleted as points around nodes
    // might be part of 2 seperate branches, so if one is deleted then an entire branch
    // could be missed
    if (node_skeleton_map[YX(oy, ox)] == CircuitParams::NOSKELETON)
        node_skeleton_map[YX(oy, ox)] = CircuitParams::SKELETON;

    if (start_node > CircuitParams::NOSKELETON) { // Valid connection actually exists
        Branch *b = new Branch(start_node, end_node, data);

        /**
         * Special case where branch ends at a node directly adjacent to start node, because branches
         * search diagonals this might result in duplicate branches. Same case applies for branches of length up to 2
         * because end and start are overwritten
         */
        if (b->ids.size() <= 2) {
            // Search backwards as chance of match is higher at ends due to searching near
            // physical proximity. If a duplicate is found ignore
            for (int j = branch_map[start_node].size() - 1; j >= 0; j--) {
                if (branch_map[start_node][j]->node1 == std::min(start_node, end_node) &&
                    branch_map[start_node][j]->node2 == std::max(start_node, end_node) &&
                    (branch_map[start_node][j]->ids.size() <= 2 ||
                     branch_map[start_node][j]->ids == b->ids))
                    return;
            }
        }

        // ids does not include end nodes, so branches of length 2 have 0 id size
        b->set_special_type(b->ids.size() && sim->parts[b->ids[0]].type == PT_CAPR,
                          b->ids.size() && sim->parts[b->ids[0]].type == PT_INDC,
                          b->ids.size() && is_chip(sim->parts[b->ids[0]].type));

        connection_map[start_node].push_back(end_node);
        connection_map[end_node].push_back(start_node);

        branch_cache.push_back(b);
        branch_map[start_node].push_back(b);
        branch_map[end_node].push_back(b);
    }
    // Floating branch (> 1 px size)
    else if (data.rspk_ids.size() > 1) {
        Branch *b = new Branch(CircuitParams::NOSKELETON, end_node, data);
        floating_branches[end_node].push_back(b);
        branch_cache.push_back(b);
    }
    // 1 px floating branch may be part of node, reset
    else if (!immutable_node_map[YX(y, x)]) {
        node_skeleton_map[YX(y, x)] = CircuitParams::SKELETON;
    }
}

/**
 * Add all the branches
 * @param coord_vec skeleton Skeleton coordinate list
 */
void Circuit::add_branches(const coord_vec &skeleton) {
    for (auto &pos : skeleton) {
        int x = pos.x, y = pos.y;

        if (node_skeleton_map[YX(y, x)] > CircuitParams::SKELETON) {
            sim->parts[ID(sim->photons[y][x])].tmp = node_skeleton_map[YX(y, x)];

            bool adjacent_node = false;
            for (auto &rxy : ADJACENT_PRIORITY) {
                int rx = rxy.first, ry = rxy.second;
                // Since directly adjacent takes priority this check will run before the next
                // else if statement
                if ((rx == 0 || ry == 0) && node_skeleton_map[YX(y + ry, x + rx)] > CircuitParams::SKELETON)
                    adjacent_node = true;

                // Don't start branches on diagonally adjacent nodes if there's a directly adjacent
                // node touching it
                else if ((rx && ry) && !adjacent_node &&
                        (node_skeleton_map[YX(y + ry, x)] > CircuitParams::SKELETON ||
                         node_skeleton_map[YX(y, x + rx)] > CircuitParams::SKELETON))
                    continue;

                // It's possible for more than 2 paths to share a pixel, so we iterate twice
                // to make sure we get all paths. The efficency loss is small because if there is
                // only 1 path, all the allowed traversals will be removed in the 1st pass, causing the
                // branch fill to terminate early.

                // (I'm not sure if there can be 3 paths, seems unlikely given the skeletonization algorithim)
                add_branch_from_skeleton(skeleton, Pos(x, y), Pos(x + rx, y + ry), node_skeleton_map[YX(y, x)]);
                add_branch_from_skeleton(skeleton, Pos(x, y), Pos(x + rx, y + ry), node_skeleton_map[YX(y, x)]);
            }
        }
    }
}

/**
 * Internal method to map particles and convert into a circuit
 * mesh, branches, nodes, etc...
 */
void Circuit::generate() {
    coord_vec skeleton = floodfill(sim, startx, starty);
    coord_vec nodes;
    circuit_size = skeleton.size();
    
    process_skeleton(skeleton);
    mark_nodes(skeleton, nodes);
    trim_nodes(nodes);
    add_branches(skeleton);

    delete_maps(); // No longer needed, deallocate to save RAM (~16 MB per circuit)
}



/**
 * Solve the circuit using nodal analysis
 * 
 * Note that this should always be called with the default argument
 * allow_recursion=true (allow_recursion is used to not infinitely solve() when
 * resolving the circuits for diodes / other special components)
 */
void Circuit::solve(bool allow_recursion) {
    if (recalc_cooldown_timer > 0)
        recalc_cooldown_timer--;

    size_t size = highest_node_id - CircuitParams::START_NODE_ID + 1;
    if (!connection_map.size())
        return;

    // Don't need to solve: circuit is non-dynamic and we already solved it
    // Force recalc is for when something changes that doesn't delete / add particles,
    // (like a console command)
    if (!contains_dynamic && solution_computed && allow_recursion &&
            sim->timer % FORCE_RECALC_EVERY_N_FRAMES != 0)
        return;

    // Components that do numeric integration like inductors / capacitors will need a copy
    // of the Circuit, but with capacitors replaced with open circuits and inductors with shorts
    // to simulate steady state conditions
    bool check_divergence = requires_divergence_checking &&
        (!computed_divergence || sim->timer % INTEGRATION_RECALC_EVERY_N_FRAMES == 0);
    if (check_divergence) {
        if (copy) delete copy;
        copy = new Circuit(*this);
        copy->requires_divergence_checking = false;
    }

    // Chip pre-solving to set constrained nodes, do this once
    if (allow_recursion) {
        for (auto &c : chips)
            c.compute_output(constrained_nodes);
    }

    // Set true for now, if no change set to false later
    new_solution_exists = true;

    // Additional special case solvers
    std::vector<NodeAndIndex> diode_branches;
    std::vector<NodeAndIndex> numeric_integration;
    std::vector<SuperNode> supernodes;

    // Solve Ax = b
    Eigen::MatrixXd A(size, size);
    Eigen::VectorXd b(size), x(size);
    int j = 0;

    for (size_t row = 0; row < size; row++) {
        bool is_constrained;
        auto node_id = connection_map.find(row + CircuitParams::START_NODE_ID);
        double * matrix_row = new double[size + 1];
        std::fill(&matrix_row[0], &matrix_row[size + 1], 0);

        // Due to limitations of node finding sometimes false nodes appear, nodes that don't connect
        // to any other nodes at all. We just assign them to ground since they don't do anything
        if (node_id == connection_map.end()) {
            matrix_row[row] = 1;
            goto assign_row;
        }

        is_constrained = constrained_nodes.count(node_id->first);

        for (size_t i = 0; i < node_id->second.size(); i++) {
            Branch * b = branch_map[node_id->first][i];

            // Verify diodes and switches
            if (b->is_diode())
                diode_branches.push_back(NodeAndIndex{node_id->first, i});

            b->compute_dynamic_resistances(sim, this);
            b->compute_dynamic_currents(sim, this);
            b->compute_dynamic_voltages(sim, this);

            if (check_divergence && b->requires_numeric_integration(sim)) {
                numeric_integration.push_back(NodeAndIndex{node_id->first, i});
                copy->branch_map[node_id->first][i]->set_to_steady_state();
            }

            if (!is_constrained) {
                /* Deal with supernodes later */
                if (b->is_voltage_source()) {
                    if (node_id->first < node_id->second[i]) // Avoid duplicate supernodes, only take 1 of them
                        supernodes.push_back(SuperNode{node_id->first, node_id->second[i], b->voltage_gain});
                }
                /* Instead of doing I = (V2 - V1) / R if a branch has a current source
                 * add / subtract current value from end */
                else if (b->is_inductor()) {
                    matrix_row[size] = b->current_gain * (node_id->first < node_id->second[i] ? 1 : -1);
                }
                /** Sum of all (N2 - N1) / R = 0 (Ignore resistances across voltage sources, dealt with
                 *  when solving supernodes) */
                else if (b->resistance) {
                    matrix_row[node_id->first - CircuitParams::START_NODE_ID] -= 1.0f / b->resistance;
                    matrix_row[node_id->second[i] - CircuitParams::START_NODE_ID] += 1.0f / b->resistance;
                }
            }
            else if (is_constrained) {
                matrix_row[node_id->first - CircuitParams::START_NODE_ID] = 1;
                matrix_row[size] = constrained_nodes[node_id->first];
                goto assign_row;
            }
        }

        assign_row:;
        for (size_t i = 0; i < size; i++)
            A(j, i) = matrix_row[i];
        b[j] = matrix_row[size];
        delete[] matrix_row;
        j++;
    }

    /* Handle supernodes */
    for (size_t j = 0; j < supernodes.size(); j++) {
        int row1 = supernodes[j].node1 - CircuitParams::START_NODE_ID,
            row2 = supernodes[j].node2 - CircuitParams::START_NODE_ID;

        A.row(row1) += A.row(row2); // KCL over both end nodes
        for (size_t k = 0; k < size; k++) { // KVL equation
            if ((int)k == row1)
                A(row2, k) = -1;
            else if ((int)k == row2)
                A(row2, k) = 1;
            else
                A(row2, k) = 0;
        }
        b[row2] = supernodes[j].voltage;
    }

    x = A.partialPivLu().solve(b); // colPivHouseholderQr is more accurate but slower
    if (isnanl(x[0])) // Failed to solve system, set all voltages to 0
        for (int i = 0; i < x.size(); i++)
            x[i] = 0;

    // std::cout << A << "\n";
    // std::cout << b << "\n";
    // std::cout << x << "\n";
    // std::cout << "\n\n";

    // Diode branches may involve re-solving if diode blocks current or voltage drop is insufficent
    // Same goes for transistors, which can be detected as 2 diodes sharing a node
    bool re_solve = false;
    if (allow_recursion) {
        re_solve = diode_branches.size() > 0; // Diodes force resolve

        // Increase resistance of invalid diodes
        for (size_t i = 0; i < diode_branches.size(); i++) {
            int index = diode_branches[i].index;
            NodeId node1 = diode_branches[i].node;
            NodeId node2 = connection_map[node1][index];
            double deltaV = x[node2 - 2] - x[node1 - 2];

            if (node1 > node2)
                deltaV *= -1; // Maintain polarity: node1 -> node2 is positive

            Branch * b = branch_map[node1][index];

            // Fail on the following conditions:
            // 1. Current is flowing right way, but does not meet threshold voltage
            // 2. Current is flowing wrong way and does not exceed breakdown voltage
            // (If deltaV < 0, means current flows from node1 --> node2)
            if ((deltaV < 0 && b->diode > 0 && fabs(deltaV) < DIODE_V_THRESHOLD) || // Correct dir
                (deltaV > 0 && b->diode > 0 && fabs(deltaV) < DIODE_V_BREAKDOWN) || // Wrong dir
                (deltaV > 0 && b->diode < 0 && fabs(deltaV) < DIODE_V_THRESHOLD) || // Correct dir
                (deltaV < 0 && b->diode < 0 && fabs(deltaV) < DIODE_V_BREAKDOWN)    // Wrong dir
            ) {
                b->resistance += REALLY_BIG_RESISTANCE;
            }
            else if (fabs(deltaV) > DIODE_V_THRESHOLD) {
                // Voltage drop across diodes :D
                b->voltage_gain = -DIODE_V_THRESHOLD;
            }
        }
        if (re_solve)
            solve(false); // Re-solve without diode branches
    }

    // If we don't need to resolve everything, assign currents to each branch
    if (!re_solve) {
        // Used to track current of adjacent branches for branches
        // that do not obey ohms law. We take the current of the ohmian
        // branch going into the node (should only be 1)

        // Order of nodes doesn't matter, we find all branches that share the first node, as
        // non-ohmian branches must be connected on both ends to be properly connected, ie
        // a voltage source must be arranged as such: [SOME OTHER NODE] -- [NODE1] - V - [NODE2] -- [SOME OTHER NODE]
        // both node1 and node2 must connect to another node, so there will always be a non-ohmian branch to borrow a
        // valid current from
        std::vector<Branch *> non_ohmian_branches;
        double current_of_adjacent = 0.0;
        int node1_of_adjacent = 1;

        // Check if result is the same
        bool solution_is_unchanged = (size_t)x.size() == prev_solution.size();
        if (solution_is_unchanged)
            for (size_t i = 0; i < prev_solution.size(); i++)
                if (prev_solution[i] != x[i])
                    solution_is_unchanged = false;
        if (solution_is_unchanged)
            new_solution_exists = false;
        else {
            prev_solution.clear();
            for (size_t i = 0; i < size; i++)
                prev_solution.push_back(x[i]);
        }

        for (auto node_id = connection_map.begin(); node_id != connection_map.end(); node_id++) {
            // Normal branches
            non_ohmian_branches.clear();
            for (size_t i = 0; i < node_id->second.size(); i++) {
                Branch * b = branch_map[node_id->first][i];
                b->V1 = x[b->node1 - CircuitParams::START_NODE_ID];
                b->V2 = x[b->node2 - CircuitParams::START_NODE_ID];

                if (b->obeys_ohms_law() && b->resistance) {
                    b->current = (b->V1 - b->V2) / b->resistance;
                    current_of_adjacent = b->current;
                    node1_of_adjacent = b->node1;
                }
                else if (!b->obeys_ohms_law())
                    non_ohmian_branches.push_back(b);
            }
            // Branches that do not obey ohms law take current of adjacent branches
            for (auto &b : non_ohmian_branches) {
                b->current = current_of_adjacent;
                if (node1_of_adjacent == b->node1) // Keep alignment of currents correct
                    b->current *= -1;
            }
        }

        for (const auto &float_branch_key : floating_branches) {
            NodeId node2 = float_branch_key.first;

            // Floating branches take voltage of connecting branches
            // Or if the only node is constrained, take that
            for (size_t i = 0; i < floating_branches[node2].size(); i++) {
                floating_branches[node2][i]->V2 = constrained_nodes.count(node2) ?
                    constrained_nodes[node2] :
                    x[node2 - CircuitParams::START_NODE_ID];
            }
        }
    }

    // Divergence check complete, set steady state current and voltage
    if (check_divergence) {
        copy->solve();
        for (auto &p : numeric_integration)  {
            Branch * b1 = branch_map[p.node][p.index];
            Branch * b2 = copy->branch_map[p.node][p.index];
            b1->SS_voltage = b2->V2 - b2->V1;
            b1->SS_current = b2->current;
        }
        new_solution_exists = true;
    }
    computed_divergence = true;
    solution_computed = true;
    for (auto &c : chips)
        c.clear();
}

void Circuit::update_sim() {
    // Skip updating simulation for mostly static components
    if (!contains_dynamic && solution_computed && !new_solution_exists && sim->timer % FORCE_RECALC_EVERY_N_FRAMES != 0)
        goto end_update;

    for (auto node_id = connection_map.begin(); node_id != connection_map.end(); node_id++) {
        // Normal branches
        for (size_t i = 0; i < node_id->second.size(); i++) {
            if (node_id->first > node_id->second[i])
                continue;

            Branch *b = branch_map[node_id->first][i];

            // Check if we can skip
            if (b->obeys_ohms_law() && !new_solution_exists)
                continue;

            ElementType prev_type = -1;
            int x = (int)(0.5f + sim->parts[b->node1_id].x),
                y = (int)(0.5f + sim->parts[b->node1_id].y),
                r = sim->pmap[y][x];
            double voltage_drop = 0.0f;

            // Set voltage and current at nodes
            sim->parts[b->node1_id].pavg[0] = restrict_double_to_flt(b->V1);
            sim->parts[b->node1_id].pavg[1] = restrict_double_to_flt(b->current);
            sim->parts[b->node2_id].pavg[0] = restrict_double_to_flt(b->V2);
            sim->parts[b->node2_id].pavg[1] = restrict_double_to_flt(b->current);

            for (auto id : b->rspk_ids) {
                x = (int)(0.5f + sim->parts[id].x);
                y = (int)(0.5f + sim->parts[id].y);
                r = sim->pmap[y][x];

                // Circuit is invalid - just in case somehow the particle
                // below the RSPK was deleted and the RSPK didn't register
                if (!r) {
                    flag_recalc(true);
                    continue;
                }

                // Consecutive SWCH - don't add resistance otherwise there will be voltage drops
                // (We wish voltage to be uniform across SWCH)
                if (b->obeys_ohms_law() && !(prev_type == PT_SWCH && TYP(r) == PT_SWCH))
                    voltage_drop += get_effective_resistance(TYP(r), sim->parts, ID(r), sim) * b->current;

                sim->parts[id].pavg[0] = restrict_double_to_flt(b->V1 - voltage_drop);
                sim->parts[id].pavg[1] = restrict_double_to_flt(b->current);

                // Post updates:
                if (TYP(r) == PT_CAPR || TYP(r) == PT_INDC) {
                    double step;
                    // Assign voltages for capcaitor: i / C = dV / dt
                    if (TYP(r) == PT_CAPR) {
                        step = INTEGRATION_TIMESTEP / sim->parts[ID(r)].pavg[0] * b->current;
                        if (fabs(sim->parts[ID(r)].pavg[1] - step) > fabs(b->SS_voltage) &&
                                b->SS_voltage != 0 &&
                                (b->SS_voltage != std::numeric_limits<double>::max() ||
                                 b->SS_current != std::numeric_limits<double>::max()))
                            sim->parts[ID(r)].pavg[1] = b->SS_voltage;
                        else if (fabs(sim->parts[ID(r)].pavg[1] - b->SS_voltage) < WITHIN_STEADY_STATE)
                            sim->parts[ID(r)].pavg[1] = b->SS_voltage;
                        else
                            sim->parts[ID(r)].pavg[1] -= step;
                    }
                    // Assign current for inductor: V / L =  dI/dt
                    else if (TYP(r) == PT_INDC) {
                        step = (b->V2 - b->V1) / sim->parts[ID(r)].pavg[0] * INTEGRATION_TIMESTEP;
                        if (fabs(sim->parts[ID(r)].pavg[1] - step) > fabs(b->SS_current) &&
                                b->SS_current != 0 &&
                                (b->SS_voltage != std::numeric_limits<double>::max() ||
                                 b->SS_current != std::numeric_limits<double>::max()))
                            sim->parts[ID(r)].pavg[1] = b->SS_current;
                        else if (fabs(sim->parts[ID(r)].pavg[1] - b->SS_current) < WITHIN_STEADY_STATE)
                            sim->parts[ID(r)].pavg[1] = b->SS_current;
                        else
                            sim->parts[ID(r)].pavg[1] -= step;
                    }
                }
                prev_type = TYP(r);
            }
        }
    }

    // Floating branches
    for (const auto &float_branch_key : floating_branches) {
        NodeId node2 = float_branch_key.first;
        for (size_t i = 0; i < floating_branches[node2].size(); i++) {
            for (size_t j = 0; j < floating_branches[node2][i]->rspk_ids.size(); j++) {
                int id = floating_branches[node2][i]->rspk_ids[j];
                sim->parts[id].pavg[0] = restrict_double_to_flt(floating_branches[node2][i]->V2);
                sim->parts[id].pavg[1] = restrict_double_to_flt(0.0f);
            }

            // Assign voltage at node if there's no other branch connected to the node
            // (ie, in the case with a chip)
            if (!connection_map.count(node2)) {
                int id = floating_branches[node2][i]->node2_id;
                sim->parts[id].pavg[0] = restrict_double_to_flt(floating_branches[node2][i]->V2);
                sim->parts[id].pavg[1] = restrict_double_to_flt(0.0f);
            }
        }
    }

    end_update:;

    // Make RSPK not die, and set flag tmp2 = 0 to trigger recalc for non-skeleton nodes
    for (auto id : global_rspk_ids) {
        sim->parts[id].life = BASE_RSPK_LIFE;
        sim->parts[id].tmp2 = 0;

        // Reset chip flags to 0
        int x = (int)(sim->parts[id].x + 0.5f);
        int y = (int)(sim->parts[id].y + 0.5f);
        if (is_chip(TYP(sim->pmap[y][x])))
            sim->parts[ID(sim->pmap[y][x])].flags = 0;
    }
}


void Circuit::update_chip_io(ParticleId chip_pid, ParticleId terminal_id, bool positive_terminal) {
    ChipId chip_id = sim->parts[chip_pid].tmp2;
    if (chip_id > chips.size())
        chips.resize(chip_id);

    int x = (int)(sim->parts[terminal_id].x + 0.5f);
    int y = (int)(sim->parts[terminal_id].y + 0.5f);
    int r = ID(sim->photons[y][x]);

    if (positive_terminal)
        chips[chip_id - 1].add_input(terminal_id, sim->parts[r].pavg[0], sim->parts[r].pavg[1], sim->parts[terminal_id].type);
    else
        chips[chip_id - 1].add_output(terminal_id, sim->parts[r].tmp);
    chips[chip_id - 1].assign_values(sim->parts[chip_pid].tmp, sim->parts[chip_pid].pavg[0]);
}


void Circuit::reset_effective_resistances() {
    // Don't need to reset: circuit is non-dynamic and we already solved it
    // Force recalc is for when something changes that doesn't delete / add particles,
    // (like a console command)
    if (!contains_dynamic && solution_computed &&
            sim->timer % FORCE_RECALC_EVERY_N_FRAMES != 0)
        return;

    for (auto b : branch_cache) {
        b->recompute_switches = true;
        b->resistance = b->base_resistance;
    }
}

void Circuit::flag_recalc(bool force) {
    if (!force && recalc_cooldown_timer > 0)
        return;

    new_solution_exists = true;
    recalc_next_frame = true;
    solution_computed = false;
    recalc_cooldown_timer = circuit_size < COOLDOWN_PARTICLE_THRESHOLD ?
        COOLDOWN_BELOW_THRESHOLD : COOLDOWN_ABOVE_THRESHOLD;
}

void Circuit::debug() {
    std::cout << "Circuit connections:\n";
    for (auto itr = connection_map.begin(); itr != connection_map.end(); itr++) {
        std::cout << itr->first << " : ";
        for (auto itr2 = itr->second.begin(); itr2 != itr->second.end(); itr2++)
            std::cout << *itr2 << " ";
        std::cout << "\n";
    }
    for (auto b : branch_cache)
        b->print();
}

void Circuit::reset() {
    for (auto id : global_rspk_ids)
        circuit_map[id] = nullptr;
    for (auto b : branch_cache)
        delete b;

    global_rspk_ids.clear();
    constrained_nodes.clear();
    branch_map.clear();
    floating_branches.clear();
    connection_map.clear();
    branch_cache.clear();
    chips.clear();

    recalc_next_frame = false;
    new_solution_exists = true;
    requires_divergence_checking = false;
    contains_dynamic = false;
    solution_computed = false;
    computed_divergence = false;
    highest_node_id = CircuitParams::NOSKELETON;
    recalc_cooldown_timer = 0;
    circuit_size = 0;
}

Circuit::Circuit(int x, int y, Simulation *sim) {
    this->sim = sim;
    startx = x, starty = y;

    generate();
    solve();
    update_sim();

#ifdef DEBUG
    // debug();
#endif
}

Circuit::Circuit(const Circuit &other) {
    sim = other.sim;
    delete_maps();

    connection_map = other.connection_map;
    constrained_nodes = other.constrained_nodes;
    recalc_next_frame = other.recalc_next_frame;
    startx = other.startx, starty = other.starty;
    contains_dynamic = other.contains_dynamic;
    new_solution_exists = other.new_solution_exists;
    requires_divergence_checking = other.requires_divergence_checking;
    highest_node_id = other.highest_node_id;
    circuit_size = other.circuit_size;
    chips = other.chips;

    for (auto node_id = connection_map.begin(); node_id != connection_map.end(); node_id++) {
        for (size_t i = 0; i < node_id->second.size(); i++) {
            Branch * new_b;
            if ((size_t)node_id->first >= branch_map[node_id->second[i]].size() ||
                    !branch_map[node_id->second[i]][node_id->first]) {
                new_b = new Branch(*(other.branch_map.at(node_id->first)[i]));
                branch_cache.push_back(new_b);
            }
            else {
                new_b = branch_map[node_id->second[i]][node_id->first];
            }
            branch_map[node_id->first].push_back(new_b);
        }
        // Floating branches
        for (size_t i = 0; i < floating_branches[node_id->first].size(); i++) {
            Branch * new_b = new Branch(*(other.floating_branches.at(node_id->first)[i]));
            floating_branches[node_id->first].push_back(new_b);
            branch_cache.push_back(new_b);
        }
    }
}

Circuit::~Circuit() {
    delete_maps();
    delete copy;
    for (auto id : get_global_rspk_ids())
        circuit_map[id] = nullptr;
    for (unsigned i = 0; i < branch_cache.size(); i++)
        delete branch_cache[i];
}

void Circuit::delete_maps() {
    delete[] node_skeleton_map;
    delete[] immutable_node_map;
    node_skeleton_map = nullptr;
    immutable_node_map = nullptr;
}
