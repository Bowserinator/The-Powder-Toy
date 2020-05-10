#include "simulation/circuits/framework.h"
#include "simulation/circuits/resistance.h"
#include "simulation/CoordStack.h"

#include <vector>
#include <iostream>

bool can_be_skeleton(int i, Simulation * sim) {
    int typ = sim->parts[i].type;
    if (!typ) return false;
    return sim->elements[typ].Properties & TYPE_SOLID ||
        (fabs(sim->parts[i].vx) < 1.0f && fabs(sim->parts[i].vy) < 1.0f);
}

bool can_be_node(int i, Simulation * sim) {
    int typ = sim->parts[i].type;
    if (is_chip(typ)) // Chips can't contain nodes
        return false;
    return sim->elements[typ].Properties & TYPE_SOLID || (typ == PT_CRBN && sim->parts[i].tmp2);
}

/**
 * Is one type allowed to conduct to another?
 */
bool allow_conduction(int totype, int fromtype) {
    if (!totype) return false; // Cannot conduct to NONE
    // SWCH cannot conduct to NSCN / PSCN or SPRK can't toggle it
    if (fromtype == PT_SWCH && (totype == PT_NSCN || totype == PT_PSCN)) return false;
    // INWR can only conduct to negative terminal, or recieve from positive
    if (fromtype == PT_INWR && totype != PT_INWR && !is_negative_terminal(totype) && totype != PT_GRND)
        return false;
    if (totype == PT_INWR && fromtype != PT_INWR && !is_positive_terminal(fromtype) && fromtype != PT_GRND)
        return false;
    return true;
}

/*
 * Floodfill touching conductors with RSPRK
 */
coord_vec floodfill(Simulation *sim, int x, int y) {
    int tmp = sim->pmap[y][x];
    if (!valid_conductor(TYP(tmp), sim, ID(tmp)))
        return coord_vec();

    CoordStack coords, offsets;
    coord_vec output;
    int crx, cry; // Temp rx and ry from offsets for junctions
    ChipId initial_chip_id = 1;

    bool visited[YRES][XRES];
    std::fill(&visited[0][0], &visited[YRES][0], 0);
 
    coords.push(x, y);
    offsets.push(0, 0);

	while (coords.getSize()) {
		coords.pop(x, y);
        offsets.pop(crx, cry);

		if (!sim->photons[y][x]) {
			int j = sim->create_part(-3, x, y, PT_RSPK);
            if (j < 0) break; // Unable to create new particles
        }

        int r = sim->pmap[y][x];
        if (TYP(r) != PT_JUNC)
            visited[y][x] = true;
        if (is_chip(TYP(r)) && !sim->parts[ID(r)].flags) {
            PropertyValue value;
			value.Integer = initial_chip_id;
			sim->flood_prop(x, y, offsetof(Particle, tmp2), value, StructProperty::Integer);
            value.Integer = 1;
            sim->flood_prop(x, y, offsetof(Particle, flags), value, StructProperty::Integer);
            initial_chip_id++;
        }
        output.push_back(Pos(x, y));

		// Floodfill
		for (int rx = -1; rx <= 1; ++rx)
		for (int ry = -1; ry <= 1; ++ry)
		if ((rx || ry)) {
			// Floodfill if valid spot.
			ElementType fromtype = TYP(r);
			ElementType totype = TYP(sim->pmap[y + ry][x + rx]);

            if (totype == PT_JUNC && rx && ry)
                continue;
            if (fromtype == PT_JUNC && totype == PT_JUNC && (rx != crx || ry != cry))
                continue;
			if (allow_conduction(totype, fromtype) && !visited[y + ry][x + rx] &&
                    valid_conductor(totype, sim, ID(sim->pmap[y + ry][x + rx]))) {
				coords.push(x + rx, y + ry);
                offsets.push(rx, ry);
                if (totype != PT_JUNC)
                    visited[y + ry][x + rx] = true;
            }
		}
	}

    return output;
}

/**
 * Helper for skeletization
 * Taken from https://web.archive.org/web/20160322113207/http://opencv-code.com/quick-tips/implementation-of-thinning-algorithm-in-opencv/
 * (Zhang-Suen algorithm for image thinning)
 */
void coord_stack_to_skeleton_iteration(Simulation *sim, coord_vec &output,
        int (&output_map)[YRES][XRES], short iter) {
    std::vector<int> delete_these_indices;
    short x, y, p2, p3, p4, p5, p6, p7, p8, p9, typ;
    delete_these_indices.reserve(output.size() / 2);

    for (unsigned int i = 0; i < output.size(); i++) {
        x = output[i].x;
        y = output[i].y;

        // Edge bounds check
        if (x == 0 || x == XRES - 1 || y == 0 || y == YRES - 1)
            continue;

        // These elements are always trimmed
        if (!can_be_skeleton(ID(sim->pmap[y][x]), sim)) {
            delete_these_indices.push_back(i);
            continue;
        }

        // Essential circuit elements cannot be trimmed if only 1 px remains
        typ = TYP(sim->pmap[y][x]);
        if (typ == PT_VOLT || typ == PT_GRND || typ == PT_CAPR || typ == PT_INDC) {
            int tcount = 0;
            for (int rx = -1; rx <= 1; rx++)
            for (int ry = -1; ry <= 1; ry++) {
                if ((rx || ry) && TYP(sim->pmap[y + ry][x + rx]) == typ)
                    tcount++;
                if (tcount > 2) // > 2 check, otherwise it allows too many to be deleted in rare situations
                    goto exit_loop;
            }
            continue;
        }
        exit_loop:;

        // Thinning algorithim
        // Get surrounding points, clockwise starting from point above
        p2 = output_map[y-1][x];
        p3 = output_map[y-1][x+1];
        p4 = output_map[y][x+1];
        p5 = output_map[y+1][x+1];
        p6 = output_map[y+1][x];
        p7 = output_map[y+1][x-1];
        p8 = output_map[y][x-1];
        p9 = output_map[y-1][x-1];

        // A(P1) is the number of 0 to 1 transitions in a clockwise direction from P9 back to itself
        // and B(P1) is the number of non-zero neighbors of P1.
        int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + 
                 (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) + 
                 (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                 (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
        int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
        int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
        int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

        if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
            delete_these_indices.push_back(i);
    }

    // Delete all indices marked for deletion
    for (int i = delete_these_indices.size() - 1; i >= 0; i--) {
        output_map[output[delete_these_indices[i]].y][output[delete_these_indices[i]].x] = CircuitParams::NOSKELETON;
        output.erase(output.begin() + delete_these_indices[i]);
    }
}

/*
 * Returns a new vector of coordinates that can be considered
 * the middle portion (a 1px wide skeleton)
 */
coord_vec coord_vec_to_skeleton(Simulation *sim, const coord_vec &floodfill, bool &has_chip) {
    coord_vec output(floodfill);
    size_t prev_size = 0;
    int output_map[YRES][XRES];
    int diff;

    std::fill(&output_map[0][0], &output_map[YRES][0], CircuitParams::NOSKELETON);
    for (Pos p : floodfill)
        output_map[p.y][p.x] = CircuitParams::SKELETON;

    // Repeatedly thin the image to 1 px until it can no longer be further thined
    do {
        coord_stack_to_skeleton_iteration(sim, output, output_map, 0);
        coord_stack_to_skeleton_iteration(sim, output, output_map, 1);

        diff = prev_size - output.size();
        prev_size = output.size();
    } while (diff);

    // Remove chips from the output
    // This step is done last because we want the skeleton to initially
    // pass through the chip, then we remove the chip part, otherwise nodes
    // might not fully connect to the chip
    for (int i = output.size() - 1; i >= 0; i--) {
        int typ = TYP(sim->pmap[output[i].y][output[i].x]);
        if (is_chip(typ)) {
            has_chip = true;
            output_map[output[i].y][output[i].x] = CircuitParams::NOSKELETON;
            output.erase(output.begin() + i);
        }
    }

    return output;
}
