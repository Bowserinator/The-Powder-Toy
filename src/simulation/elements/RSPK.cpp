#include "simulation/ElementCommon.h"
#include "simulation/magnetics/magnetics.h"

#include "simulation/circuits/circuits.h"
#include "simulation/circuits/resistance.h"
#include "simulation/circuits/framework.h"
#include "simulation/circuits/components/circuit.h"

static int update(UPDATE_FUNC_ARGS);
static int graphics(GRAPHICS_FUNC_ARGS);
static void change_type(ELEMENT_CHANGETYPE_FUNC_ARGS);

static void copy_values(UPDATE_FUNC_ARGS);

#include <vector>
#include <set>
#include <iostream>

const static std::vector<int> itrx({ -1, -1, -1, 0, 0, 1, 1, 1 });
const static std::vector<int> itry({ -1, 0, 1, -1, 1, -1, 0, 1 });

namespace RSPK {
const int REFRESH_EVERY_FRAMES = 5;


float get_power(int x, int y, Simulation *sim) {
	int r = sim->photons[y][x];
	float voltage = sim->parts[ID(r)].pavg[1];
	r = sim->pmap[y][x];
	float resistance = get_resistance(TYP(r), sim->parts, ID(r), sim);
	if (resistance == 0.0f)
		return 0.0f;
	return voltage / resistance * voltage;
}
};

void Element::Element_RSPK() {
	Identifier = "DEFAULT_PT_RSPK";
	Name = "RSPK";
	Colour = PIXPACK(0xFFFF80);
	MenuVisible = 1;
	MenuSection = SC_ELECTROMAG;
	Enabled = 1;

	Advection = 0.0f;
	AirDrag = 0.00f * CFDS;
	AirLoss = 0.90f;
	Loss = 0.00f;
	Collision = 0.0f;
	Gravity = 0.0f;
	Diffusion = 0.00f;
	HotAir = 0.000f	* CFDS;
	Falldown = 0;

	Flammable = 0;
	Explosive = 0;
	Meltable = 0;
	Hardness = 1;
	PhotonReflectWavelengths = 0x00000000;

	Weight = -1;

	HeatConduct = 0;
	Description = "Instant spark with voltage.";

	Properties = TYPE_ENERGY;

	LowPressure = IPL;
	LowPressureTransition = NT;
	HighPressure = IPH;
	HighPressureTransition = NT;
	LowTemperature = ITL;
	LowTemperatureTransition = NT;
	HighTemperature = ITH;
	HighTemperatureTransition = NT;

	DefaultProperties.life = 10;

	Update = &update;
	Graphics = &graphics;
	ChangeType = &change_type;
}

static void change_type(ELEMENT_CHANGETYPE_FUNC_ARGS) {
	if (circuit_map[i])
		circuit_map[i]->flag_recalc(true);
	circuit_map[i] = nullptr;
}

static int update(UPDATE_FUNC_ARGS) {
	/**
	 * Properties:
	 * tmp   - Current node ID, 1 = branch, 0 = not part of skeleton
	 * tmp2  - Whether non-skeleton nodes are updated (1 = up to date from skeleton, 2 = up to date by proxy)
	 * pavg0 - Voltage relative to ground
	 * pavg1 - Current through pixel
	 */
	if (!circuit_map[i] || !pmap[y][x] || !valid_conductor(TYP(pmap[y][x]), sim, ID(pmap[y][x]))) {
		sim->kill_part(i);
		return 1;
	}

	parts[i].life--;
	if (parts[i].life < 0) {
		sim->kill_part(i);
		return 1;
	}
	parts[i].vx = parts[i].vy = 0; // Negate velocity, or it can be affected by Newtonian gravity

	// Detect new unconducted particles
	for (int rx =- 1; rx <= 1; rx++)
	for (int ry = -1; ry <= 1; ry++)
		if (BOUNDS_CHECK && (rx || ry)) {
			int r = sim->photons[y + ry][x + rx];
			int fromtype = TYP(pmap[y][x]);
			int totype = TYP(pmap[y + ry][x + rx]);

			if (TYP(r) != PT_RSPK && circuit_map[i] && valid_conductor(totype, sim, ID(pmap[y + ry][x + rx])) &&
			   		allow_conduction(totype, fromtype)) {
				circuit_map[i]->flag_recalc();
			}

			if (TYP(r) == PT_RSPK) {
				if ((rx == 0 || ry == 0) && is_chip(totype) && is_terminal(fromtype)) {
					circuit_map[i]->update_chip_io(
						ID(pmap[y + ry][x + rx]),
						ID(pmap[y][x]), is_positive_terminal(fromtype));
				}
			}
		}

	int ontype = TYP(pmap[y][x]);
	double resistance = get_effective_resistance(ontype, parts, ID(pmap[y][x]), sim);

	// Heat up the conductor its on
	double power = parts[i].pavg[1] * parts[i].pavg[1] * resistance; // I^2 * R
	if (!doesnt_heat_up(ontype)) {
		if (power > MAX_TEMP)
			parts[ID(pmap[y][x])].temp = MAX_TEMP;
		else
			parts[ID(pmap[y][x])].temp += power;
	}
	
	// Function of RSPK depending on particle it's on
	switch (ontype) {
		case PT_SPRK:
			sim->kill_part(i);
			return 1;
		case PT_ETRD:
		case PT_NBLE:
		case PT_TESC:
			break;
	}
 
	// Set ctype and temp to whats under it
	parts[i].ctype = TYP(pmap[y][x]);
	parts[i].temp = parts[ID(pmap[y][x])].temp;

	if (!ignores_non_skeleton(parts[i].ctype))
		copy_values(sim, i, x, y, surround_space, nt, parts, pmap);

	// Project electric field
	// float efield = res == 0.0f ? 0.0f : isign(parts[i].pavg[0]) * parts[i].pavg[1] / res;
	// sim->emfield->electric[FASTXY(x / EMCELL, y / EMCELL)] += efield;

	// if (parts[i].ctype != PT_VOLT) {
	// 	for (int rx = -1; rx <= 1; ++rx)
	// 	for (int ry = -1; ry <= 1; ++ry)
	// 		if (BOUNDS_CHECK && (rx || ry)) {
	// 			r = pmap[y + ry][x + rx];
	// 			if (!r && RNG::Ref().chance(1, 100)) {
	// 				// St elmo's fire
	// 				if (parts[i].pavg[0] > 100000) {
	// 					int ni = sim->create_part(-1, x + rx ,y + ry, PT_PLSM);
	// 					if (ni > -1) {
	// 						parts[ni].temp = parts[i].temp;
	// 						parts[ni].life = RNG::Ref().between(0, 70);
	// 					}
	// 				}
	// 				// Thermonic emission
	// 				if (parts[i].pavg[0] > 1000) {
	// 					// int ni = sim->create_part(-3, x + rx, y + ry, PT_ELEC);
	// 					// if (ni > -1) {
	// 					// 	parts[ni].temp = parts[i].temp;
	// 					// 	parts[ni].life = RNG::Ref().between(0, 570);
	// 					// 	parts[ni].vx = rx;
	// 					// 	parts[ni].vy = ry;
	// 					// }
	// 				}
	// 			}
	// 			// Ionizing gases
	// 			if (parts[i].pavg[0] > 1000 && sim->elements[TYP(r)].Properties & TYPE_GAS) {
	// 				sim->part_change_type(ID(r), x + rx, y + ry, PT_PLSM);
	// 				parts[ID(r)].life = RNG::Ref().between(0, 570);
	// 			}
	// 		}
	// }

	// Merge self with any RSPK occupying same space
	while (true) {
		int r = sim->photons[y][x];
		if (r && ID(r) != i && TYP(r) == PT_RSPK) {
			parts[i].pavg[0] += parts[ID(r)].pavg[0];
			parts[i].pavg[1] += parts[ID(r)].pavg[1];
			sim->kill_part(ID(r));
		}
		else { break; }
	}

	return 0;
}

static int graphics(GRAPHICS_FUNC_ARGS) {
	*colr = 255; *colg = *colb = 0;
	*cola = 255;

	if (cpart->tmp2 == 1 && !cpart->tmp) {
		*colr = 255;
		*colg = 255;
		*colb = 0;
		if (cpart->tmp2 == 2)
			*colg = 180;
	}
	else if (!cpart->tmp) {
		*colr = *colg = *colb = 200;
	}

	if (cpart->tmp > 1) {
		*colr = 0;
		*colg = 0;
		*colb = 255;
		//ren->fillcircle(cpart->x - 6, cpart->y - 6, 5, 5, 255, 255, 0, 55);
		//ren->drawtext(cpart->x, cpart->y - 10, String::Build(cpart->pavg[1]), 0,0,0, 255);
	}
	return 0;

	// Power = V^2 / R and is used for brightness
	float power = RSPK::get_power(nx, ny, ren->sim);
	*firea = clamp_flt(power / 10.0f, 0, 500);

	*firer = *colr / 2;
	*fireg = *colg / 2;
	*fireb = *colb / 2;
	*pixel_mode |= FIRE_SPARK | PMODE_NONE;

	if (cpart->ctype == PT_NEON) {
		int r = ren->sim->pmap[ny][nx];
		*colr = PIXR(ren->sim->parts[ID(r)].dcolour);
		*colg = PIXG(ren->sim->parts[ID(r)].dcolour);
		*colb = PIXB(ren->sim->parts[ID(r)].dcolour);
	}
	else if (*firea < 20 && cpart->ctype > 0 && cpart->ctype < PT_NUM) {
		int o_r = PIXR(ren->sim->elements[cpart->ctype].Colour);
		int o_g = PIXG(ren->sim->elements[cpart->ctype].Colour);
		int o_b = PIXB(ren->sim->elements[cpart->ctype].Colour);

		*colr = (*colr - o_r) * (*firea + 1) / 20.0f + o_r;
		*colg = (*colg - o_g) * (*firea + 1) / 20.0f + o_g;
		*colb = (*colb - o_b) * (*firea + 1) / 20.0f + o_b;
	}
	return 0;
}

/**
 * Copy values from skeleton for non-skeleton RSPK
 */
static void copy_values(UPDATE_FUNC_ARGS) {
	if (!parts[i].tmp && !parts[i].tmp2) {
		int r2 = -1;
		int new_tmp2 = 1;

		// Check skeleton first
		for (auto &rxy : ADJACENT_PRIORITY) {
			int rx = rxy.first, ry = rxy.second;
			int r = sim->photons[y + ry][x + rx];
			if (TYP(r) == PT_RSPK && parts[ID(r)].tmp) {
				r2 = r;
				goto end;
			}
		}
		// Next check already scanned nearby
		for (auto &rxy : ADJACENT_PRIORITY) {
			int rx = rxy.first, ry = rxy.second;
			int r = sim->photons[y + ry][x + rx];
			if (TYP(r) == PT_RSPK && parts[ID(r)].tmp2 == 1) {
				r2 = r, new_tmp2 = 2;
				goto end;
			}
		}
		// Scan in a spoke circle pattern for skeleton
		for (int delta = 1; delta < XRES; delta++) {
			std::set<int> invalidated_directions;
			for (int dx : itrx)
			for (int dy : itry) {
				// Current direction cannot be traversed
				int direction_hash = dy * 10 + dx;
				if (invalidated_directions.find(direction_hash) != invalidated_directions.end())
					continue;
				if (invalidated_directions.size() == itrx.size())
					goto end; // All directions are invalid now

				int delta2 = std::round(delta * ((dx != 0 && dy != 0) ? 0.7f : 1.0f));
				if (x + delta2 * dx >= XRES || x + delta2 * dx < 0 ||
					y + delta2 * dy >= YRES || y + delta2 * dy < 0)
					continue;
				
				int r = sim->photons[y + delta2 * dy][x + delta2 * dx];
				if (!r || TYP(r) != PT_RSPK) {
					invalidated_directions.insert(direction_hash);
					continue;
				}

				if (parts[ID(r)].tmp || parts[ID(r)].tmp2 == 1) {
					r2 = r;
					goto end;
				}
			}	
		}

		end:;
		// Failed to find a skeleton, at this point we'll copy anything
		if (r2 < 0) {
			parts[i].tmp2 = 2; // So it doesn't keep trying and lag the game
			for (auto &rxy : ADJACENT_PRIORITY) {
				int rx = rxy.first, ry = rxy.second;
				int r = sim->photons[y + ry][x + rx];
				if (TYP(r) == PT_RSPK && (parts[ID(r)].pavg[0] || parts[ID(r)].pavg[1])) {
					r2 = r;
					goto end2;
				}
			}
		}
		end2:;

		if (r2 > -1) {
			int r = r2;
			parts[i].pavg[0] = parts[ID(r)].pavg[0];
			parts[i].pavg[1] = parts[ID(r)].pavg[1];
			parts[i].tmp2 = new_tmp2;

			if (!circuit_map[i] && circuit_map[ID(r)])
				circuit_map[i] = circuit_map[ID(r)];
		}
	}
}
