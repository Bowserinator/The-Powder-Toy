#include "simulation/ElementCommon.h"

static int update(UPDATE_FUNC_ARGS);
static int graphics(GRAPHICS_FUNC_ARGS);
static void create(ELEMENT_CREATE_FUNC_ARGS);

void Element::Element_PTNM() {
	Identifier = "DEFAULT_PT_PTNM";
	Name = "PTNM";
	Colour = PIXPACK(0xD5E0EB);
	MenuVisible = 1;
	MenuSection = SC_SOLIDS;
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
	Meltable = 1;
	Hardness = 0;
	Weight = 100;

	HeatConduct = 251;
	Description = "Platinum. Catalyzes certain reactions.";

	Properties = TYPE_SOLID | PROP_CONDUCTS | PROP_LIFE_DEC | PROP_HOT_GLOW | PROP_SPARKSETTLE;

	LowPressure = IPL;
	LowPressureTransition = NT;
	HighPressure = IPH;
	HighPressureTransition = NT;
	LowTemperature = ITL;
	LowTemperatureTransition = NT;
	HighTemperature = 1768.0f + 273.15f;
	HighTemperatureTransition = PT_LAVA;

	Update = &update;
	Graphics = &graphics;
	Create = &create;
}

static int next_shield_transition(int type) {
	switch(type) {
		case PT_SHLD1: return PT_SHLD2;
		case PT_SHLD2: return PT_SHLD3;
		case PT_SHLD3: return PT_SHLD4;
	}
	return PT_SHLD1;
}

static int update(UPDATE_FUNC_ARGS) {
	int hygn1_id = -1; // Id of a hydrogen particle for all the hydrogen reactions

	// Fast conduction (like GOLD)
	static int checkCoordsX[] = { -4, 4, 0, 0 };
	static int checkCoordsY[] = { 0, 0, -4, 4 };

	if (!parts[i].life) {
		for (int j = 0; j < 4; j++) {
			int rx = checkCoordsX[j];
			int ry = checkCoordsY[j];
			int r = pmap[y + ry][x + rx];
			if (!r) continue;
			if (TYP(r) == PT_SPRK && parts[ID(r)].life && parts[ID(r)].life < 4) {
				sim->part_change_type(i, x, y, PT_SPRK);
				parts[i].life = 4;
				parts[i].ctype = PT_PTNM;
			}
		}
	}

	// Single element reactions
	for (int rx = -1; rx <= 1; rx++)
	for (int ry = -1; ry <= 1; ry++)
		if (BOUNDS_CHECK && (rx || ry)) {
			int r = pmap[y + ry][x + rx];
			if (!r) continue;
			int rt = TYP(r);

			if (rt == PT_H2 && hygn1_id < 0)
				hygn1_id = ID(r);

			// These reactions will occur instantly in contact with PTNM
			// --------------------------------------------------------

			// Shield instantly grows (even without SPRK)
			if (!parts[ID(r)].life && (rt == PT_SHLD1 || rt == PT_SHLD2 || rt == PT_SHLD3)) {
				int next = next_shield_transition(rt);
				sim->part_change_type(ID(r), x + rx, y + ry, next);
				parts[ID(r)].life = 7;
				continue;
			}

				// These reactions are dependent on temperature
				// Probability goes quadratically from 0% / frame to 100% / frame from 0 C to 1500 C
				// --------------------------------------------------------
				float prob = std::min(1.0f, parts[i].temp / (273.15f + 1500.0f));
				prob *= prob;

				if (RNG::Ref().uniform01() <= prob)
				{
					switch (rt)
					{
					case PT_GAS: // GAS + > 2 pressure + >= 200 C -> INSL
						if (parts[ID(r)].temp >= 200.0f + 273.15f && sim->pv[(y + ry) / CELL][(x + rx) / CELL] > 2.0f)
						{
							sim->part_change_type(ID(r), x + rx, y + ry, PT_INSL);
							parts[i].temp += 60.0f; // Other part is INSL, adding temp is useless
						}
						break;

					case PT_BREC: // BREL + > 1000 C + > 50 pressure -> EXOT
						if (parts[ID(r)].temp > 1000.0f + 273.15f && sim->pv[(y + ry) / CELL][(x + rx) / CELL] > 50.0f)
						{
							sim->part_change_type(ID(r), x + rx, y + ry, PT_EXOT);
							parts[ID(r)].temp -= 30.0f;
							parts[i].temp -= 30.0f;
						}
						break;
					// SMKE -> CO2
					case PT_SMKE:
						sim->part_change_type(ID(r), x + rx, y + ry, PT_CO2);
						break;
					// COAL / BCOL + > 5 pressure -> SUFR
					case PT_COAL:
					case PT_BCOL:
						if (sim->pv[y / CELL][x / CELL] > 5.0f && RNG::Ref().chance(1, 100))
							sim->part_change_type(ID(r), x + rx, y + ry, PT_SUFR);
						break;
				}
			}
		}

	// Hydrogen reactions
	if (hygn1_id >= 0) {
		for (int rx = -1; rx <= 1; rx++)
		for (int ry = -1; ry <= 1; ry++)
			if (BOUNDS_CHECK && (rx || ry)) {
				int r = pmap[y + ry][x + rx];
				if (!r || ID(r) == hygn1_id) continue;
				int rt = TYP(r);

				// HYGN + DESL -> OIL + WATR
				if (rt == PT_DESL) {
					sim->part_change_type(ID(r), x + rx, y + ry, PT_WATR);
					sim->part_change_type(hygn1_id, (int)(parts[hygn1_id].x + 0.5f),
						(int)(parts[hygn1_id].y + 0.5f), PT_OIL);
					goto end_hydrogen_reactions;
				}

				// HYGN + OXYG -> DSTW + SPRK + Heat
				if (rt == PT_O2 && !parts[i].life) {
					sim->part_change_type(ID(r), x + rx, y + ry, PT_DSTW);
					sim->part_change_type(hygn1_id, (int)(parts[hygn1_id].x + 0.5f),
						(int)(parts[hygn1_id].y + 0.5f), PT_DSTW);
					parts[ID(r)].temp += 5.0f;
					parts[hygn1_id].temp += 5.0f;

					parts[i].ctype = PT_PTNM;
					parts[i].life = 4;
					sim->part_change_type(i, x, y, PT_SPRK);
					goto end_hydrogen_reactions;
				}

				// Cold fusion: 2 hydrogen > 500 C has a chance to fuse
				if (rt == PT_H2 && RNG::Ref().chance(1, 1000) &&
						parts[ID(r)].temp > 500.0f + 273.15f && parts[hygn1_id].temp > 500.0f + 273.15f) {
					sim->part_change_type(ID(r), x + rx, y + ry, PT_NBLE);
					sim->part_change_type(hygn1_id, (int)(parts[hygn1_id].x + 0.5f),
						(int)(parts[hygn1_id].y + 0.5f), PT_NEUT);
					
					parts[ID(r)].temp += 1000.0f;
					parts[hygn1_id].temp += 1000.0f;
					sim->pv[y / CELL][x / CELL] += 10.0f;

					int j = sim->create_part(-3, x + rx, y + ry, PT_PHOT);
					if (j > -1) {
						parts[j].ctype = 0x7C0000;
						parts[j].temp = parts[ID(r)].temp;
						parts[j].tmp = 0x1;
					}
					if (RNG::Ref().chance(1, 10)) {
						int j = sim->create_part(-3, x + rx, y + ry, PT_ELEC);
						if (j > -1)
							parts[j].temp = parts[ID(r)].temp;
					}
					goto end_hydrogen_reactions;
				}
			}
	}
	end_hydrogen_reactions:;

	return 0;
}

static int graphics(GRAPHICS_FUNC_ARGS) {
	if (cpart->tmp)
		*pixel_mode |= PMODE_FLARE;
	return 0;
}

static void create(ELEMENT_CREATE_FUNC_ARGS) {
	if (RNG::Ref().chance(1, 15))
		sim->parts[i].tmp = 1;
}
