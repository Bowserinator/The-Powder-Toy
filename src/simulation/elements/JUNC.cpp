#include "simulation/ElementCommon.h"

//#TPT-Directive ElementClass Element_JUNC PT_JUNC 259
Element_JUNC::Element_JUNC()
{
	Identifier = "DEFAULT_PT_JUNC";
	Name = "JUNC";
	Colour = PIXPACK(0x358f36);
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
	HotAir = 0.000f * CFDS;
	Falldown = 0;

	Flammable = 0;
	Explosive = 0;
	Meltable = 0;
	Hardness = 1;

	Weight = 100;

	HeatConduct = 251;
	Description = "Crossover junction. Allows wires to cross without interference.";

	Properties = TYPE_SOLID | PROP_CONDUCTS | PROP_LIFE_DEC;

	LowPressure = IPL;
	LowPressureTransition = NT;
	HighPressure = IPH;
	HighPressureTransition = NT;
	LowTemperature = ITL;
	LowTemperatureTransition = NT;
	HighTemperature = 3000.0f;
	HighTemperatureTransition = PT_LAVA;

	Update = NULL;
	Graphics = NULL;
}

Element_JUNC::~Element_JUNC() {}