#pragma once

#include <vector>

#include "Particle.h"
#include "simulation/circuits/circuits.h"
#include "json/json.h"

class Snapshot
{
public:
	std::vector<float> AirPressure;
	std::vector<float> AirVelocityX;
	std::vector<float> AirVelocityY;
	std::vector<float> AmbientHeat;

	std::vector<Particle> Particles;

	std::vector<float> GravVelocityX;
	std::vector<float> GravVelocityY;
	std::vector<float> GravValue;
	std::vector<float> GravMap;

	std::vector<unsigned char> BlockMap;
	std::vector<unsigned char> ElecMap;
	std::vector<unsigned short> FaradayMap;
	unsigned int timer;

	std::vector<char> TimeDilation;
	std::vector<float> FanVelocityX;
	std::vector<float> FanVelocityY;
	std::vector<short> oneWayDir;
	std::vector<Circuit> circuits;

	std::vector<std::tuple<short, short, short, Particle> > PortalParticles;
	std::vector<unsigned char> WirelessData;
	std::vector<playerst> stickmen;
	std::vector<sign> signs;

	Json::Value Authors;

	Snapshot() :
		AirPressure(),
		AirVelocityX(),
		AirVelocityY(),
		AmbientHeat(),
		Particles(),
		GravVelocityX(),
		GravVelocityY(),
		GravValue(),
		GravMap(),
		BlockMap(),
		ElecMap(),
		FanVelocityX(),
		FanVelocityY(),
		PortalParticles(),
		WirelessData(),
		stickmen(),
		signs()
	{

	}

	virtual ~Snapshot()
	{

	}
};
