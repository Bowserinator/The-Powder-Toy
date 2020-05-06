#ifndef CIRCUIT_CORE
#define CIRCUIT_CORE

#include <vector>
#include <exception>

/* Defines for variable types to be less ambigious in code */
typedef unsigned short NodeId;
typedef int ElementType;
typedef int ParticleId;

typedef double Ohms;
typedef double Volts;
typedef double Amps;

class Pos {
public:
    short x, y;

    Pos(int x, int y) : x((short)x), y((short)y) {}
    Pos(short x, short y) : x(x), y(y) {}

    inline bool operator==(const Pos &other) const { return x == other.x && y == other.y; }
};
typedef std::vector<Pos> coord_vec;

// Exceptions
class CircuitParticleMapDesync: public std::exception {
    inline virtual const char* what() const throw() {
        return "Circuit tried to access non-existent conductor, circuit id map and particle map may have desynced.";
    }
};

// Config variables
const int BASE_RSPK_LIFE = 4;
const int FORCE_RECALC_EVERY_N_FRAMES = 20;

// Divergence checking
const double WITHIN_STEADY_STATE = 0.0001;                // Error < value is basically steady state
const double INTEGRATION_TIMESTEP = 0.016666666666666666; // 1 frame = 1/60 s
const int INTEGRATION_RECALC_EVERY_N_FRAMES = 20;

// Component limits
const Amps MAX_CAPACITOR_CURRENT = 10000.0f;
const Amps MAX_CAPACITOR_REVERSE_CURRENT = 1.0f;

// Resistances
const Ohms REALLY_BIG_RESISTANCE = 10000000000000.0;
const Ohms SUPERCONDUCTING_RESISTANCE = 1e-16;
const Ohms DEFAULT_RESISTANCE = 5e-7; // Random default resistance value if not defined

const Volts DIODE_V_THRESHOLD = 0.7f;
const Volts DIODE_V_BREAKDOWN = 1000.0f;

// Cooldown for recalc
const int COOLDOWN_PARTICLE_THRESHOLD = 1500;
const int COOLDOWN_BELOW_THRESHOLD = 0;  // In frames
const int COOLDOWN_ABOVE_THRESHOLD = 12; // In frames

#endif