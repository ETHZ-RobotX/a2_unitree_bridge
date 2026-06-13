/*
 * Selects the correct topic structs at compile time.
 * Build with -DA2_MODE_SIM for simulation, without it for real robot.
 */
#ifndef INTERFACES_H_
#define INTERFACES_H_

#ifdef A2_MODE_SIM
#include "interfaces_sim.hpp"
#else
#include "interfaces_robot.hpp"
#endif

#endif /* INTERFACES_H_ */
