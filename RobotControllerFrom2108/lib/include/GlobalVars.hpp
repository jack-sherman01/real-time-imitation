#pragma once

#include <memory>

#include "SolverBase.hpp"
#include "CommandHandler.hpp"
#include "Simulation/CommandHandlerSim.hpp"

std::shared_ptr<Robot> robot;
std::shared_ptr<CommandHandler> ch;
std::shared_ptr<CommandHandlerSim> chSim;

// define for apiBlock: communication by API
std::shared_ptr<APIBlock> api;
