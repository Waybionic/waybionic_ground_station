#pragma once

// Arduino entry point for the WayBionic CAN code. Everything under common/ is plain C++ with
// no Arduino, Linux or ROS headers, no heap and no exceptions; the waybionic_can CMake build
// compiles and tests the same files on the host. r4/ (the UNO R4's built-in CAN controller) and
// mcp2515/ (an MCP2515 module on SPI) are the only board-specific parts.

#include "common/BenchTelemetry.h"
#include "common/BringupConsole.h"
#include "common/BusLoad.h"
#include "common/Candump.h"
#include "common/Frame.h"
#include "common/GatewayLogic.h"
#include "common/IActuator.h"
#include "common/ICanTransport.h"
#include "common/Mcp2515CanTransport.h"
#include "common/MksFrame.h"
#include "common/NodeLogic.h"
#include "common/SlcanBridge.h"
#include "common/SoftwareActuator.h"

#include "mcp2515/ArduinoMcp2515Spi.h"
#include "r4/R4CanTransport.h"
#include "r4/UsbHandBack.h"
