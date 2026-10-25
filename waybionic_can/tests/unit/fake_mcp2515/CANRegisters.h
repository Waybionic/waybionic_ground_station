#pragma once

// The MCP2515 registers and bits the transport uses, as in AA_MCP2515's CANRegisters.h and the
// datasheet (DS20001801).

#define INSTRUCTION_RTS0 0x81

#define READSTATUS_TX0REQ (1 << 2)
#define READSTATUS_RX1IF (1 << 1)
#define READSTATUS_RX0IF (1 << 0)

#define CANSTAT 0x0e
#define CANCTRL 0x0f
#define CNF3 0x28
#define CNF2 0x29
#define CNF1 0x2a
#define CANINTF 0x2c
#define EFLG 0x2d
#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define RXB0CTRL 0x60
#define RXB0SIDH 0x61
#define RXB1CTRL 0x70
#define RXB1SIDH 0x71

#define CANSTAT_OPMOD_MASK 0b11100000
#define CANCTRL_REQOP_MASK 0b11100000

#define CANINTF_RX1IF (1 << 1)
#define CANINTF_RX0IF (1 << 0)

#define EFLG_RX1OVR (1 << 7)
#define EFLG_RX0OVR (1 << 6)
#define EFLG_TXBO (1 << 5)
#define EFLG_TXEP (1 << 4)
#define EFLG_RXEP (1 << 3)

#define TXBnCTRL_TXREQ (1 << 3)
#define RXB0CTRL_BUKT (1 << 2)
