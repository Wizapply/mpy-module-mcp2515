# Location of top-level MicroPython directory
MPY_DIR = ../

# Name of module
MOD = mcp2515

# Source files (.c or .py)
SRC = mcp2515_main.c MCP2515.c

# Architecture to build for (x86, x64, armv6m, armv7m, xtensa, xtensawin)
ARCH = armv6m

# Include to get the rules for compiling and linking the module
include $(MPY_DIR)/py/dynruntime.mk