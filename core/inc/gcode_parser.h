#pragma once

#include <stdbool.h>

typedef enum {
    CMD_VALID,
    CMD_INVALID
} GCommandType;

typedef struct {
    // 1. Primary Command
    char command_letter;  // Usually 'G', 'M', or 'T'
    int command_number;   // e.g., 1 for G1, 104 for M104
    GCommandType type;    // valid/invalid

    // 2. Presence Flags (Crucial for knowing if a value was updated)
    // In deeply embedded systems, you might compress this into a single uint16_t bitmask
    bool has_X;
    bool has_Y;
    bool has_Z;
    bool has_E; // Extruder
    bool has_F; // Feedrate
    bool has_S; // Spindle speed / Temperature
    bool has_P; // Dwell time / Parameter

    // 3. Parameter Values
    float X;
    float Y;
    float Z;
    float E;
    float F;
    int S; 
    int P; 
} GCodeCommand;

bool parse_command(const char* gcode_line, GCodeCommand* cmd);