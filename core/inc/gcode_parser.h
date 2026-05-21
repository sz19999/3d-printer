#pragma once

#include <stdbool.h>

typedef struct {
    // 1. Primary Command
    char command_letter;  // Usually 'G', 'M', or 'T'
    int command_number;   // e.g., 1 for G1, 104 for M104

    // 2. Presence Flags (Crucial for knowing if a value was updated)
    // In deeply embedded systems, you might compress this into a single uint16_t bitmask
    bool has_X : 1;
    bool has_Y : 1;
    bool has_Z : 1;
    bool has_E : 1; // Extruder
    bool has_F : 1; // Feedrate
    bool has_S : 1; // Spindle speed / Temperature
    bool has_P : 1; // Dwell time / Parameter

    // 3. Parameter Values
    float X;
    float Y;
    float Z;
    float E;
    float F;
    int S; 
    int P; 
} GCodeCommand;