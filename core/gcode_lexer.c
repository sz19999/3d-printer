// an example for a gcode file:
// 
// init 3d printer:
// G10 ... ; (a comment) absolute position
// M108 S100 ; start heating hotend to 100C
// M200 S60 ; start heating bed to 60C
// ..
//
// printing starts here:
// G0 X55.0 Y0.0 Z0.0 ; move printer head to (55.0, 0.0, 0.0)
// G1 X60.0 Y10.0 Z0.0 F55 ; move in a line to (60, 10, 0) and extrude 55mm/s
// ...
// 
// reset printer and turn off:
// 
// RAmbo Jambo Alejandro
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "gcode_lexer.h"

const char* lexer_get_next_token(const char* cursor, GCodeToken* token) {  // pointer to a constant character
    // skip spaces
    while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
        return ++cursor;
    }
    
    // check if we hit the end of the line or a comment
    if (*cursor == '\0' || *cursor == '\n' || *cursor == '\r' || *cursor == ';') {
        token->type = TOKEN_EOF;
        token->letter = '\0';
        token->value = 0.0f;
        return cursor;
    }

    if  (*cursor >= 'a' && *cursor <= 'z') {  //convert to uppercase
        token->letter = toupper((unsigned char)*cursor);
        token->type = TOKEN_VALID;
    }
    else if((*cursor >= 'A' && *cursor <= 'Z')) {
        token->type = TOKEN_VALID;
    }
    else {
        token->type = TOKEN_ERROR;
    }

    token->letter = *cursor;
    cursor++;

    // extract value
    char* tmp_ptr;
    token->value = strtof(cursor, &tmp_ptr); // str to f -> string to float

    return tmp_ptr;
}
