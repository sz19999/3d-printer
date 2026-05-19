#ifndef GCODE_LEXER_H
#define GCODE_LEXER_H

typedef enum {
    TOKEN_VALID,   // Successfully found a Letter-Number pair (e.g., X10.5)
    TOKEN_EOF,     // Reached the end of the line or a comment (;)
    TOKEN_ERROR    // Found a syntax error or a garbage character
} TokenType;

typedef struct {
    TokenType type; // What kind of token is this? (Valid, End-of-line, Error)
    char letter;    // Holds the command axis or parameter letter ('G', 'M', 'X', 'Y', etc.)
    float value;    // Holds the numeric value attached to that letter (e.g., 1.0, -50.25)
} GCodeToken;


const char* lexer_get_next_token(const char* cursor, GCodeToken* token);

#endif