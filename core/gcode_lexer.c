
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

    if  ((*cursor >= 'a' && *cursor <= 'z') || (*cursor >= 'A' && *cursor <= 'Z')) {
        token->letter = toupper((unsigned char)*cursor); //convert to uppercase
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
