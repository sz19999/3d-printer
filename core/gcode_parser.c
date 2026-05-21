#include <stdio.h>
#include "gcode_lexer.h"
#include "gcode_parser.h"
#include <stdbool.h>
//
void apply_token_to_command(const GCodeToken* tok, GCodeCommand* cmd);
//
GCodeCommand cmd;
GCodeToken token;
char gcode_line[] = "G1 F523 X12 Y22.7 Z0 ; Move to (12,22.7,0)";
char* cursor = gcode_line;
//
void parsCommand (GCodeCommand cmd){

  while(token.type != TOKEN_EOF){
    cursor = lexer_get_next_token(cursor , &token); // step 1 - extracting a token
    apply_token_to_command(&token, &cmd);
  }

}


// Assuming your Lexer outputs a Token struct like this:
// { char letter; float value; }
void apply_token_to_command(const GCodeToken* tok, GCodeCommand* cmd) {
    switch (tok->letter) {
        case 'G':
        case 'M':
        case 'T':
            cmd->command_letter = tok->letter;
            cmd->command_number = (int)tok->value;
            break;
        case 'X':
            cmd->has_X = true;
            cmd->X = tok->value;
            break;
        case 'Y':
            cmd->has_Y = true;
            cmd->Y = tok->value;
            break;
        case 'Z':
            cmd->has_Z = true;
            cmd->Z = tok->value;
            break;
        case 'E':
            cmd->has_E = true;
            cmd->E = tok->value;
            break;
        case 'F':
            cmd->has_F = true;
            cmd->F = tok->value;
            break;
        case 'S':
            cmd->has_S = true;
            cmd->S = tok->value;
            break;
        case 'P':
            cmd->has_P = true;
            cmd->P = tok->value;
            break;
        default:
            // Handle invalid token letters
            break;
    }
}