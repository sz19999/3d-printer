#include <stdio.h>
#include <stdbool.h>
#include "gcode_lexer.h"
#include "gcode_parser.h"

static bool apply_token_to_command(const GCodeToken* tok, GCodeCommand* cmd);
bool parse_command(const char* gcode_line, GCodeCommand* cmd);

bool parse_command(const char* gcode_line, GCodeCommand* cmd) {
    
    const char* cursor = gcode_line;
    GCodeToken current_token;

  do {
    cursor = lexer_get_next_token(cursor , &current_token);                  // 1. extract a token
    bool applied_successfully = apply_token_to_command(&current_token, cmd); // 2. then parse the token

    if (!applied_successfully) {
        cmd->type = CMD_INVALID;
        return false;
    }
  } while (current_token.type != TOKEN_EOF);

  cmd->type = CMD_VALID;
  return true;
}

static bool apply_token_to_command(const GCodeToken* tok, GCodeCommand* cmd) {

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
            // handle invalid token letters
            if (tok->type != TOKEN_EOF) {
                return false;
            }
    }

    return true;
}