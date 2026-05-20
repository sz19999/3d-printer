
#include "gcode_lexer.h"
#include <stdio.h>

char* gcode_line = "G1 F523 X12 Y22.7 Z0 ; Move to (12,22.7,0)";

int main() { 
    const char *cursor = gcode_line; // read only
    GCodeToken currentToken;

    printf("\noriginal string: %s\n", cursor);

    while (*cursor != '\0' && *cursor != ';') {

        cursor = lexer_get_next_token(cursor, &currentToken);
        
        if (*cursor != ' ') { 

            printf("\ncurrent string: %s\n", cursor);
            printf("current token: %c\n", currentToken.letter);
            printf("current value: %.2f\n", currentToken.value);
    
            switch (currentToken.type) {
                case 0:
                    printf("current token type: TOKEN_VALID\n");
                    break;
                case 1:
                    printf("current token type: TOKEN_EOF\n");
                    break;
                case 2:
                    printf("current token type: TOKEN_ERROR\n");
                    break;
            }
        }
        
        for (int i = 0; i < 2000000000; i++);  // delay
    }

    printf("\nTest completed successfully!\n");

    return 0;
}