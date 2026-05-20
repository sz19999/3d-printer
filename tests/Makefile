
# SYSTEM CONFIGURATION
CC = gcc
CFLAGS = -Wall -Wextra -g -I../core/inc


# This is where all compiler output will go
BUILD_DIR = build


# Executable outputs
LEXER_TARGET = $(BUILD_DIR)/run_lexer_test.exe
PARSER_TARGET = $(BUILD_DIR)/run_parser_test.exe


# DEFAULT RULE: shows what units are available for test
all:
	@echo ERROR: Please specify which test module you want to run!
	@echo Available test modules:
	@echo   make lexer    - Runs isolated G-Code Lexer tests
	@echo   make parser   - Runs isolated G-Code Parser tests
	@echo   make clean    - Wipes out test executables


# link ONLY the lexer files
lexer: $(LEXER_TARGET)
	@echo    RUNNING ISOLATED LEXER TEST
	@.\$(LEXER_TARGET)

$(LEXER_TARGET): $(BUILD_DIR)/lexer_test.o $(BUILD_DIR)/gcode_lexer.o
	$(CC) $(CFLAGS) $^ -o $@


# parser isn't ready to be tested yet, when ready update rule target to "parser"
# link ONLY the parser files
parser.disabled: $(PARSER_TARGET)
	@echo    RUNNING PARSER TEST
	@.\$(PARSER_TARGET)

$(PARSER_TARGET): $(BUILD_DIR)/parser_test.o $(BUILD_DIR)/gcode_parser.o $(BUILD_DIR)/gcode_lexer.o
	$(CC) $(CFLAGS) $^ -o $@


# PATTERN RULES (The Compilation Step to create .o files)
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: ../core/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@


# make build/ dir if it doesn't exist
$(BUILD_DIR):
	@if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)


# CLEAN UP: Just type 'make clean' to wipe out executables
clean:
	@echo Cleaning up build artifacts...
	@if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)