# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Desktop/MEGN540

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/MEGN540/Lab1-Serial

# Include any dependencies generated for this target.
include Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/depend.make

# Include the progress variables for this target.
include Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/progress.make

# Include the compile flags for this target's objects.
include Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/flags.make

Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.obj: Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/flags.make
Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.obj: ../Lab0-Blink/Lab0-blink.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/MEGN540/Lab1-Serial/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.obj"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink && /usr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.obj   -c /home/pi/Desktop/MEGN540/Lab0-Blink/Lab0-blink.c

Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.i"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink && /usr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/Desktop/MEGN540/Lab0-Blink/Lab0-blink.c > CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.i

Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.s"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink && /usr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/Desktop/MEGN540/Lab0-Blink/Lab0-blink.c -o CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.s

Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.obj: Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/flags.make
Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.obj: ../Lab0-Blink/led_interface.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/MEGN540/Lab1-Serial/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.obj"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink && /usr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.obj   -c /home/pi/Desktop/MEGN540/Lab0-Blink/led_interface.c

Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.i"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink && /usr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/Desktop/MEGN540/Lab0-Blink/led_interface.c > CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.i

Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.s"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink && /usr/bin/avr-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/Desktop/MEGN540/Lab0-Blink/led_interface.c -o CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.s

# Object files for target Lab0-atmega32u4.elf
Lab0__atmega32u4_elf_OBJECTS = \
"CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.obj" \
"CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.obj"

# External object files for target Lab0-atmega32u4.elf
Lab0__atmega32u4_elf_EXTERNAL_OBJECTS =

Lab0-Blink/Lab0-atmega32u4.elf: Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/Lab0-blink.c.obj
Lab0-Blink/Lab0-atmega32u4.elf: Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/led_interface.c.obj
Lab0-Blink/Lab0-atmega32u4.elf: Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/build.make
Lab0-Blink/Lab0-atmega32u4.elf: libLUFA_USB-atmega32u4.a
Lab0-Blink/Lab0-atmega32u4.elf: libMEGN540-atmega32u4.a
Lab0-Blink/Lab0-atmega32u4.elf: libLUFA_USB-atmega32u4.a
Lab0-Blink/Lab0-atmega32u4.elf: Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Desktop/MEGN540/Lab1-Serial/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable Lab0-atmega32u4.elf"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Lab0-atmega32u4.elf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/build: Lab0-Blink/Lab0-atmega32u4.elf

.PHONY : Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/build

Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/clean:
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink && $(CMAKE_COMMAND) -P CMakeFiles/Lab0-atmega32u4.elf.dir/cmake_clean.cmake
.PHONY : Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/clean

Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/depend:
	cd /home/pi/Desktop/MEGN540/Lab1-Serial && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Desktop/MEGN540 /home/pi/Desktop/MEGN540/Lab0-Blink /home/pi/Desktop/MEGN540/Lab1-Serial /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink /home/pi/Desktop/MEGN540/Lab1-Serial/Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Lab0-Blink/CMakeFiles/Lab0-atmega32u4.elf.dir/depend
