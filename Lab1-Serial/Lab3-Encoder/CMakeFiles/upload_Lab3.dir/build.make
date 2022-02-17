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

# Utility rule file for upload_Lab3.

# Include the progress variables for this target.
include Lab3-Encoder/CMakeFiles/upload_Lab3.dir/progress.make

Lab3-Encoder/CMakeFiles/upload_Lab3: Lab3-Encoder/Lab3-atmega32u4.hex
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Desktop/MEGN540/Lab1-Serial/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Uploading Lab3-atmega32u4.hex to atmega32u4 using avr109"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab3-Encoder && avrdude -p atmega32u4 -c avr109 -U flash:w:Lab3-atmega32u4.hex -P /dev/ttyZumoCarAVR

Lab3-Encoder/Lab3-atmega32u4.hex: Lab3-Encoder/Lab3-atmega32u4.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Desktop/MEGN540/Lab1-Serial/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Lab3-atmega32u4.hex"
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab3-Encoder && /usr/bin/avr-objcopy -j .text -j .data -O ihex Lab3-atmega32u4.elf Lab3-atmega32u4.hex
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab3-Encoder && /usr/bin/avr-size -C --mcu=atmega32u4 Lab3-atmega32u4.elf

upload_Lab3: Lab3-Encoder/CMakeFiles/upload_Lab3
upload_Lab3: Lab3-Encoder/Lab3-atmega32u4.hex
upload_Lab3: Lab3-Encoder/CMakeFiles/upload_Lab3.dir/build.make

.PHONY : upload_Lab3

# Rule to build all files generated by this target.
Lab3-Encoder/CMakeFiles/upload_Lab3.dir/build: upload_Lab3

.PHONY : Lab3-Encoder/CMakeFiles/upload_Lab3.dir/build

Lab3-Encoder/CMakeFiles/upload_Lab3.dir/clean:
	cd /home/pi/Desktop/MEGN540/Lab1-Serial/Lab3-Encoder && $(CMAKE_COMMAND) -P CMakeFiles/upload_Lab3.dir/cmake_clean.cmake
.PHONY : Lab3-Encoder/CMakeFiles/upload_Lab3.dir/clean

Lab3-Encoder/CMakeFiles/upload_Lab3.dir/depend:
	cd /home/pi/Desktop/MEGN540/Lab1-Serial && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Desktop/MEGN540 /home/pi/Desktop/MEGN540/Lab3-Encoder /home/pi/Desktop/MEGN540/Lab1-Serial /home/pi/Desktop/MEGN540/Lab1-Serial/Lab3-Encoder /home/pi/Desktop/MEGN540/Lab1-Serial/Lab3-Encoder/CMakeFiles/upload_Lab3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Lab3-Encoder/CMakeFiles/upload_Lab3.dir/depend
