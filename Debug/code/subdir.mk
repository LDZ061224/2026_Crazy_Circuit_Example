################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/Ctrl.c \
../code/Fun.c \
../code/Racing_Track.c \
../code/TCA9555.c \
../code/pid.c 

COMPILED_SRCS += \
code/Ctrl.src \
code/Fun.src \
code/Racing_Track.src \
code/TCA9555.src \
code/pid.src 

C_DEPS += \
code/Ctrl.d \
code/Fun.d \
code/Racing_Track.d \
code/TCA9555.d \
code/pid.d 

OBJS += \
code/Ctrl.o \
code/Fun.o \
code/Racing_Track.o \
code/TCA9555.o \
code/pid.o 


# Each subdirectory must supply rules for building sources it contributes
code/Ctrl.src: ../code/Ctrl.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Ctrl.o: code/Ctrl.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Fun.src: ../code/Fun.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Fun.o: code/Fun.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Racing_Track.src: ../code/Racing_Track.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Racing_Track.o: code/Racing_Track.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/TCA9555.src: ../code/TCA9555.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/TCA9555.o: code/TCA9555.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/pid.src: ../code/pid.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/pid.o: code/pid.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/Ctrl.d code/Ctrl.o code/Ctrl.src code/Fun.d code/Fun.o code/Fun.src code/Racing_Track.d code/Racing_Track.o code/Racing_Track.src code/TCA9555.d code/TCA9555.o code/TCA9555.src code/pid.d code/pid.o code/pid.src

.PHONY: clean-code

