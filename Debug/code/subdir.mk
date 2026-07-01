################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/Ctrl.c \
../code/Debug_Car.c \
../code/Fun.c \
../code/Racing_Track.c \
../code/Uart_Adjust.c \
../code/WS2812.c \
../code/pid.c 

COMPILED_SRCS += \
code/Ctrl.src \
code/Debug_Car.src \
code/Fun.src \
code/Racing_Track.src \
code/Uart_Adjust.src \
code/WS2812.src \
code/pid.src 

C_DEPS += \
code/Ctrl.d \
code/Debug_Car.d \
code/Fun.d \
code/Racing_Track.d \
code/Uart_Adjust.d \
code/WS2812.d \
code/pid.d 

OBJS += \
code/Ctrl.o \
code/Debug_Car.o \
code/Fun.o \
code/Racing_Track.o \
code/Uart_Adjust.o \
code/WS2812.o \
code/pid.o 


# Each subdirectory must supply rules for building sources it contributes
code/Ctrl.src: ../code/Ctrl.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Ctrl.o: code/Ctrl.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Debug_Car.src: ../code/Debug_Car.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Debug_Car.o: code/Debug_Car.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Fun.src: ../code/Fun.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Fun.o: code/Fun.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Racing_Track.src: ../code/Racing_Track.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Racing_Track.o: code/Racing_Track.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/Uart_Adjust.src: ../code/Uart_Adjust.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/Uart_Adjust.o: code/Uart_Adjust.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/WS2812.src: ../code/WS2812.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/WS2812.o: code/WS2812.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/pid.src: ../code/pid.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/pid.o: code/pid.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/Ctrl.d code/Ctrl.o code/Ctrl.src code/Debug_Car.d code/Debug_Car.o code/Debug_Car.src code/Fun.d code/Fun.o code/Fun.src code/Racing_Track.d code/Racing_Track.o code/Racing_Track.src code/Uart_Adjust.d code/Uart_Adjust.o code/Uart_Adjust.src code/WS2812.d code/WS2812.o code/WS2812.src code/pid.d code/pid.o code/pid.src

.PHONY: clean-code

