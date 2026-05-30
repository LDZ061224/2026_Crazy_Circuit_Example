################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/OLED/FlashFun.c \
../code/OLED/Font.c \
../code/OLED/JBD_simiic.c \
../code/OLED/OLEDKeyboard.c \
../code/OLED/UI.c \
../code/OLED/dev_CH455.c \
../code/OLED/dev_ssd1306.c 

COMPILED_SRCS += \
code/OLED/FlashFun.src \
code/OLED/Font.src \
code/OLED/JBD_simiic.src \
code/OLED/OLEDKeyboard.src \
code/OLED/UI.src \
code/OLED/dev_CH455.src \
code/OLED/dev_ssd1306.src 

C_DEPS += \
code/OLED/FlashFun.d \
code/OLED/Font.d \
code/OLED/JBD_simiic.d \
code/OLED/OLEDKeyboard.d \
code/OLED/UI.d \
code/OLED/dev_CH455.d \
code/OLED/dev_ssd1306.d 

OBJS += \
code/OLED/FlashFun.o \
code/OLED/Font.o \
code/OLED/JBD_simiic.o \
code/OLED/OLEDKeyboard.o \
code/OLED/UI.o \
code/OLED/dev_CH455.o \
code/OLED/dev_ssd1306.o 


# Each subdirectory must supply rules for building sources it contributes
code/OLED/FlashFun.src: ../code/OLED/FlashFun.c code/OLED/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/OLED/FlashFun.o: code/OLED/FlashFun.src code/OLED/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/OLED/Font.src: ../code/OLED/Font.c code/OLED/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/OLED/Font.o: code/OLED/Font.src code/OLED/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/OLED/JBD_simiic.src: ../code/OLED/JBD_simiic.c code/OLED/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/OLED/JBD_simiic.o: code/OLED/JBD_simiic.src code/OLED/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/OLED/OLEDKeyboard.src: ../code/OLED/OLEDKeyboard.c code/OLED/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/OLED/OLEDKeyboard.o: code/OLED/OLEDKeyboard.src code/OLED/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/OLED/UI.src: ../code/OLED/UI.c code/OLED/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/OLED/UI.o: code/OLED/UI.src code/OLED/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/OLED/dev_CH455.src: ../code/OLED/dev_CH455.c code/OLED/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/OLED/dev_CH455.o: code/OLED/dev_CH455.src code/OLED/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/OLED/dev_ssd1306.src: ../code/OLED/dev_ssd1306.c code/OLED/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/workspace/2026_Crazy_Circuit_V4.0/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
code/OLED/dev_ssd1306.o: code/OLED/dev_ssd1306.src code/OLED/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-OLED

clean-code-2f-OLED:
	-$(RM) code/OLED/FlashFun.d code/OLED/FlashFun.o code/OLED/FlashFun.src code/OLED/Font.d code/OLED/Font.o code/OLED/Font.src code/OLED/JBD_simiic.d code/OLED/JBD_simiic.o code/OLED/JBD_simiic.src code/OLED/OLEDKeyboard.d code/OLED/OLEDKeyboard.o code/OLED/OLEDKeyboard.src code/OLED/UI.d code/OLED/UI.o code/OLED/UI.src code/OLED/dev_CH455.d code/OLED/dev_CH455.o code/OLED/dev_CH455.src code/OLED/dev_ssd1306.d code/OLED/dev_ssd1306.o code/OLED/dev_ssd1306.src

.PHONY: clean-code-2f-OLED

