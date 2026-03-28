# AI Coding Agent Instructions for Crazy Circuit Robot Project

## Project Overview
This is an STM32F103RCT6-based autonomous robot for track following in the "Crazy Circuit" competition. The system uses light sensors, IMU, encoders, and PID control for navigation.

## Architecture
- **Main Control Loop**: Timer interrupt (TIM6, ~10ms) handles sensor reading and control updates
- **Sensor Fusion**: AD7689 ADC for 16 light sensors, ICM20602 IMU, dual encoders
- **Control System**: PID-based speed/direction control with multiple running modes
- **Track Navigation**: Predefined track structures with mileage-based waypoint navigation
- **User Interface**: OLED display with keyboard for parameter tuning

## Key Files & Modules
- `Core/Src/main.c`: Main program with initialization and control loop
- `Code/control.c`: Core control logic, error calculation, mode switching
- `Code/track.c`: Predefined track layouts and navigation data
- `Code/pid.c`: PID controller implementations
- `Code/AD7689.c`: Light sensor ADC interface
- `Code/icm20602.c`: IMU sensor interface
- `Code/encoder.c`: Motor encoder reading
- `Code/OLED/`: Display and keyboard interface system

## Control Flow
1. **Initialization**: Peripherals (SPI, I2C, TIM, UART) and sensors
2. **Calibration Phase**: First 1500 timer cycles for threshold auto-adjustment
3. **Main Loop**: `Car_Go()` called every timer cycle:
   - `Get_Speed()`: Encoder-based velocity measurement with filtering
   - `Get_IMU()`: Orientation data from ICM20602
   - `Get_Error()`: Track processing and error calculation
   - `Set_Out()`: PID control output to motors

## Track Processing
- `Image_Process()`: Converts 16-element light sensor array to track edge detection
- Error calculation uses weighted position array: `{-32,-26,-17,-14,-10,-5,-2,-1,1,2,5,10,14,17,26,32}`
- Supports multiple modes: Normal tracking, turns, straight sections, mileage-based navigation

## Running Modes
- `Normal_Mode`: Standard line following
- `Turn_Left/Right`: Dedicated turn handling
- `Straight_Mode`: Straight line optimization
- `Mileage_Mode`: Predefined path following with distance-based waypoints

## Development Workflow
- **Configuration**: Use STM32CubeMX (.ioc file) for pin/peripheral changes
- **Build**: Keil uVision (MDK-ARM) or VS Code with STM32 extension
- **Debugging**: UART output via `VOFA_JustFloat()` for real-time plotting
- **Parameter Tuning**: OLED interface for runtime PID and threshold adjustment
- **Flashing**: Standard STM32 programming (JTAG/SWD)

## Code Patterns
- **Sensor Filtering**: Exponential moving average (0.5/0.3/0.2 weights) for encoder data
- **Timer Callbacks**: `HAL_TIM_PeriodElapsedCallback()` for control loop
- **State Machines**: Mode switching based on track conditions and mileage
- **Data Structures**: `Racing_track_Typedef` for track definitions with nodes/elements
- **HAL Usage**: Standard STM32 HAL library for all peripherals

## Common Tasks
- **Adding Sensors**: Follow AD7689/icm20602 pattern: init function, read function, data filtering
- **New Control Modes**: Add to `Run_Mode` enum and switch statement in `Get_Error()`
- **Parameter Tuning**: Add to OLED display system and `Oled_Data_Load()`
- **Track Modification**: Update `Racing_track_Typedef` structures in `track.c`

## Debugging
- Serial output at 115200 baud for sensor data monitoring
- OLED display shows real-time parameters and status
- LED indicators for mode/status feedback
- Breakpoints in timer callbacks for control loop inspection</content>
<parameter name="filePath">c:\system\Desktop\Winter_camp_courses\Crazy_Circuit_mileage1.1\2026_Crazy_Circuit_Example\.github\copilot-instructions.md