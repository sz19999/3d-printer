#ifndef CONFIG_H
#define CONFIG_H

#define MICROSTEPS     16                    
#define PULLEY_TEETH   20                    
#define BELT_PITCH     2                     
#define STEP_ANGLE_DEG 1.8  
#define SCREW_LEAD     8
#define EFF_EX_GEAR_D  11   // need to verify the extruder gear diameter   
#define PI             3.14159265f

#define STEPS_PER_MM_BELT  ((float)((360  * MICROSTEPS) / (STEP_ANGLE_DEG * PULLEY_TEETH * BELT_PITCH)))
#define STEPS_PER_MM_SCREW ((float)((360  * MICROSTEPS) / (STEP_ANGLE_DEG * SCREW_LEAD)))
#define STEPS_PER_MM_GEAR  ((float)((360 * MICROSTEPS) / (EFF_EX_GEAR_D * PI)))

#define MAX_VELOCITY_X  100.0f / 60.0f  // 100 mm/s
#define MAX_VELOCITY_Y  100.0f / 60.0f
#define MAX_VELOCITY_Z  100.0f / 60.0f
#define MAX_VELOCITY_E  100.0f / 60.0f

#define MAX_ACCELERATION_X  100 // need to verify each acceleration constant and velocity
#define MAX_ACCELERATION_Y  100
#define MAX_ACCELERATION_Z  100
#define MAX_ACCELERATION_E  100

#define JUNCTION_DEVIATION  0.02f // 0.02mm - need to test

#endif