/*
 * SSET_02_GSLV_Compression_System
 * STM32H743ZI2 Firmware - 14 Axis Synchronized Stepper Compression
 * Real-time force feedback, PID, FreeRTOS, Safety Critical
 *
 * Hardware Mapping (from spec):
 *   - TIM1, TIM8, TIM2, TIM4 for STEP generation (DMA capable)
 *   - GPIO for DIR/EN (14 axes)
 *   - Bit-banged or SPI HX711 for 14 load cells
 *   - I2C1/I2C2 + TCA9548A for IMUs & HX711 if needed
 *   - EXTI for Deadman / E-Stop
 *   - SSR + Clutch control GPIOs
 *
 * CubeMX Configuration Required:
 *   - System Clock: 480 MHz
 *   - FreeRTOS CMSIS_V2
 *   - TIM1, TIM8, TIM2, TIM4 in PWM Output mode (for step frequency)
 *   - Multiple I2C, GPIO, EXTI
 *   - DMA for high-frequency step generation if needed
 */

#include "main.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ====================== DEFINES ======================
#define NUM_AXES                14
#define CONTROL_LOOP_HZ         500
#define SAFETY_LOOP_HZ          1000

#define MAX_STEP_FREQ_HZ        200000UL   // Safe upper limit
#define BODY_MASS_KG_DEFAULT    75.0f

#define G_SAFETY_THRESHOLD      8.0f
#define FORCE_TARGET_MAX_N      3400.0f

#define HX711_CLK_DELAY_US      2

// Axis Groups
#define GROUP_TORSO             0  // 0-5
#define GROUP_ARMS              1  // 6-11
#define GROUP_LEGS              2  // 12-13

// ====================== TYPEDEFS ======================
typedef enum {
    SYS_IDLE = 0,
    SYS_ARMED,
    SYS_RAMPING,
    SYS_HOLDING,
    SYS_RELEASING,
    SYS_EMERGENCY,
    SYS_LOCKOUT
} SystemState_t;

typedef enum {
    TRAJ_LINEAR = 0,
    TRAJ_CUBIC,
    TRAJ_EXPONENTIAL
} TrajectoryType_t;

typedef struct {
    float duration_s;
    float start_g;
    float end_g;
    TrajectoryType_t curve;
} LaunchPhase_t;

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output;
} PID_t;

typedef struct {
    uint8_t axis_id;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef* en_port;
    uint16_t en_pin;
    TIM_HandleTypeDef* step_timer;
    uint32_t step_channel;
    float target_force_N;
    float current_force_N;
    PID_t pid;
} Axis_t;

// ====================== GLOBALS ======================
SystemState_t system_state = SYS_IDLE;
float body_mass_kg = BODY_MASS_KG_DEFAULT;
float current_g_equivalent = 0.0f;

Axis_t axes[NUM_AXES];

float axis_force_N[NUM_AXES] = {0};
float total_force_N = 0.0f;

volatile uint8_t deadman1_active = 0;
volatile uint8_t deadman2_active = 0;
volatile uint8_t estop_active = 0;
volatile uint8_t safety_flag = 0;

// Phase Profile (from spec)
const LaunchPhase_t launch_profile[] = {
    {5.0f, 0.0f, 1.5f, TRAJ_CUBIC},
    {8.0f, 1.5f, 3.5f, TRAJ_CUBIC},
    {6.0f, 3.5f, 7.0f, TRAJ_EXPONENTIAL},
    {4.0f, 7.0f, 5.0f, TRAJ_LINEAR},
    {3.0f, 5.0f, 0.5f, TRAJ_CUBIC},
    {0.0f, 0.0f, 0.0f, TRAJ_LINEAR}
};

uint8_t current_phase = 0;
float phase_elapsed = 0.0f;

// ====================== PID ======================
void PID_Init(PID_t* pid, float kp, float ki, float kd) {
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->integral = 0.0f; pid->prev_error = 0.0f;
}

float PID_Compute(PID_t* pid, float error, float dt) {
    pid->integral += error * dt;
    pid->integral = fmaxf(fminf(pid->integral, 500.0f), -500.0f);
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}

// ====================== AXIS CONTROL ======================
void Axis_SetStepFrequency(uint8_t axis, uint32_t freq_hz) {
    if (axis >= NUM_AXES || freq_hz == 0) {
        HAL_TIM_PWM_Stop(axes[axis].step_timer, axes[axis].step_channel);
        return;
    }
    uint32_t arr = (HAL_RCC_GetPCLK2Freq() / 2) / freq_hz - 1; // Adjust based on timer clock
    __HAL_TIM_SET_AUTORELOAD(axes[axis].step_timer, arr);
    __HAL_TIM_SET_COMPARE(axes[axis].step_timer, axes[axis].step_channel, arr/2);
    HAL_TIM_PWM_Start(axes[axis].step_timer, axes[axis].step_channel);
}

void Axis_SetDirection(uint8_t axis, uint8_t tighten) {
    HAL_GPIO_WritePin(axes[axis].dir_port, axes[axis].dir_pin,
                     tighten ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Axis_Enable(uint8_t axis, uint8_t enable) {
    HAL_GPIO_WritePin(axes[axis].en_port, axes[axis].en_pin,
                     enable ? GPIO_PIN_RESET : GPIO_PIN_SET); // Active LOW
}

// ====================== HX711 LOAD CELL ======================
uint32_t HX711_Read(uint8_t channel) {
    // Bit-bang implementation - map channel to GPIO CLK/DATA pins
    // (Implement full 24-bit read with channel selection via TCA9548A if used)
    // Placeholder
    return 0x800000; // Mid-scale example
}

void Update_All_Load_Cells(void) {
    total_force_N = 0.0f;
    for (int i = 0; i < NUM_AXES; i++) {
        uint32_t raw = HX711_Read(i);
        axis_force_N[i] = (raw - 0x800000) * 0.001f; // Calibrate properly
        total_force_N += axis_force_N[i];
    }
    current_g_equivalent = total_force_N / (body_mass_kg * 9.81f);
}

// ====================== SAFETY ======================
void TriggerFullEmergencyRelease(void) {
    system_state = SYS_EMERGENCY;
    HAL_GPIO_WritePin(SSR_MASTER_GPIO_Port, SSR_MASTER_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CLUTCH_ALL_GPIO_Port, CLUTCH_ALL_Pin, GPIO_PIN_RESET);
    
    for (int i = 0; i < NUM_AXES; i++) {
        Axis_SetStepFrequency(i, 0);
        Axis_Enable(i, 0);
    }
    safety_flag = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == DEADMAN1_Pin || GPIO_Pin == DEADMAN2_Pin || GPIO_Pin == ESTOP_Pin) {
        if (!deadman1_active || !deadman2_active || estop_active) {
            TriggerFullEmergencyRelease();
        }
    }
}

// ====================== TASKS (FreeRTOS) ======================
osThreadId_t safetyTaskHandle, motionTaskHandle, sequencerTaskHandle;

// Safety Monitor - 1kHz
void SafetyMonitorTask(void *argument) {
    for (;;) {
        Update_All_Load_Cells();

        if (current_g_equivalent >= G_SAFETY_THRESHOLD) {
            TriggerFullEmergencyRelease();
        }

        // Asymmetry check
        float mean = total_force_N / NUM_AXES;
        for (int i = 0; i < NUM_AXES; i++) {
            if (fabsf(axis_force_N[i] - mean) / mean > 0.20f) {
                // Partial release logic for pair
            }
        }

        if (!deadman1_active || !deadman2_active) {
            TriggerFullEmergencyRelease();
        }

        osDelay(1);
    }
}

// Motion Control - 500Hz
void MotionControlTask(void *argument) {
    for (;;) {
        if (system_state != SYS_HOLDING && system_state != SYS_RAMPING) {
            osDelay(2);
            continue;
        }

        float target_g = /* get from sequencer */;
        float target_force_per_axis = (target_g * body_mass_kg * 9.81f) / NUM_AXES;

        for (int i = 0; i < NUM_AXES; i++) {
            float error = target_force_per_axis - axis_force_N[i];
            float pid_out = PID_Compute(&axes[i].pid, error, 0.002f);

            uint8_t tighten = (pid_out > 0);
            Axis_SetDirection(i, tighten);

            uint32_t step_freq = (uint32_t)fabsf(pid_out) * 500; // Scale appropriately
            step_freq = (step_freq > MAX_STEP_FREQ_HZ) ? MAX_STEP_FREQ_HZ : step_freq;
            Axis_SetStepFrequency(i, step_freq);
        }
        osDelay(2);
    }
}

// Phase Sequencer
void PhaseSequencerTask(void *argument) {
    for (;;) {
        if (system_state == SYS_RAMPING || system_state == SYS_HOLDING) {
            // Advance phase, compute current target G with trajectory
            // Update phase_elapsed etc.
        }
        osDelay(50);
    }
}

// ====================== INITIALIZATION ======================
void Init_Axes(void) {
    // Populate axes[] with correct GPIO / Timer mappings from spec
    // Example for axis 0:
    // axes[0].step_timer = &htim1;
    // axes[0].step_channel = TIM_CHANNEL_1;
    // axes[0].dir_port = DIR0_GPIO_Port; etc.

    for (int i = 0; i < NUM_AXES; i++) {
        PID_Init(&axes[i].pid, 0.8f, 0.3f, 0.05f); // Tune these
        Axis_Enable(i, 0);
    }
}

void MX_FREERTOS_Init(void) {
    osThreadAttr_t attributes = {0};
    attributes.priority = osPriorityHigh;

    attributes.name = "Safety";
    safetyTaskHandle = osThreadNew(SafetyMonitorTask, NULL, &attributes);

    attributes.name = "Motion";
    attributes.priority = osPriorityAboveNormal;
    motionTaskHandle = osThreadNew(MotionControlTask, NULL, &attributes);

    attributes.name = "Sequencer";
    attributes.priority = osPriorityNormal;
    sequencerTaskHandle = osThreadNew(PhaseSequencerTask, NULL, &attributes);
}

// ====================== MAIN ======================
int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM8_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();

    // Start timers in PWM mode for step generation
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); etc. for all used channels

    Init_Axes();
    MX_FREERTOS_Init();

    // Initial state
    system_state = SYS_IDLE;

    // Enable safety interrupts
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0); // Highest priority

    osKernelStart();  // Start FreeRTOS

    while (1) {
        // Background tasks, telemetry, SSET03 sync, etc.
    }
}
