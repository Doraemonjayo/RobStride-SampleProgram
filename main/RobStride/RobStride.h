#ifndef __RobStride_H__
#define __RobStride_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"

// Command types
#define Set_mode          'j'        // Set control mode
#define Set_parameter     'p'        // Set parameter

// Motion control modes
#define move_control_mode   0  // Torque mode
#define Pos_control_mode    1  // Position mode (PP)
#define Speed_control_mode  2  // Speed mode
#define Elect_control_mode  3  // Current mode
#define Set_Zero_mode       4  // Zero position mode
#define CSP_control_mode    5  // CSP position mode

// Communication types
#define Communication_Type_Get_ID               0x00   // Read unique 64-bit MCU ID
#define Communication_Type_MotionControl        0x01   // Torque mode / basic motion command
#define Communication_Type_MotorRequest         0x02   // Request motor status
#define Communication_Type_MotorEnable          0x03   // Enable motor
#define Communication_Type_MotorStop            0x04   // Stop motor
#define Communication_Type_SetPosZero           0x06   // Set zero mechanical position
#define Communication_Type_Can_ID               0x07   // Read current CAN ID
#define Communication_Type_Control_Mode         0x12   // Set control mode
#define Communication_Type_GetSingleParameter   0x11   // Read single parameter
#define Communication_Type_SetSingleParameter   0x12   // Set single parameter
#define Communication_Type_ErrorFeedback        0x15   // Error feedback frame
// Note: When using this mode, ensure firmware version >= 0.13.0
#define Communication_Type_MotorDataSave        0x16   // Save parameter to flash
#define Communication_Type_BaudRateChange       0x17   // Change baudrate (takes effect after reset)
#define Communication_Type_ProactiveEscalationSet 0x18 // Proactive reporting configuration
#define Communication_Type_MotorModeSet         0x19   // Motor mode configuration frame (takes effect after reset)

typedef struct {
	uint16_t index;
	float data;
} data_read_write_one;

// List of 18 writable parameters
static const uint16_t Index_List[] = {
    0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011,
    0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A,
    0x701B, 0x701C, 0x701D
};

// Read/write parameter list
// Parameter             Index         Type      Size     Unit/Description
typedef struct
{
	data_read_write_one run_mode;        // 0: torque mode, 1: position mode, 2: speed mode,
										 // 3: current mode, 4: zero mode (uint8, 1 byte)

	data_read_write_one iq_ref;          // Current mode Iq reference     (float, 4 bytes, -23~23 A)
	data_read_write_one spd_ref;         // Speed mode target speed       (float, 4 bytes, -30~30 rad/s)
	data_read_write_one imit_torque;     // Torque limit                  (float, 4 bytes, 0~12 Nm)
	data_read_write_one cur_kp;          // Current loop Kp               (float, default 0.125)
	data_read_write_one cur_ki;          // Current loop Ki               (float, default 0.0158)
	data_read_write_one cur_filt_gain;   // Current loop filter gain      (float, default 0.1)
	data_read_write_one loc_ref;         // Position mode reference       (float, rad)
	data_read_write_one limit_spd;       // Speed limit in position mode  (float, 0~30 rad/s)
	data_read_write_one limit_cur;       // Current limit                 (float, 0~23 A)

	// Read-only parameters
	data_read_write_one mechPos;         // Mechanical absolute angle     (float, rad)
	data_read_write_one iqf;             // Filtered iq                   (float, -23~23 A)
	data_read_write_one mechVel;         // Mechanical angular velocity   (float, -30~30 rad/s)
	data_read_write_one VBUS;            // Bus voltage                   (float, V)
	data_read_write_one rotation;        // Number of turns               (int16, 2 bytes)
} data_read_write;

void data_read_write_init(data_read_write *this, const uint16_t *index_list /* = Index_List */);

typedef struct {
    float Angle;
    float Speed;
    float Torque;
    float Temp;
    int pattern;   // Feedback mode: 0-position, 1-calibration, 2-debug
} Motor_Pos_RobStride_Info;

typedef struct {
    int set_motor_mode;
    float set_current;
    float set_speed;
    float set_acceleration;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_limit_speed;
    float set_Kp;
    float set_Ki;
    float set_Kd;
} Motor_Set;

typedef enum
{
    operationControl = 0,
    positionControl  = 1,
    speedControl     = 2
} MIT_TYPE;

// RobStride motor class
typedef struct {
    uint8_t CAN_ID;               // CAN ID (default 127 / 0x7F, view via command 1)
    uint64_t Unique_ID;           // Unique 64-bit MCU ID
    uint16_t Master_CAN_ID;       // Master ID (default 0x1F on startup)

    Motor_Set Motor_Set_All;      // Motor set values
    uint8_t error_code;

    bool MIT_Mode;                // MIT mode flag
    MIT_TYPE MIT_Type;            // MIT control type

    float output;
    int Can_Motor;
    Motor_Pos_RobStride_Info Pos_Info;     // Feedback values
    data_read_write drw;                   // Parameter read/write
} RobStride;

void RobStride_Set_MIT_Mode(RobStride *robstride, bool MIT_Mode);
void RobStride_Set_MIT_Type(RobStride *robstride, MIT_TYPE MIT_Type);

void RobStride_Init(RobStride *robstride, uint8_t CAN_Id, bool MIT_Mode);

void RobStride_Get_CAN_ID(RobStride *robstride);
void Set_RobStride_Motor_parameter(RobStride *robstride, uint16_t Index, float Value, char Value_mode);
void Get_RobStride_Motor_parameter(RobStride *robstride, uint16_t Index);
void RobStride_Motor_Analysis(RobStride *robstride, uint8_t *DataFrame, uint32_t ID_ExtId);

void RobStride_Motor_move_control(RobStride *robstride, float Torque, float Angle, float Speed, float Kp, float Kd);
void RobStride_Motor_Pos_control(RobStride *robstride, float Speed, float Angle);
void RobStride_Motor_CSP_control(RobStride *robstride, float Angle, float limit_spd);
void RobStride_Motor_Speed_control(RobStride *robstride, float Speed, float limit_cur);
void RobStride_Motor_current_control(RobStride *robstride, float current);
void RobStride_Motor_Set_Zero_control(RobStride *robstride);
void RobStride_Motor_MotorModeSet(RobStride *robstride, uint8_t F_CMD);
void Enable_Motor(RobStride *robstride);
void Disenable_Motor(RobStride *robstride, uint8_t clear_error);
void Set_CAN_ID(RobStride *robstride, uint8_t Set_CAN_ID);
void Set_ZeroPos(RobStride *robstride);

bool RobStride_Get_MIT_Mode(RobStride *robstride);
MIT_TYPE RobStride_get_MIT_Type(RobStride *robstride);

void RobStride_Motor_MIT_Control(RobStride *robstride, float Angle, float Speed, float Kp, float Kd, float Torque);
void RobStride_Motor_MIT_PositionControl(RobStride *robstride, float position_rad, float speed_rad_per_s);
void RobStride_Motor_MIT_SpeedControl(RobStride *robstride, float speed_rad_per_s, float current_limit);
void RobStride_Motor_MIT_Enable(RobStride *robstride);
void RobStride_Motor_MIT_Disable(RobStride *robstride);
void RobStride_Motor_MIT_SetZeroPos(RobStride *robstride);
void RobStride_Motor_MIT_ClearOrCheckError(RobStride *robstride, uint8_t F_CMD);
void RobStride_Motor_MIT_SetMotorType(RobStride *robstride, uint8_t F_CMD);
void RobStride_Motor_MIT_SetMotorId(RobStride *robstride, uint8_t F_CMD);
void RobStride_Motor_MotorDataSave(RobStride *robstride);
void RobStride_Motor_BaudRateChange(RobStride *robstride, uint8_t F_CMD);
void RobStride_Motor_ProactiveEscalationSet(RobStride *robstride, uint8_t F_CMD);
void RobStride_Motor_MIT_MotorModeSet(RobStride *robstride, uint8_t F_CMD);

#ifdef __cplusplus
 }
#endif

#endif
