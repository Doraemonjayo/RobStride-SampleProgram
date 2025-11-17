#include "RobStride.h"
#include "string.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f

uint32_t Mailbox; // CAN Tx mailbox

/*******************************************************************************
* @Function     : Constructor for RobStride motor instance
* @Param        : CAN ID
* @Return       : void
* @Description  : Initialize motor ID.
*******************************************************************************/
void RobStride_Init(RobStride *robstride, uint8_t CAN_Id, bool MIT_mode, void (*canTxFunc)(uint32_t id, const uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote)) {
    robstride->CAN_ID = CAN_Id;
    robstride->Master_CAN_ID = 0xFD;
    robstride->Motor_Set_All.set_motor_mode = move_control_mode;
    robstride->MIT_Mode = MIT_mode;
    robstride->MIT_Type = operationControl;
    robstride->canTxFunc = canTxFunc;
}

/*******************************************************************************
* @Function     : Convert uint16_t to float
* @Param1       : Value to convert
* @Param2       : Minimum of x
* @Param3       : Maximum of x
* @Param4       : Bit width
* @Return       : Float in decimal
*******************************************************************************/
static float uint16_to_float(uint16_t x,float x_min,float x_max,int bits) {
    uint32_t span = (1 << bits) - 1;
    x &= span;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

/*******************************************************************************
* @Function     : Convert float to uint
* @Param1       : Value to convert
* @Param2       : Minimum of x
* @Param3       : Maximum of x
* @Param4       : Bit width
* @Return       : Integer (scaled)
*******************************************************************************/
static int float_to_uint(float x,float x_min,float x_max,int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}

/*******************************************************************************
* @Function     : Convert 8-byte array to float
* @Param        : Data array
* @Return       : Float decimal
*******************************************************************************/
float Byte_to_float(uint8_t* bytedata) {
    uint32_t data = bytedata[7]<<24 | bytedata[6]<<16 | bytedata[5]<<8 | bytedata[4];
    float data_float = *(float*)(&data);
    return data_float;
}

/*******************************************************************************
* @Function     : Convert MIT fault code to private fault code
* @Param        : MIT fault code (uint16)
* @Return       : Private fault code (uint8)
*******************************************************************************/
uint8_t mapFaults(uint16_t fault16) {
    uint8_t fault8 = 0;

    if (fault16 & (1 << 14)) fault8 |= (1 << 4); // Overload
    if (fault16 & (1 << 7))  fault8 |= (1 << 5); // Not calibrated
    if (fault16 & (1 << 3))  fault8 |= (1 << 3); // Encoder fault
    if (fault16 & (1 << 2))  fault8 |= (1 << 0); // Undervoltage
    if (fault16 & (1 << 1))  fault8 |= (1 << 1); // Driver fault
    if (fault16 & (1 << 0))  fault8 |= (1 << 2); // Over temperature

    return fault8;
}

/*******************************************************************************
* @Function     : Motor receive & parsing function
* @Param1       : Received data frame
* @Param2       : Received CAN extended ID
* @Return       : None
* @Description  : `drw` is valid only after communication 17 is sent
*******************************************************************************/
void RobStride_Motor_Analysis(RobStride *robstride, uint32_t id, uint8_t *data, uint8_t dlc, bool isExtended, bool isRemote)
{
	if (!isExtended) return;
	if (isRemote) return;
	if (dlc != 8) return;

    if(robstride->MIT_Mode)
    {
        if((id & 0xFF) == 0XFD)
        {
            // MIT Fault frame
            if(data[3] == 0x00 && data[4] == 0x00 && data[5] == 0x00 && data[6] == 0x00 && data[7] == 0x00)
            {
                uint16_t fault16 = 0;
                memcpy(&fault16, &data[1], 2);
                robstride->error_code = mapFaults(fault16);
            }
            else
            {
                // MIT Feedback frame
            	robstride->Pos_Info.Angle  = uint16_to_float((data[1]<<8)|(data[2]), P_MIN, P_MAX, 16);
            	robstride->Pos_Info.Speed  = uint16_to_float((data[3]<<4)|(data[4]>>4), V_MIN, V_MAX, 12);
            	robstride->Pos_Info.Torque = uint16_to_float((data[4]<<8)|(data[5]), T_MIN, T_MAX, 12);
            	robstride->Pos_Info.Temp   = ((data[6]<<8) | data[7]) * 0.1;
            }
        }
        else
        {
            memcpy(&robstride->Unique_ID, data, 8);
        }
    }
    else
    {
        if ((uint8_t)((id & 0xFF00) >> 8) == robstride->CAN_ID)
        {
            int type = (int)((id & 0x3F000000) >> 24);

            // Position feedback mode
            if (type == 2)
            {
            	robstride->Pos_Info.Angle  = uint16_to_float(data[0]<<8 | data[1], P_MIN, P_MAX, 16);
            	robstride->Pos_Info.Speed  = uint16_to_float(data[2]<<8 | data[3], V_MIN, V_MAX, 16);
            	robstride->Pos_Info.Torque = uint16_to_float(data[4]<<8 | data[5], T_MIN, T_MAX, 16);
            	robstride->Pos_Info.Temp   = (data[6]<<8 | data[7]) * 0.1;

            	robstride->error_code      = (uint8_t)((id & 0x3F0000) >> 16);
            	robstride->Pos_Info.pattern= (uint8_t)((id & 0xC00000) >> 22);
            }

            // Communication type 17 reply
            else if (type == 17)
            {
                for (int index_num = 0; index_num <= 13; index_num++)
                {
                    if ((data[1]<<8 | data[0]) == Index_List[index_num])
                        switch(index_num)
                        {
                            case 0:  robstride->drw.run_mode.data    = (uint8_t)(data[4]); break;
                            case 1:  robstride->drw.iq_ref.data      = Byte_to_float(data); break;
                            case 2:  robstride->drw.spd_ref.data     = Byte_to_float(data); break;
                            case 3:  robstride->drw.imit_torque.data = Byte_to_float(data); break;
                            case 4:  robstride->drw.cur_kp.data      = Byte_to_float(data); break;
                            case 5:  robstride->drw.cur_ki.data      = Byte_to_float(data); break;
                            case 6:  robstride->drw.cur_filt_gain.data = Byte_to_float(data); break;
                            case 7:  robstride->drw.loc_ref.data     = Byte_to_float(data); break;
                            case 8:  robstride->drw.limit_spd.data   = Byte_to_float(data); break;
                            case 9:  robstride->drw.limit_cur.data   = Byte_to_float(data); break;
                            case 10: robstride->drw.mechPos.data     = Byte_to_float(data); break;
                            case 11: robstride->drw.iqf.data         = Byte_to_float(data); break;
                            case 12: robstride->drw.mechVel.data     = Byte_to_float(data); break;
                            case 13: robstride->drw.VBUS.data        = Byte_to_float(data); break;
                        }
                }
            }

            // Communication type 0 reply (ID change response)
            else if ((uint8_t)((id & 0xFF)) == 0xFE)
            {
            	robstride->CAN_ID = (uint8_t)((id & 0xFF00)>>8);
                memcpy(&robstride->Unique_ID, data, 8);
            }
        }
    }
}


/*******************************************************************************
* @Function        : RobStride motor get device ID and MCU information (Type 0)
* @Param           : None
* @Return          : void
* @Description     : None
*******************************************************************************/
void RobStride_Get_CAN_ID(RobStride *robstride)
{
    uint8_t txdata[8] = {0};                        // TX data
    robstride->canTxFunc(Communication_Type_Get_ID<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);
}

/*******************************************************************************
* @Function        : RobStride motor motion control (Type 1)
* @Param1          : Torque (-4Nm ~ 4Nm)
* @Param2          : Target angle (-4π ~ 4π)
* @Param3          : Target speed (-30rad/s ~ 30rad/s)
* @Param4          : Kp (0.0 ~ 500.0)
* @Param5          : Kd (0.0 ~ 5.0)
* @Return          : void
* @Description     : None
*******************************************************************************/
void RobStride_Motor_move_control(RobStride *robstride, float Torque, float Angle, float Speed, float Kp, float Kd)
{
    uint8_t txdata[8] = {0};                        // TX data
    robstride->Motor_Set_All.set_Torque = Torque;
    robstride->Motor_Set_All.set_angle = Angle;
    robstride->Motor_Set_All.set_speed = Speed;
    robstride->Motor_Set_All.set_Kp = Kp;
    robstride->Motor_Set_All.set_Kd = Kd;

    if (robstride->drw.run_mode.data != 0)
    {
        Set_RobStride_Motor_parameter(robstride, 0X7005, move_control_mode, Set_mode); // Set motor mode
        Get_RobStride_Motor_parameter(robstride, 0x7005);
        Enable_Motor(robstride);
        robstride->Motor_Set_All.set_motor_mode = move_control_mode;
    }

    if (robstride->Pos_Info.pattern != 2)
    {
        Enable_Motor(robstride);
    }

    txdata[0] = float_to_uint(robstride->Motor_Set_All.set_angle, P_MIN, P_MAX, 16)>>8;
    txdata[1] = float_to_uint(robstride->Motor_Set_All.set_angle, P_MIN, P_MAX, 16);
    txdata[2] = float_to_uint(robstride->Motor_Set_All.set_speed, V_MIN, V_MAX, 16)>>8;
    txdata[3] = float_to_uint(robstride->Motor_Set_All.set_speed, V_MIN, V_MAX, 16);
    txdata[4] = float_to_uint(robstride->Motor_Set_All.set_Kp,   KP_MIN, KP_MAX, 16)>>8;
    txdata[5] = float_to_uint(robstride->Motor_Set_All.set_Kp,   KP_MIN, KP_MAX, 16);
    txdata[6] = float_to_uint(robstride->Motor_Set_All.set_Kd,   KD_MIN, KD_MAX, 16)>>8;
    txdata[7] = float_to_uint(robstride->Motor_Set_All.set_Kd,   KD_MIN, KD_MAX, 16);

    robstride->canTxFunc(Communication_Type_MotionControl<<24 | float_to_uint(robstride->Motor_Set_All.set_Torque, T_MIN, T_MAX, 16)<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);
}

/*******************************************************************************
* @Function        : Enable MIT mode
* @Param           : None
* @Return          : None
*******************************************************************************/
void RobStride_Motor_MIT_Enable(RobStride *robstride)
{
    uint8_t txdata[8] = {0};    // TX data array

    txdata[0] = 0xFF; txdata[1] = 0xFF; txdata[2] = 0xFF; txdata[3] = 0xFF;
    txdata[4] = 0xFF; txdata[5] = 0xFF; txdata[6] = 0xFF; txdata[7] = 0xFC;

    robstride->canTxFunc(robstride->CAN_ID,
    		txdata, 8, false, false);
}

/*******************************************************************************
* @Function        : Disable MIT mode
* @Param           : None
* @Return          : None
*******************************************************************************/
void RobStride_Motor_MIT_Disable(RobStride *robstride)
{
    uint8_t txdata[8] = {0};    // TX data array

    txdata[0] = 0xFF; txdata[1] = 0xFF; txdata[2] = 0xFF; txdata[3] = 0xFF;
    txdata[4] = 0xFF; txdata[5] = 0xFF; txdata[6] = 0xFF; txdata[7] = 0xFD;

    robstride->canTxFunc(robstride->CAN_ID,
    		txdata, 8, false, false);
}

/*******************************************************************************
* @Function        : MIT clear/check error
* @Param           : F_CMD (command code)
* @Return          : None
*******************************************************************************/
void RobStride_Motor_MIT_ClearOrCheckError(RobStride *robstride, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};    // TX data array

    txdata[0] = 0xFF; txdata[1] = 0xFF; txdata[2] = 0xFF; txdata[3] = 0xFF;
    txdata[4] = 0xFF; txdata[5] = 0xFF;
    txdata[6] = F_CMD;
    txdata[7] = 0xFB;

    robstride->canTxFunc(robstride->CAN_ID,
    		txdata, 8, false, false);
}

/*******************************************************************************
* @Function        : MIT set motor operation type
* @Param           : F_CMD
* @Return          : None
*******************************************************************************/
void RobStride_Motor_MIT_SetMotorType(RobStride *robstride, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};    // TX data array

    txdata[0] = 0xFF; txdata[1] = 0xFF; txdata[2] = 0xFF; txdata[3] = 0xFF;
    txdata[4] = 0xFF; txdata[5] = 0xFF;
    txdata[6] = F_CMD;
    txdata[7] = 0xFC;

    robstride->canTxFunc(robstride->CAN_ID,
    		txdata, 8, false, false);
}

/*******************************************************************************
* @Function        : MIT set motor ID
* @Param           : F_CMD
* @Return          : None
*******************************************************************************/
void RobStride_Motor_MIT_SetMotorId(RobStride *robstride, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};    // TX data array

    txdata[0] = 0xFF; txdata[1] = 0xFF; txdata[2] = 0xFF; txdata[3] = 0xFF;
    txdata[4] = 0xFF; txdata[5] = 0xFF;
    txdata[6] = F_CMD;
    txdata[7] = 0x01;

    robstride->canTxFunc(robstride->CAN_ID,
    		txdata, 8, false, false);
}

/*******************************************************************************
* @Function        : MIT control mode
* @Param1          : Angle
* @Param2          : Speed
* @Param3          : Kp
* @Param4          : Kd
* @Param5          : Torque
* @Return          : None
*******************************************************************************/
void RobStride_Motor_MIT_Control(RobStride *robstride, float Angle, float Speed, float Kp, float Kd, float Torque)
{
    uint8_t txdata[8] = {0};    // TX data array

    txdata[0] = float_to_uint(Angle, P_MIN, P_MAX, 16)>>8;
    txdata[1] = float_to_uint(Angle, P_MIN, P_MAX, 16);
    txdata[2] = float_to_uint(Speed, V_MIN, V_MAX, 12)>>4;
    txdata[3] = (float_to_uint(Speed, V_MIN, V_MAX, 12)<<4) | (float_to_uint(Kp, KP_MIN, KP_MAX, 12)>>8);
    txdata[4] = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
    txdata[5] = float_to_uint(Kd, KD_MIN, KD_MAX, 12)>>4;
    txdata[6] = (float_to_uint(Kd, KD_MIN, KD_MAX, 12)<<4) | (float_to_uint(Torque, T_MIN, T_MAX, 12)>>8);
    txdata[7] = float_to_uint(Torque, T_MIN, T_MAX, 12);

    robstride->canTxFunc(robstride->CAN_ID,
    		txdata, 8, false, false);
}

// MIT position control mode
void RobStride_Motor_MIT_PositionControl(RobStride *robstride, float position_rad, float speed_rad_per_s)
{
    uint8_t txdata[8] = {0};     // TX data buffer

    memcpy(&txdata[0], &position_rad, 4);      // Copy position (float)
    memcpy(&txdata[4], &speed_rad_per_s, 4);   // Copy velocity (float)

    robstride->canTxFunc((1 << 8) | robstride->CAN_ID,
    		txdata, 8, false, false);
}

// MIT speed control mode
void RobStride_Motor_MIT_SpeedControl(RobStride *robstride, float speed_rad_per_s, float current_limit)
{
    uint8_t txdata[8] = {0};     // TX data buffer

    memcpy(&txdata[0], &speed_rad_per_s, 4);
    memcpy(&txdata[4], &current_limit, 4);

    robstride->canTxFunc((2 << 8) | robstride->CAN_ID,
    		txdata, 8, false, false);
}

// MIT zero-position setting
void RobStride_Motor_MIT_SetZeroPos(RobStride *robstride)
{
    uint8_t txdata[8] = {0};     // TX data buffer

    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = 0xFF;
    txdata[7] = 0xFE;

    robstride->canTxFunc(robstride->CAN_ID,
    		txdata, 8, false, false);
}

/*******************************************************************************
* @Function     : RobStride motor — Position mode (PP interpolation mode)
* @Param1       : Target speed (-30rad/s ~ 30rad/s)
* @Param2       : Target angle (-4π ~ 4π)
* @Return       : void
*******************************************************************************/
void RobStride_Motor_Pos_control(RobStride *robstride, float Speed, float Angle)
{
	robstride->Motor_Set_All.set_speed = Speed;
	robstride->Motor_Set_All.set_angle = Angle;

    if (robstride->drw.run_mode.data != 1)
    {
        Set_RobStride_Motor_parameter(robstride, 0X7005, Pos_control_mode, Set_mode);
        Get_RobStride_Motor_parameter(robstride, 0x7005);
        robstride->Motor_Set_All.set_motor_mode = Pos_control_mode;
        Enable_Motor(robstride);
        Set_RobStride_Motor_parameter(robstride, 0X7024, robstride->Motor_Set_All.set_limit_speed, Set_parameter);
        Set_RobStride_Motor_parameter(robstride, 0X7025, robstride->Motor_Set_All.set_acceleration, Set_parameter);
    }
    Set_RobStride_Motor_parameter(robstride, 0X7016, robstride->Motor_Set_All.set_angle, Set_parameter);
}

/*******************************************************************************
* @Function     : RobStride motor — CSP mode (cyclic synchronous position)
* @Param1       : Target angle (-4π ~ 4π)
* @Param2       : Target velocity limit (0 ~ 44 rad/s)
* @Return       : void
*******************************************************************************/
void RobStride_Motor_CSP_control(RobStride *robstride, float Angle, float limit_spd)
{
    if (robstride->MIT_Mode) {
        RobStride_Motor_MIT_PositionControl(robstride, Angle, limit_spd);
    }
    else {
    	robstride->Motor_Set_All.set_angle = Angle;
    	robstride->Motor_Set_All.set_limit_speed = limit_spd;

        if (robstride->drw.run_mode.data != 1)
        {
            Set_RobStride_Motor_parameter(robstride, 0X7005, CSP_control_mode, Set_mode);
            Get_RobStride_Motor_parameter(robstride, 0x7005);
            Enable_Motor(robstride);
            Set_RobStride_Motor_parameter(robstride, 0X7017, robstride->Motor_Set_All.set_limit_speed, Set_parameter);
        }
        Set_RobStride_Motor_parameter(robstride, 0X7016, robstride->Motor_Set_All.set_angle, Set_parameter);
    }
}

/*******************************************************************************
* @Function     : RobStride motor — Speed mode
* @Param1       : Target speed (-30rad/s ~ 30rad/s)
* @Param2       : Current limit (0~23A)
* @Return       : void
*******************************************************************************/
uint8_t count_set_motor_mode_Speed = 0;
void RobStride_Motor_Speed_control(RobStride *robstride, float Speed, float limit_cur)
{
	robstride->Motor_Set_All.set_speed = Speed;
	robstride->Motor_Set_All.set_limit_cur = limit_cur;

    if (robstride->drw.run_mode.data != 2)
    {
        Set_RobStride_Motor_parameter(robstride, 0X7005, Speed_control_mode, Set_mode);
        Get_RobStride_Motor_parameter(robstride, 0x7005);
        Enable_Motor(robstride);
        robstride->Motor_Set_All.set_motor_mode = Speed_control_mode;
        Set_RobStride_Motor_parameter(robstride, 0X7018, robstride->Motor_Set_All.set_limit_cur, Set_parameter);
        Set_RobStride_Motor_parameter(robstride, 0X7022, 10, Set_parameter);
    }
    Set_RobStride_Motor_parameter(robstride, 0X700A, robstride->Motor_Set_All.set_speed, Set_parameter);
}

/*******************************************************************************
* @Function     : RobStride motor — Current mode
* @Param        : Target current (-23A ~ 23A)
* @Return       : void
*******************************************************************************/
uint8_t count_set_motor_mode = 0;
void RobStride_Motor_current_control(RobStride *robstride, float current)
{
	robstride->Motor_Set_All.set_current = current;
	robstride->output = robstride->Motor_Set_All.set_current;

    if (robstride->Motor_Set_All.set_motor_mode != 3)
    {
        Set_RobStride_Motor_parameter(robstride, 0X7005, Elect_control_mode, Set_mode);
        Get_RobStride_Motor_parameter(robstride, 0x7005);
        robstride->Motor_Set_All.set_motor_mode = Elect_control_mode;
        Enable_Motor(robstride);
    }

    Set_RobStride_Motor_parameter(robstride, 0X7006, robstride->Motor_Set_All.set_current, Set_parameter);
}

/*******************************************************************************
* @Function     : RobStride motor — Zero return mode (return to mechanical zero)
* @Param        : None
* @Return       : void
*******************************************************************************/
void RobStride_Motor_Set_Zero_control(RobStride *robstride)
{
    Set_RobStride_Motor_parameter(robstride, 0X7005, Set_Zero_mode, Set_mode);
}

/*******************************************************************************
* @Function     : Enable RobStride motor (Communication type 3)
* @Param        : None
* @Return       : void
*******************************************************************************/
void Enable_Motor(RobStride *robstride)
{
    if (robstride->MIT_Mode)
    {
        RobStride_Motor_MIT_Enable(robstride);
    }
    else
    {
        uint8_t txdata[8] = {0};                // TX data

        robstride->canTxFunc(Communication_Type_MotorEnable<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
        		txdata, 8, true, false);
    }
}

/*******************************************************************************
* @Function     : Disable RobStride motor (Communication type 4)
* @Param        : clear_error  (0: keep errors, 1: clear errors)
* @Return       : void
*******************************************************************************/
void Disenable_Motor(RobStride *robstride, uint8_t clear_error)
{
    if (robstride->MIT_Mode)
    {
        RobStride_Motor_MIT_Disable(robstride);
    }
    else
    {
        uint8_t txdata[8] = {0};                // TX data

        txdata[0] = clear_error;

        robstride->canTxFunc(Communication_Type_MotorStop<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
        		txdata, 8, true, false);

        Set_RobStride_Motor_parameter(robstride, 0X7005, move_control_mode, Set_mode);
    }
}

/*******************************************************************************
* @Function     : Write a single RobStride motor parameter (Communication type 18)
* @Param1       : Parameter index
* @Param2       : Parameter value
* @Param3       : Mode ('p' = parameter, 'j' = control mode)
* @Return       : void
*******************************************************************************/
void Set_RobStride_Motor_parameter(RobStride *robstride, uint16_t Index, float Value, char Value_mode)
{
    uint8_t txdata[8] = {0};                // TX data

    txdata[0] = Index;
    txdata[1] = Index >> 8;
    txdata[2] = 0x00;
    txdata[3] = 0x00;

    if (Value_mode == 'p')      // Write parameter
    {
        memcpy(&txdata[4], &Value, 4);
    }
    else if (Value_mode == 'j') // Write control mode
    {
    	robstride->Motor_Set_All.set_motor_mode = (int)(Value);
        txdata[4] = (uint8_t)Value;
        txdata[5] = 0x00;
        txdata[6] = 0x00;
        txdata[7] = 0x00;
    }

    robstride->canTxFunc(Communication_Type_SetSingleParameter<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);
}

/*******************************************************************************
* @Function     : Read a single RobStride motor parameter (Communication type 17)
* @Param        : Parameter index
* @Return       : void
*******************************************************************************/
void Get_RobStride_Motor_parameter(RobStride *robstride, uint16_t Index)
{
    uint8_t txdata[8] = {0};                // TX data

    txdata[0] = Index;
    txdata[1] = Index >> 8;

    robstride->canTxFunc(Communication_Type_GetSingleParameter<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);
}

/*******************************************************************************
* @Function     : Set RobStride motor CAN ID (Communication type 7)
* @Param        : New (preset) CAN ID
* @Return       : void
*******************************************************************************/
void Set_CAN_ID(RobStride *robstride, uint8_t Set_CAN_ID)
{
    Disenable_Motor(robstride, 0);

    uint8_t txdata[8] = {0};                // TX data

    robstride->canTxFunc(Communication_Type_Can_ID<<24 | Set_CAN_ID<<16 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);
}

/*******************************************************************************
* @Function     : Set mechanical zero position (Communication type 6)
* @Param        : None
* @Return       : void
* @Note         : Sets current motor position as mechanical zero.
*                 Motor will be disabled first, then enabled again.
*******************************************************************************/
void Set_ZeroPos(RobStride *robstride)
{
    Disenable_Motor(robstride, 0);                      // Disable motor

    uint8_t txdata[8] = {0};                // TX data

    txdata[0] = 1;

    robstride->canTxFunc(Communication_Type_SetPosZero<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);

    Enable_Motor(robstride);
}

/*******************************************************************************
* @Function     : Save motor parameters to flash (Comm Type 22)
* @Param        : None
* @Return       : void
* @Description  : Saves the current single-parameter table as default.
*                 After power reboot, parameters remain as those at the time
*                 this command was executed.
*******************************************************************************/
void RobStride_Motor_MotorDataSave(RobStride *robstride)
{
    uint8_t txdata[8] = {0};                // Data to send

    txdata[0] = 0x01;
    txdata[1] = 0x02;
    txdata[2] = 0x03;
    txdata[3] = 0x04;
    txdata[4] = 0x05;
    txdata[5] = 0x06;
    txdata[6] = 0x07;
    txdata[7] = 0x08;

    robstride->canTxFunc(Communication_Type_MotorStop<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);
}

/*******************************************************************************
* @Function     : Change RobStride motor baud rate (Comm Type 23)
* @Param        : Baudrate mode:
*                 01 (1M)
*                 02 (500K)
*                 03 (250K)
*                 04 (125K)
* @Return       : void
* @Description  : Sets the motor CAN baud rate to the specified mode.
*                 For example, parameter 01 sets baud rate to 1 Mbps.
*******************************************************************************/
void RobStride_Motor_BaudRateChange(RobStride *robstride, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};                // Data to send

    txdata[0] = 0x01;
    txdata[1] = 0x02;
    txdata[2] = 0x03;
    txdata[3] = 0x04;
    txdata[4] = 0x05;
    txdata[5] = 0x06;
    txdata[6] = F_CMD;
    txdata[7] = 0x08;   // This byte is irrelevant, can be any value

    robstride->canTxFunc(Communication_Type_BaudRateChange<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);
}

/*******************************************************************************
* @Function     : Enable or disable proactive motor reporting (Comm Type 24)
* @Param        : Reporting mode:
*                 00 (Disable)
*                 01 (Enable)
* @Return       : void
* @Description  : Enables or disables active motor reporting.
*                 Default reporting interval is 10 ms.
*******************************************************************************/
void RobStride_Motor_ProactiveEscalationSet(RobStride *robstride, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};                // Data to send

    txdata[0] = 0x01;
    txdata[1] = 0x02;
    txdata[2] = 0x03;
    txdata[3] = 0x04;
    txdata[4] = 0x05;
    txdata[5] = 0x06;
    txdata[6] = F_CMD;
    txdata[7] = 0x08;   // This byte is irrelevant, arbitrary value

    robstride->canTxFunc(Communication_Type_ProactiveEscalationSet<<24 | robstride->Master_CAN_ID<<8 | robstride->CAN_ID,
    		txdata, 8, true, false);
}

/*******************************************************************************
* @Function     : Set RobStride motor protocol mode (Comm Type 25)
* @Param        : Protocol type:
*                 00 (Private protocol)
*                 01 (CANopen)
*                 02 (MIT protocol)
* @Return       : void
* @Description  : None
*******************************************************************************/
void RobStride_Motor_MIT_MotorModeSet(RobStride *robstride, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};                // Data to send

    txdata[0] = 0xFF;
    txdata[1] = 0xFF;
    txdata[2] = 0xFF;
    txdata[3] = 0xFF;
    txdata[4] = 0xFF;
    txdata[5] = 0xFF;
    txdata[6] = F_CMD;
    txdata[7] = 0xFD;   // This byte is irrelevant, arbitrary

    robstride->canTxFunc(robstride->CAN_ID,
    		txdata, 8, false, false);
}

/*******************************************************************************
* @Function     : Initialize parameter index table for motor data
* @Param        : Parameter index list
* @Return       : void
* @Description  : Automatically called when the motor class is created.
*******************************************************************************/
void data_read_write_init(data_read_write *this, const uint16_t *index_list)
{
	this->run_mode.index = index_list[0];
	this->iq_ref.index = index_list[1];
	this->spd_ref.index = index_list[2];
	this->imit_torque.index = index_list[3];
	this->cur_kp.index = index_list[4];
	this->cur_ki.index = index_list[5];
    this->cur_filt_gain.index = index_list[6];
    this->loc_ref.index = index_list[7];
    this->limit_spd.index = index_list[8];
    this->limit_cur.index = index_list[9];
    this->mechPos.index = index_list[10];
    this->iqf.index = index_list[11];
    this->mechVel.index = index_list[12];
    this->VBUS.index = index_list[13];
    this->rotation.index = index_list[14];
}

/*******************************************************************************
* @Function     : Set motor operating mode (Comm Type XX)
* @Param        : Mode value
* @Return       : void
* @Description  : None
*******************************************************************************/
void RobStride_Motor_MotorModeSet(RobStride *robstride, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0};                // Data to send

    txdata[0] = 0x01;
    txdata[1] = 0x02;
    txdata[2] = 0x03;
    txdata[3] = 0x04;
    txdata[4] = 0x05;
    txdata[5] = 0x06;
    txdata[6] = F_CMD;
    txdata[7] = 0x08;   // Arbitrary value

    robstride->canTxFunc(Communication_Type_MotorModeSet<<24|robstride->Master_CAN_ID<<8|robstride->CAN_ID,
    		txdata, 8, true, false);
}
