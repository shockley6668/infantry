//
// Created by xhuanc on 2021/10/13.
//

#ifndef _GIMBAL_H_
#define _GIMBAL_H_
/*      Include     */

#include "can_receive.h"
#include "PID.h"
#include "remote.h"
#include "AHRS.h"
#include "cmsis_os.h"
#include "launcher.h"
#include "can_receive.h"
#include "user_lib.h"
#include "protocol_shaob.h"
#include "filter.h"
#include "Detection.h"
#include "packet.h"
/*      define     */

#define GIMBAL_TASK_INIT_TIME 3000
#define YAW_CHANNEL 2
#define PITCH_CHANNEL 3

#define RC_TO_YAW 0.001f
#define RC_TO_PITCH 0.0008f
//最大最小绝对相对角度
#define MAX_ABS_ANGLE (30)
#define MIN_ABS_ANGLE (-30)
#define MAX_RELA_ANGLE (35)
#define MIN_RELA_ANGLE (-20)
//云台转动速度系数
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f//0.5f
#define GIMBAL_RC_MOVE_RATIO_YAW 0.8f//0.8f
#define GIMBAL_YAW_PATROL_SPEED 0.08f



#define GIMBAL_PITCH_PATROL_SPEED 0.06f

#define GIMBAL_YAW_ANGLE_PID_KP     15.f//4.8f
#define GIMBAL_YAW_ANGLE_PID_KI     0.01f//0.0018f
#define GIMBAL_YAW_ANGLE_PID_KD     800.0f//300.f
#define GIMBAL_YAW_ANGLE_MAX_OUT    10000.f
#define GIMBAL_YAW_ANGLE_MAX_IOUT   60.f

#define GIMBAL_YAW_SPEED_PID_KP     180.f
#define GIMBAL_YAW_SPEED_PID_KI     20.5f//50.0f
#define GIMBAL_YAW_SPEED_PID_KD     80.0f//300.f
#define GIMBAL_YAW_SPEED_MAX_OUT    30000.f
#define GIMBAL_YAW_SPEED_MAX_IOUT   3000.f

//
//#define GIMBAL_YAW_SPEED_PID_KP     225.f
//#define GIMBAL_YAW_SPEED_PID_KI     30.0f
//#define GIMBAL_YAW_SPEED_PID_KD     120.0f
//#define GIMBAL_YAW_SPEED_MAX_OUT    30000.0f
//#define GIMBAL_YAW_SPEED_MAX_IOUT   1000.0f

//#define GIMBAL_PITCH_ANGLE_PID_KP   45.f
//#define GIMBAL_PITCH_ANGLE_PID_KI   1.5f
//#define GIMBAL_PITCH_ANGLE_PID_KD   500.f//300.f
//#define GIMBAL_PITCH_ANGLE_MAX_OUT  360.f
//#define GIMBAL_PITCH_ANGLE_MAX_IOUT 10.f
//
//#define GIMBAL_PITCH_SPEED_PID_KP   200.f//300.f
//#define GIMBAL_PITCH_SPEED_PID_KI   1.5f//0.6f
//#define GIMBAL_PITCH_SPEED_PID_KD   0.0f
//#define GIMBAL_PITCH_SPEED_MAX_OUT  25000.f
//#define GIMBAL_PITCH_SPEED_MAX_IOUT 1000.f

//#define GIMBAL_PITCH_ANGLE_PID_KP   20.f
//#define GIMBAL_PITCH_ANGLE_PID_KI   1.5f
//#define GIMBAL_PITCH_ANGLE_PID_KD   500.f//300.f
//#define GIMBAL_PITCH_ANGLE_MAX_OUT  360.f
//#define GIMBAL_PITCH_ANGLE_MAX_IOUT 10.f
//
//#define GIMBAL_PITCH_SPEED_PID_KP   200.f//300.f
//#define GIMBAL_PITCH_SPEED_PID_KI   1.5f//0.6f
//#define GIMBAL_PITCH_SPEED_PID_KD   0.0f
//#define GIMBAL_PITCH_SPEED_MAX_OUT  25000.f
//#define GIMBAL_PITCH_SPEED_MAX_IOUT 1000.f

#define GIMBAL_PITCH_ANGLE_PID_KP   20.f
#define GIMBAL_PITCH_ANGLE_PID_KI   0.15f
#define GIMBAL_PITCH_ANGLE_PID_KD   300.f//300.f
#define GIMBAL_PITCH_ANGLE_MAX_OUT  360.f
#define GIMBAL_PITCH_ANGLE_MAX_IOUT 30.f

#define GIMBAL_PITCH_SPEED_PID_KP   200.f//300.f
#define GIMBAL_PITCH_SPEED_PID_KI   20.5f//0.6f
#define GIMBAL_PITCH_SPEED_PID_KD   80.0f
#define GIMBAL_PITCH_SPEED_MAX_OUT  25000.f
#define GIMBAL_PITCH_SPEED_MAX_IOUT 1000.f

/*      结构体和枚举     */

typedef enum {
    GIMBAL_RELAX=0,//云台失能
    GIMBAL_BACK,//云台回中
    GIMBAL_ACTIVE,
    GIMBAL_AUTO,
    GIMBAL_BUFF, //大符
    GIMBAL_SBUFF, //小符
}gimbal_mode_e;

typedef struct {
    motor_6020_t yaw;
    motor_6020_t pitch;
    motor_2006_t trigger;
    gimbal_mode_e mode;
    gimbal_mode_e last_mode;

//    AHRS_Eulr_t*Eulr;   //姿态角

    fp32 relative_gyro_yaw;
    fp32 absolute_gyro_yaw;
    bool_t yaw_is_back;
    fp32 relative_gyro_pitch;
    fp32 absolute_gyro_pitch;
    bool_t pitch_is_back;
    int32_t yaw_imu_offset_angle;
    float horizon_angle;

}gimbal_t;

_Noreturn extern void gimbal_task(void const*pvParameters);


#endif
