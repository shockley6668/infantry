//
// Created by xhuanc on 2021/10/13.
//

/*  Include */
#include <filter.h>
#include "Gimbal.h"
#include "cmsis_os.h"
#include "launcher.h"
//#include "can_receive.h"
#include "user_lib.h"
#include "Atti.h"
//#include "bsp_laser.h"
#include "Auto.h"
#include "key_board.h"
#include "Referee.h"
#include "Detection.h"
#include "bsp_servo_pwm.h"
#include "bsp_led.h"
/*      define      */

/*      变量      */
gimbal_t gimbal;
uint32_t chassis_move_time;
uint32_t magazine_cover_time;
int16_t y_cnt=0;
fp32 gyro_yaw=0.0f;
fp32 gyro_pitch=0.0f;
extern int32_t total_ecd_ref;
/*     结构体      */
extern RC_ctrl_t rc_ctrl;
extern led_t led;
extern launcher_t launcher;
extern Eulr_t Eulr;
extern key_board_t KeyBoard;
extern osThreadId ChassisTaskHandle;
extern robot_ctrl_info_t robot_ctrl;
extern vision_t vision_data;
extern fp32 INS_angle[3];
extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];
/*      滤波      */
first_order_filter_type_t pitch_first_order_set;
first_order_filter_type_t pitch_current_first_order_set;
first_order_filter_type_t filter_yaw_gyro_in;
first_order_filter_type_t filter_pitch_gyro_in;
first_order_filter_type_t mouse_in_y;
first_order_filter_type_t mouse_in_x;
first_order_filter_type_t auto_pitch;
first_order_filter_type_t auto_yaw;
first_kalman_filter_t filter_autoYaw;
moving_Average_Filter MF_auto_yaw={.length=3};//yaw??????????
moving_Average_Filter MF_auto_pitch={.length=3};
second_lowPass_filter pitch_current_out;
second_lowPass_filter pitch_speed_out;


/*      函数及声明   */
static void gimbal_init();
static void gimbal_mode_set();
static void gimbal_back_handle();
static void gimbal_active_handle();
static void gimbal_relax_handle();
static void gimbal_ctrl_loop_cal();
static void gimbal_man_ctrl_loop_cal();
static void gimbal_angle_update();
static void gimbal_auto_handle();
static void pit_offset_get();
static void gimbal_device_offline_handle();
static void gimbal_turn_back_judge();
static void magazine_cover_control();
static void gimbal_uiInfo_packet();
static void gimbal_power_stop();
static void gimbal_mode_change();
static void gimbal_can_send_back_mapping();



void gimbal_task(void const*pvParameters)
{
    //任务初始化时间
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //云台初始化
    gimbal_init();////需要编写

    //发射机构初始化
    launcher_init();

    vTaskResume(ChassisTaskHandle);

    while(1){
//        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误

        gimbal_angle_update();////更新绝对、相对角度接收值，需要编写

        gimbal_mode_set();////根据遥控器设置云台控制模式，需要编写

        launcher_mode_set();//发射模式设置

        switch (gimbal.mode) {

            case GIMBAL_RELAX://云台失能
                gimbal_relax_handle();////需要编写
                break;

            case GIMBAL_BACK://云台回中
                gimbal_back_handle();////需要编写
                break;

            case GIMBAL_ACTIVE://云台控制
                gimbal_active_handle();  ////得到遥控器对云台电机的控制，需要编写
                gimbal_ctrl_loop_cal();////需要编写
                break;

            case GIMBAL_AUTO:
            case GIMBAL_BUFF:
            case GIMBAL_SBUFF:
            {
                gimbal_auto_handle();
                gimbal_ctrl_loop_cal();////需要编写
            }
                break;
        }

        magazine_cover_control();//弹仓盖开合控制

        launcher_control();//发射机构控制
        gimbal_device_offline_handle();//检测离线

        gimbal_can_send_back_mapping();

//        xTaskResumeAll();
        vTaskDelay(1);
    }
}

pid_t auto_yaw_angle_pid;
pid_t auto_yaw_speed_pid;
static void gimbal_init(){
    //云台相关电机初始化
    //1.更新云台电机yaw轴的数据还有pitch轴的数据

    //云台模式初始化，默认模式为失能


    //yaw轴电机 角度环和速度环PID初始化
   //下面请写PID角度环初始化


    //下面请写PID速度环初始化


    //pit轴电机 角度环和速度环PID初始化
    //下面写法与yaw的类似





//    pid_init(&auto_yaw_angle_pid, GIMBAL_AUTO_YAW_ANGLE_MAX_OUT,
//             GIMBAL_AUTO_YAW_ANGLE_MAX_IOUT,
//             GIMBAL_AUTO_YAW_ANGLE_PID_KP,
//             GIMBAL_AUTO_YAW_ANGLE_PID_KI
//             , GIMBAL_AUTO_YAW_ANGLE_PID_KD);
//
//
//    pid_init(&auto_yaw_speed_pid, GIMBAL_AUTO_YAW_SPEED_MAX_OUT,
//             GIMBAL_AUTO_YAW_SPEED_MAX_IOUT,
//             GIMBAL_AUTO_YAW_SPEED_PID_KP,
//             GIMBAL_AUTO_YAW_SPEED_PID_KI
//            , GIMBAL_AUTO_YAW_SPEED_PID_KD);

    //滤波器初始化，包含一节低通滤波和卡尔曼滤波
    first_order_filter_init(&pitch_first_order_set, 0.f, 500);
    first_order_filter_init(&pitch_current_first_order_set, 5, 30);
    first_order_filter_init(&filter_yaw_gyro_in, 5, 30);
    first_order_filter_init(&mouse_in_x, 1, 40);
    first_order_filter_init(&mouse_in_y, 0.5, 20);
    first_order_filter_init(&filter_pitch_gyro_in, 1, 20);
    first_order_filter_init(&auto_pitch, 1, 30);
    first_order_filter_init(&auto_yaw, 1, 30);

    //卡尔曼滤波
    first_Kalman_Create(&filter_autoYaw,1,20);

    SetCutoffFreq(&pitch_current_out,500,188);
    SetCutoffFreq(&pitch_speed_out,500,188 );

    //初始化时 云台设为未回中状态,在gimbal结构体里可以找到



    //上电时默认先设置成失能模式，再切换到当前遥控设置模式，提示：last_mode



    chassis_move_time=0;
    magazine_cover_time=0;
    //yaw轴和pitch轴电机的校准编码值
    //offset_ecd是用于角度环计算用的，这里yaw的初始值设为7492，pitch的初始值设为3196



}

//云台模式设置（获取遥控器信息，判断模式）
static void gimbal_mode_set(){

    //根据遥控器设置云台模式

    //使用switch语句实现，以右手的键位来判断
    //当键位是下拉位时，为云台失能状态
    //       中间位和上拉位是使能状态
    //提示：每次模式更换都要记录好上一次的状态
    //注意：当从失能转为使能时要进行云台回中yaw和pitch回中,最后还要加上gimbal_mode_change()就是下面那个函数
    switch (rc_ctrl.rc.s[RC_s_R]) {

        case RC_SW_DOWN:
        {

        }

        case RC_SW_MID:
        case RC_SW_UP:
        {

        }

        default:{
            break;
        }
    }


}

static void gimbal_mode_change() {
    if (gimbal.mode == GIMBAL_ACTIVE) {   //自瞄判定
        if ((KeyBoard.Mouse_r.status == KEY_PRESS) || (rc_ctrl.rc.ch[4] > 300)) {
            vision_data.mode = 0x21;
        } else {
            vision_data.mode = 0;
        }
        if ((KeyBoard.Mouse_r.status == KEY_PRESS && robot_ctrl.target_lock == 0x31 &&
            (detect_list[DETECT_AUTO_AIM].status == ONLINE)) ||
            (rc_ctrl.rc.ch[4]>300 && robot_ctrl.target_lock == 0x31 &&
            (detect_list[DETECT_AUTO_AIM].status == ONLINE))) {
            gimbal.last_mode = GIMBAL_ACTIVE;
            gimbal.mode = GIMBAL_AUTO;
        }
//            robot_ctrl.target_2   lock=0x32;

        if (KeyBoard.C.status == KEY_PRESS && robot_ctrl.target_lock == 0x31 &&
            (detect_list[DETECT_AUTO_AIM].status == ONLINE)) {
            gimbal.last_mode = GIMBAL_ACTIVE;
            gimbal.mode = GIMBAL_BUFF;
        }

        if (KeyBoard.X.status == KEY_PRESS && robot_ctrl.target_lock == 0x31 &&
            (detect_list[DETECT_AUTO_AIM].status == ONLINE)) {
            gimbal.last_mode = GIMBAL_ACTIVE;
            gimbal.mode = GIMBAL_SBUFF;
        }

    } else if (gimbal.mode == GIMBAL_AUTO) {   //自瞄失效判定                                                //0x32表示自瞄数据无效
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) || robot_ctrl.target_lock == 0x32) {
            gimbal.last_mode = GIMBAL_AUTO;
            gimbal.mode = GIMBAL_ACTIVE;//默认回到一般模式
            vision_data.mode = 0;
        }
    } else if (gimbal.mode == GIMBAL_BUFF) {
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) ||
            !(KeyBoard.C.status == KEY_PRESS && robot_ctrl.target_lock == 0x31)) {
            gimbal.last_mode = GIMBAL_BUFF;
            gimbal.mode = GIMBAL_ACTIVE;
        }
    } else if (gimbal.mode == GIMBAL_SBUFF) {
        if ((detect_list[DETECT_AUTO_AIM].status == OFFLINE) ||
            !(KeyBoard.X.status == KEY_PRESS && robot_ctrl.target_lock == 0x31)) {
            gimbal.last_mode = GIMBAL_SBUFF;
            gimbal.mode = GIMBAL_ACTIVE;
        }
    }
}

    //UI更新---云台模式

static void gimbal_can_send_back_mapping(){
    int16_t *real_motor_give_current[4];

    real_motor_give_current[0] = &launcher.fire_l.give_current;
    real_motor_give_current[1] = &gimbal.pitch.give_current;
    real_motor_give_current[2] = &launcher.fire_r.give_current;
    real_motor_give_current[3] = &launcher.trigger.give_current;
    //CAN1发送的是云台yaw轴的电流
    CAN_cmd_motor(CAN_1,
                  CAN_MOTOR_0x1FF_ID,
                  gimbal.yaw.give_current,
                  0,
                  0,
                  0);

//    CAN_cmd_motor(CAN_1,
//                  CAN_MOTOR_0x1FF_ID,
//                  0,
//                  0,
//                  0,
//                  0);
    //CAN2发送的是发射机构左右摩擦轮电流和拨弹机电流，还有一个云台pitch轴电流
    CAN_cmd_motor(CAN_2,
                  CAN_MOTOR_0x1FF_ID,
                  launcher.fire_r.give_current,
                  gimbal.pitch.give_current,
                  launcher.fire_l.give_current,
                  launcher.trigger.give_current);

//    CAN_cmd_motor(CAN_2,
//                  CAN_MOTOR_0x1FF_ID,
//                  0,
//                  gimbal.pitch.give_current,
//                  0,
//                  launcher.trigger.give_current);
}

//回中处理函数（判断云台是否回中，将标识符置1）
static void gimbal_back_handle(){
    //首先是yaw轴的云台回中，根据相对角度判断，当相对角度的绝对值大于2时，将标志xx_is_back设为0，表示还未回中，否则设为1



    //pitch轴的也是一样，当时当相对角度绝对值大于2°是标志为0，否则……
    //但是原代码在否则那里的处理是直接在里面写了回中的算法，思路是：先通过相对角0度回中，再将绝对角设为0度准备进行控制
    //然后对pitch回中时进行动态限位，防止坡上复活出现超过限位的情况，限位的函数是fp32_constrain();
    ////else{
        //先通过相对角0度回中，再将绝对角设为0度准备进行控制
        //就是把绝对角度和相对角度都设为0



        //对pitch回中时进行动态限位，防止坡上复活出现超过限位的情况


//        gimbal.pitch.absolute_angle_set=fp32_constrain(gimbal.pitch.absolute_angle_set,
//                                                       gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get,
//                                                       gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get);

////  }
    //在没有回中时，每次执行函数都进行闭环控制回中
    //这里我们先进行pitch轴的位置环控制
    //用PID算法进行位置环控制，先将设定pitch轴的速度值gyro_set用角度换算出来



    //这里过滤后就再次转换，速度换电流
    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);




    //在回中完成后yaw轴按照当前相对角为0度为初始角度控制
    gimbal.yaw.absolute_angle_set=gimbal.yaw.absolute_angle_get;
    total_ecd_ref=launcher.trigger.motor_measure->total_ecd;
}
//使能模式
static void gimbal_active_handle(){
    //在yaw期望值上按遥控器进行增减

    //鼠标输入滤波
    first_order_filter_cali(&mouse_in_x,rc_ctrl.mouse.x);
    //云台期望设置
    //期望值用的是绝对角度，赋值还是遥控器值，顺便加上鼠标值，提示：角度 = 原角度 - 遥控器值，鼠标值则是加上（还要乘上转换的系数）



    //一键掉头判断
    gimbal_turn_back_judge();

    //鼠标输入滤波
    first_order_filter_cali(&mouse_in_y,rc_ctrl.mouse.y);
    //云台期望设置
    //在pitch上同yaw一样，提示：pitch的是加上，鼠标值则是减去



//    VAL_LIMIT(gimbal.pitch.absolute_angle_set,MAX_RELA_ANGLE,MIN_RELA_ANGLE);

    //云台绕圈时进行绝对角循环设置
    if(gimbal.yaw.absolute_angle_set>=180){
        gimbal.yaw.absolute_angle_set-=360;
    }
    else if(gimbal.yaw.absolute_angle_set<=-180){
        gimbal.yaw.absolute_angle_set+=360;
    }


    //对pit期望值进行动态限幅（通过陀螺仪和编码器得到动态的限位）
    gimbal.pitch.absolute_angle_set=fp32_constrain(gimbal.pitch.absolute_angle_set,
                                                   MIN_RELA_ANGLE+
                                                   gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get,
                                                   MAX_RELA_ANGLE+
                                                   gimbal.pitch.absolute_angle_get-gimbal.pitch.relative_angle_get);

}
fp32 radio_y=0.0098f;
fp32 radio_p=0.01f;
fp32 auto_yaw_add;
int16_t p_cnt=0;

static void gimbal_auto_handle(){

//    first_Kalman_Filter(&filter_autoYaw,Vision_yaw_add);
//    //获取视觉发送的角度误差
//    if(!isnan(MF_auto_yaw.aver_num))
//        gimbal.yaw.absolute_angle_set=MF_auto_yaw.aver_num;
//    else
//        average_clear(&MF_auto_yaw);
//    if(!isnan(MF_auto_pitch.aver_num))
//        gimbal.pitch.absolute_angle_set=MF_auto_pitch.aver_num;
//    else
//        average_clear(&MF_auto_pitch);

    //获取视觉发送的角度误差
//    average_add(&MF_auto_yaw,robot_ctrl.yaw);
//    average_add(&MF_auto_yaw,robot_ctrl.pitch);

    //这里就是自动瞄准的功能了
    first_order_filter_cali(&auto_pitch, robot_ctrl.pitch);
    first_order_filter_cali(&auto_yaw, robot_ctrl.yaw);
    //我们以F键为触发键
    if(KeyBoard.F.click_flag == 0) {
        if(robot_ctrl.yaw >178 || robot_ctrl.yaw < -178) {
            gimbal.yaw.absolute_angle_set = robot_ctrl.yaw;
        }
        else{
            gimbal.yaw.absolute_angle_set = auto_yaw.out;
        }
    }
    else{
        //鼠标输入滤波
        first_order_filter_cali(&mouse_in_x,rc_ctrl.mouse.x);
        //云台期望设置
        gimbal.yaw.absolute_angle_set-=rc_ctrl.rc.ch[YAW_CHANNEL]*RC_TO_YAW*GIMBAL_RC_MOVE_RATIO_YAW
                                       +mouse_in_x.out*MOUSE_X_RADIO;
    }

        gimbal.pitch.absolute_angle_set=auto_pitch.out;

//    gimbal.yaw.absolute_angle_set=robot_ctrl.yaw;
//
//    gimbal.pitch.absolute_angle_set=robot_ctrl.pitch;

    //云台绕圈时进行绝对角循环设置
    if(gimbal.yaw.absolute_angle_set>=180){
        gimbal.yaw.absolute_angle_set-=360;
    }
    else if(gimbal.yaw.absolute_angle_set<=-180){
        gimbal.yaw.absolute_angle_set+=360;
    }
}
//失能模式处理（两轴电流为0）
static void gimbal_relax_handle(){
    //yaw和pitch的电流设为0



    //TODO: 拨弹电机电流 gimbal.trigger.give_current=0;
}

//云台上相关电机离线 则给电机0电流
static void gimbal_device_offline_handle() {
    if(detect_list[DETECT_REMOTE].status == OFFLINE){
        gimbal.pitch.give_current = 0;
        gimbal.yaw.give_current = 0;
//        launcher.fire_l.speed = 0;
//        launcher.fire_r.speed = 0;
        launcher.trigger.give_current = 0;
    }
    if (detect_list[DETECT_GIMBAL_6020_PITCH].status == OFFLINE) {
        gimbal.pitch.give_current = 0;
    }
    if (detect_list[DETECT_GIMBAL_6020_YAW].status == OFFLINE) {
        gimbal.yaw.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_L].status == OFFLINE) {
        launcher.fire_l.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_3508_FIRE_R].status == OFFLINE) {
        launcher.fire_r.give_current = 0;
    }
    if (detect_list[DETECT_LAUNCHER_2006_TRIGGER].status == OFFLINE) {
        launcher.trigger.give_current = 0;
    }
}

//云台电机闭环控制函数
fp32 radio_rc=0.8f;
int16_t pitch_LF2_test;
static void gimbal_ctrl_loop_cal(){
    //云台转速（gyro)设置
    //PID计算,是位置环控制，先算出速度设定值gyro_set，然后再用这个设定值算出电流值,速度最大值设为180°，最小值设为-180
    //使用的函数是pid_loop_calc();，这个函数实际上是一个上层调用，大家可以点进去理解一下



    //滤波器过滤yaw角
    first_order_filter_cali(&filter_yaw_gyro_in, gyro_yaw);
    //打上电流，使用的函数是pid_calc()




////调试速度环用（注意角度环所输出的量级可能与单环时给的set不同，可能会导致在调角度环时的问题，建议双环一起调）
//    first_order_filter_cali(&filter_yaw_gyro_in,gyro_yaw);
//    gimbal.yaw.give_current= pid_calc(&gimbal.yaw.speed_p,
//                                      filter_yaw_gyro_in.out,
//                                      -rc_ctrl.rc.ch[YAW_CHANNEL]);

    //pitch角也和上面差不多,两个都是使用pid_calc()函数



    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);




//    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);
//    gimbal.pitch.give_current= -pid_calc(&gimbal.pitch.speed_p,
//                                      filter_pitch_gyro_in.out,
//                                      rc_ctrl.rc.ch[PITCH_CHANNEL]*0.5);
    //最后做一个滤波处理
    gimbal.pitch.give_current= (int16_t)Apply(&pitch_current_out,gimbal.pitch.give_current);
}


static void gimbal_man_ctrl_loop_cal(){
    gimbal.yaw.gyro_set= pid_loop_calc(&gimbal.yaw.angle_p,
                                       gimbal.yaw.absolute_angle_get,
                                       gimbal.yaw.absolute_angle_set,
                                       180,
                                       -180);

    first_order_filter_cali(&filter_yaw_gyro_in, gyro_yaw);

    gimbal.yaw.give_current= pid_calc(&gimbal.yaw.speed_p,
                                      gyro_yaw,
                                      gimbal.yaw.gyro_set);

    gimbal.pitch.gyro_set= pid_calc(&gimbal.pitch.angle_p,
                                    gimbal.pitch.absolute_angle_get,
                                    gimbal.pitch.absolute_angle_set);
    gyro_pitch=Apply(&pitch_speed_out,gyro_pitch);
    first_order_filter_cali(&filter_pitch_gyro_in,gyro_pitch);

//    first_order_filter_cali(&pitch_first_order_set,gimbal.pitch.gyro_set);

    gimbal.pitch.give_current= -pid_calc(&gimbal.pitch.speed_p,
                                         gyro_pitch,
                                         gimbal.pitch.gyro_set);//?????????????

    gimbal.pitch.give_current= (int16_t)Apply(&pitch_current_out,gimbal.pitch.give_current);
}


//云台角度更新
static void gimbal_angle_update(){
    //pitch绝对角度和相对角度，提示：绝对角度储存在了INS_angle[]数组里面，要弧度换为角度
    //相对角度需要使用函数motor_ecd_to_angle_change()函数，这个函数在can_receive.c文件里，算出相对角度



    //yaw的绝对角度和相对角度，跟上面一样



    //pitch和yaw陀螺仪绝对角度，提示：储存在了INS_gyro里



    //接下来是处理视觉的数据
    //id数据储存
    if (Referee.GameRobotStat.robot_id<10){
        vision_data.id = 7;
    }else{
        vision_data.id = 107;
    }
    //将yaw, pitch的绝对角度还有射击速度shoot_speed,roll存进vision_data中,可以点开vision_data中看看有什么是要初始化的
    //提示：shoot_speed在裁判系统变量Referee中,roll还是存在INS_angle[](姿态解算)中



    //把四元数存进去,四元数还是在INS中，提示：INS_quat[]




    rm_queue_data(VISION_ID, &vision_data, sizeof(vision_t));
}

//通过这个函数，云台检测到pit绝对角为接近0时，获取当前pit_offset。在调试模式获取了offset后，在初始化函数中更改。
static void pit_offset_get(){
    if(gimbal.pitch.absolute_angle_get<=-0.002 && gimbal.pitch.absolute_angle_get>=0.002){
        gimbal.pitch.motor_measure->offset_ecd=motor_pitch_measure.ecd;//在这一行打断点调试，触发时成功获取0度时offset
    }
}

//一键掉头
static void gimbal_turn_back_judge(){
    if(KeyBoard.R.click_flag == 1){
        KeyBoard.R.click_flag = 0;
        gimbal.yaw.absolute_angle_set+=180;
    }
}

//弹仓盖控制
static void magazine_cover_control() {
    if (KeyBoard.G.click_flag == 1 || rc_ctrl.rc.ch[4]<-300) {//搓那个滚轮
        servo_pwm_set(1800, 2);//TIM_CHANNEL_4||左弹仓
        servo_pwm_set(800, 3);//TIM_CHANNEL_3||右弹仓
    } else {
        servo_pwm_set(800, 2);
        servo_pwm_set(1800, 3);
    }
}

static void gimbal_uiInfo_packet(){
    ui_robot_status.gimbal_mode=gimbal.mode;
    ui_robot_status.fire_mode=launcher.fire_mode;
    ui_robot_status.relative_yaw_value=gimbal.yaw.relative_angle_get;
    ui_robot_status.pitch_value=gimbal.pitch.absolute_angle_get;
    //?????????????灯
//    if(ABS(launcher.fire_l.motor_measure->speed_rpm)>500&&ABS(launcher.fire_r.motor_measure->speed_rpm)>500)
////        HAL_GPIO_WritePin(LED7_PORT,LED5_PIN,GPIO_PIN_RESET);
//    else
////        HAL_GPIO_WritePin(LED7_PORT,LED5_PIN,GPIO_PIN_SET);
    if(ABS(launcher.fire_l.motor_measure->speed_rpm)>500&&ABS(launcher.fire_r.motor_measure->speed_rpm)>500)
        led.mode=SHOOT;

}

static void gimbal_power_stop(){
    if(Referee.GameRobotStat.mains_power_gimbal_output==0)
    {
        gimbal.mode=GIMBAL_RELAX;
    }

    if(Referee.GameRobotStat.mains_power_shooter_output==0)
    {
        launcher.fire_l.give_current = 0;
        launcher.fire_r.give_current = 0;
        launcher.trigger.give_current = 0;
    }
}