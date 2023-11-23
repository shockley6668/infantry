//
// Created by xhuanc on 2021/10/10.
//

/*include*/
#include "Chassis.h"
/*define*/
/*轮子控制映射：                                 解算坐标：      x(前)
            ****      前       ****                                |
           * 2 LF *          * 1 RF *                              |
            ****              ****                                 |
                                                                   |
           左                   右                 y  --------------z-----------
                                                                   |
            ****              ****                                 |
          * 3 LB *          * 4 RB *                               |
            ****      后      ****                                 |

*/
/*变量*/
extern RC_ctrl_t rc_ctrl;
ramp_function_source_t chassis_auto_vx_ramp;
ramp_function_source_t chassis_auto_vy_ramp;
ramp_function_source_t chassis_auto_vw_ramp;
ramp_function_source_t chassis_3508_ramp[4];
chassis_t chassis;
extern gimbal_t gimbal;
//上位机下发数据
extern robot_ctrl_info_t robot_ctrl;
//底盘解算发送数据
extern chassis_odom_info_t chassis_odom;
//遥控器数据
extern key_board_t KeyBoard;
//电容数据
extern cap2_info_t cap2;
//发送机器人id
vision_t vision_data;
//功率控制中的电流控制
int16_t give_current_limit[4];
static fp32 rotate_ratio_f = ((Wheel_axlespacing + Wheel_spacing) / 2.0f - GIMBAL_OFFSET); //rad
static fp32 rotate_ratio_b = ((Wheel_axlespacing + Wheel_spacing) / 2.0f + GIMBAL_OFFSET);
static fp32 wheel_rpm_ratio = 60.0f / (PERIMETER * M3508_DECELE_RATIO); //车轮转速比

/*      函数及声明   */
static void chassis_init(chassis_t *chassis_ptr);

static void chassis_set_mode(chassis_t *chassis_ptr);

static void chassis_ctrl_info_get();

static void chassis_relax_handle();

static void chassis_spin_handle();

static void chassis_wheel_cal();

static void chassis_wheel_loop_cal();

void chassis_device_offline_handle();

void send_robot_id();

void chassis_follow_gimbal_handle();

static void chassis_power_limit();

static void chassis_vector_powerControl();

void chassis_can_send_back_mapping();

static void chassis_pc_ctrl();

static float chassis_speed_change();

/*程序主体*/
_Noreturn void chassis_task(void const *pvParameters) {

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis);////底盘初始化，需要编写
    send_robot_id(); //给视觉发送机器人ID，确认自己是红蓝方
    //主任务循环
    while (1) {
        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误

        //更新PC的控制信息
        update_pc_info();

        //设置底盘模式
        chassis_set_mode(&chassis);////需要编写

        //遥控器获取底盘方向矢量
        chassis_ctrl_info_get();////需要编写

        //遥控断电失能
        chassis_device_offline_handle();////需要编写

        //判断底盘模式选择 决定是否覆盖底盘转速vw;
        switch (chassis.mode) {
            case CHASSIS_ONLY:
                HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
                break;

            case CHASSIS_FOLLOW_GIMBAL:
                HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
                chassis_follow_gimbal_handle();////需要编写
                break;

            case CHASSIS_SPIN:
                HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_RESET);
                chassis_spin_handle();////需要编写
                break;

            case CHASSIS_RELAX:
                chassis_relax_handle();////需要编写
                HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
                break;
            case CHASSIS_BLOCK:
            {
                chassis.vx=0;
                chassis.vy=0;
                chassis.vw=0;
                HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
            }
        }

        if (chassis.mode != CHASSIS_RELAX) {
            //底盘解算
            chassis_wheel_cal();////需要编写
            //驱电机闭环
            chassis_wheel_loop_cal();////需要编写
            //功率限制
            chassis_power_limit();
            //电机映射
            chassis_can_send_back_mapping();////需要编写
        }
        xTaskResumeAll();

        vTaskDelay(1);
    }

}
void send_robot_id()
{
    if(Referee.GameRobotStat.robot_id<10)//红色方的ID小于10
    {
        vision_data.id=1;
    }
    else{
        vision_data.id=0;
    }
}
static void chassis_init(chassis_t *chassis_ptr) {
    //初始化步骤可以看看这个函数参数chassis_ptr里面需要初始化什么
    if (chassis_ptr == NULL)
        return ;
    //以下是原来代码的思路，
    //1.pid初始化，这里我们需要将每个电机都初始化
    //扭动电机的pid初始化，在Chassic.h文件里可以看见一个chassis_vw_pid，是用来跟随云台的电机。



    //底盘驱动电机速度环初始化和电机数据结构体获取，这里是四个轮子的初始化，写个fof循环可以解决
    //提示：motor_measure需要初始化，里面储存了当前电机的信息，在receive.h里面有；还有pid速度环需要初始化



    //2.模式设置
    //初始时底盘模式为失能 还有旋转模式为NORMAL_SPIN



    //接下来还有一个斜坡函数的初始化，这个函数目的是让轮子转的更平滑点，用ramp_init()函数，在ramp.c文件中可见
    //大家看看就行了
    ramp_init(&chassis_3508_ramp[LF],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[RF],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[RB],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[LB],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_auto_vx_ramp, 0.8f, MAX_CHASSIS_AUTO_VX_SPEED, -MAX_CHASSIS_AUTO_VX_SPEED);
    ramp_init(&chassis_auto_vy_ramp, 0.8f, MAX_CHASSIS_AUTO_VY_SPEED, -MAX_CHASSIS_AUTO_VY_SPEED);
    ramp_init(&chassis_auto_vw_ramp, 0.3f, MAX_CHASSIS_AUTO_VW_SPEED, -MAX_CHASSIS_AUTO_VW_SPEED);

    //chassis.spin_mode=NORMAL_SPIN;

}

static void chassis_set_mode(chassis_t* chassis){
    //这里需要设置底盘模式，模式根据遥控系统确定
    if(chassis==NULL)
        return;
    //模式设置
    //这里是个宏定义，可以自己点击去看看，当括号里面的值是设定的值时有true，不是则false
    //下面请模仿第一样例根据提示写出其他模式
    if(switch_is_down(rc_ctrl.rc.s[RC_s_L])&&switch_is_down(rc_ctrl.rc.s[RC_s_R]))
    {
        chassis->last_mode=chassis->mode;
        chassis->mode=CHASSIS_RELAX;
    }
    //当左键不是在最下面且右键在最下面时为CHASSIS_ONLY模式



    //当右键处于中间时，为底盘跟随云台模式CHASSIS_FOLLOW_GIMBAL



    //当右键上拉时，为底盘旋转模式,CHASSIS_SPIN



    //打符时最好保持底盘静止
    //如果开启打符模式，底盘刹车,CHASSIS_BLOCK模式



    //UI更新---底盘模式
    ui_robot_status.chassis_mode=chassis->mode;
}

static void chassis_ctrl_info_get() {
    chassis_pc_ctrl();//这里是对电脑控制时进行的更新
    //下面是对遥控器控制时进行更新
    //提示：遥控器控制是某个摇杆的值乘上系数，将这个数赋给vx就行了，乘上的系数也有了，就是RC_TO_VX
    //值得注意的是，遥控器的控制还需要叠加上电脑端的控制，比如vx_pc啊这些



//    VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-0.300),(MAX_CHASSIS_VX_SPEED-0.300));


}

//将期望速度转为转子期望转速
static void chassis_wheel_cal(){
    fp32 max=0;
    fp32 wheel_rpm[4];
    fp32 vx, vy, vw;

    //底盘最大速度控制（功率控制）根据裁判信息选择
    chassis_vector_powerControl();

    vx=chassis.vx;
    vy=chassis.vy;
    vw=chassis.vw;

    //轮运动学解算
    //轮运动学解算，例子：wheel_rpm[0] = ？？？？（这里就是写解算的算法了，自己琢磨一下）




    // find max item
    //下面是一个限制
    for (uint8_t i = 0; i < 4; i++) {
        if (abs(wheel_rpm[i]) > max) {
            max = abs(wheel_rpm[i]);
        }
    }
    // equal proportion
    if (max > M3508_MAX_RPM) {
        float rate = M3508_MAX_RPM / max;
        for (uint8_t i = 0; i < 4; i++) wheel_rpm[i] *= rate;
    }
    //下面请把轮子的设定数据rpm_set赋值，例子：chassis.motor_chassis[RF].rpm_set=wheel_rpm[0];



}

static void chassis_wheel_loop_cal() {
    //这里是PID把速度换算成电流，将当前值和设定值放进去运算即可
    //请把代码写在每个ramp_calc()下面,里面那个chassis.motor_chassis[].rpm_set就是设定值
    ramp_calc(&chassis_3508_ramp[RF],chassis.motor_chassis[RF].rpm_set);


    ramp_calc(&chassis_3508_ramp[LF],chassis.motor_chassis[LF].rpm_set);



    ramp_calc(&chassis_3508_ramp[RB],chassis.motor_chassis[RB].rpm_set);



    ramp_calc(&chassis_3508_ramp[LB],chassis.motor_chassis[LB].rpm_set);


}

//功率控制
void chassis_power_limit() {
    //更新电容状态
    cap_info_update();

    if(cap2.charge_status!=1) {

        chassis.chassis_power_limit.total_current = 0;
        chassis.chassis_power_limit.total_current_limit = 0;
        fp32 power_buffer = chassis.chassis_power_limit.power_buff;
        fp32 limit_k;
        if (detect_list[DETECT_REFEREE].status != ONLINE) {
            chassis.chassis_power_limit.total_current_limit = CHASSIS_CURRENT_LIMIT_40W;
        } else {
            chassis.chassis_power_limit.power_buff = Referee.PowerHeatData.chassis_power_buffer > CHASSIS_POWER_BUFF ?
                                                     CHASSIS_POWER_BUFF : Referee.PowerHeatData.chassis_power_buffer;
//                chassis.chassis_power_limit.limit_k=(cap_info.cap_value-cap_info.min_voltage)/(cap_info.max_cap_voltage-cap_info.min_voltage);
            chassis.chassis_power_limit.limit_k= chassis.chassis_power_limit.power_buff/CHASSIS_POWER_BUFF;
//            if (Referee.PowerHeatData.chassis_power = Referee.GameRobotStat.max_chassis_power-20) {
            if(chassis.chassis_power_limit.limit_k <=0)
            {
                chassis.chassis_power_limit.limit_k =0.15f;
            }
            else if(Referee.PowerHeatData.chassis_power_buffer<(Referee.PowerHeatData.chassis_power_buffer)*0.3) {
                chassis.chassis_power_limit.limit_k =
                        chassis.chassis_power_limit.limit_k*chassis.chassis_power_limit.limit_k;
            }

            chassis.chassis_power_limit.total_current_limit =
                    chassis.chassis_power_limit.limit_k * CHASSIS_CURRENT_LIMIT_TOTAL;
        }
        for (uint8_t i = 0; i < 4; i++)
            chassis.chassis_power_limit.total_current += abs(chassis.motor_chassis[i].give_current);

        for (uint8_t i = 0; i < 4; i++)
            give_current_limit[i] = chassis.motor_chassis[i].give_current;

        if (chassis.chassis_power_limit.total_current > chassis.chassis_power_limit.total_current_limit) {
            for (uint8_t i = 0; i < 4; i++)
                give_current_limit[i] = (int16_t) (chassis.motor_chassis[i].give_current *
                                                   chassis.chassis_power_limit.total_current_limit
                                                   / chassis.chassis_power_limit.total_current);
        }
        //????????
        for (uint8_t i = 0; i < 4; i++)
            chassis.motor_chassis[i].give_current = give_current_limit[i];
    }
//    power_real=((ABS(chassis.motor_chassis[0].motor_measure->given_current)+
//                 ABS(chassis.motor_chassis[1].motor_measure->given_current)+
//                 ABS(chassis.motor_chassis[2].motor_measure->given_current)+
//                 ABS(chassis.motor_chassis[3].motor_measure->given_current))*20/16384)*cap_info.input_current;
}

//把can接收时对真实电机的映射，在发送控制时映射回去为真实的电机，因为控制函数要按电机ID 1～4发送
void chassis_can_send_back_mapping(){
    //这里是把电流发送给电调
    int16_t *real_motor_give_current[4];
    //为发送变量赋值，例子：real_motor_give_current[0] = &chassis.motor[LF].give_current



    //使用CAN_cmd_motor()函数进行发送，注意是用哪个CAN口和电机ID




//    CAN_cmd_motor(CAN_1,
//                  CAN_MOTOR_0x200_ID,
//                  0,
//                  0,
//                  0,
//                  0
//    );

}

void chassis_device_offline_handle() {
    //在Detection.h里面是检测设备的，在清单list里，如果DETECT_REMOTE状态是掉线的话，将底盘模式设为RELAX失能状态
    //大家可以点进去看看detect_list是啥

}
static void chassis_relax_handle() {
    //这个就直接给电机发送0电流就行了

}

void chassis_follow_gimbal_handle(){
    // 以上电时的yaw为中心，yaw不动使底盘去归中到对应的编码器中点，在运动时底盘运动方向为底盘正方向，
    // 当yaw转动后产生了编码器与编码器中值的偏移使底盘进行“跟随yaw”转动，
    // 然后将底盘正方向移至yaw所指向的方向，就做到了底盘跟云台然后yaw指哪走哪

    //找到相对角度，根据这个相对角度旋转就行
    //1.得到云台的相对角度，转化为弧度制



    //2.算出这个弧度下的sin和cos是多少




    //3.最后就是把算出来的sin和cos值乘上去做一个速度矢量分解
    //速度矢量分解



    //4.算出vw也就是角速度，使用pid算法，以云台相对角度为当前值，0作为设定值，算出角速度



    //VAL_LIMIT(chassis.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);
}

void chassis_spin_handle()
{
    //小陀螺实现分为三步：
    //1、获取底盘与云台的相对角度θ。底盘绝对角度由YAW轴电机提供，云台绝对角度由云台上的角度传感器提供。
    //2、根据θ，把整车运动速度（大小和方向）分解到底盘坐标
    //3、根据底盘坐标速度进行麦轮速度分解，整车效果则表现为按云台坐标运动

    //对于一直旋转这个功能，我们还是要算vx,vy,vw这三个值，其中vx,vy还是要根据相对角度算(跟跟随云台那个功能一样)，vw给个速度转就行
    /*


    //速度矢量分解

    //这里一个是匀速旋转，一个是变速旋转,两个模式,使用switch解决
    /*
    switch(chassis.spin_mode){
        case NORMAL_SPIN:

            break;

        case HIDDEN_ARMOR_SPEED_CHANGE:
            if((gimbal.yaw.relative_angle_get>=0 && gimbal.yaw.relative_angle_get<=60) ||
               (gimbal.yaw.relative_angle_get>=90 && gimbal.yaw.relative_angle_get<=150)||
               (gimbal.yaw.relative_angle_get>=-90 && gimbal.yaw.relative_angle_get<=-30)||
               (gimbal.yaw.relative_angle_get>=-180 && gimbal.yaw.relative_angle_get<=-120)
               ){
               //装甲板没有面向敌人的速度
            }
            else{
                 //装甲板面向敌人的速度
            }
    }*/
}

static void chassis_pc_ctrl(){

    float speed_change=chassis_speed_change();//获取加速度

    //键盘控制下的底盘以斜坡式变化
    if(KeyBoard.W.status==KEY_PRESS)//键盘前进键按下
    {
        chassis.vx_pc+=speed_change;//速度增量
    }
    else if(KeyBoard.S.status==KEY_PRESS)
    {
        chassis.vx_pc-=speed_change;
    }
    else{
        chassis.vx_pc=0;
    }

    if(KeyBoard.A.status==KEY_PRESS)//键盘前进键按下
    {
        chassis.vy_pc+=speed_change;
    }
    else if(KeyBoard.D.status==KEY_PRESS)
    {
        chassis.vy_pc-=speed_change;
    }
    else{
        chassis.vy_pc=0;
    }
//    if(chassis.mode==CHASSIS_SPIN)//灯
//    {
//        HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_RESET);
//    } else{
//        HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_SET);
//    }

    if(KeyBoard.E.click_flag==1)//
    {
        chassis.mode=CHASSIS_SPIN;
    }

    if(chassis.mode==CHASSIS_SPIN)//灯
    {
//        led.mode=SPIN;//无led
    }

//    if(chassis.mode==CHASSIS_CLIMBING)//灯
//    {
//        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_RESET);
//    } else{
//        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_SET);
//    }
}

static float chassis_speed_change() {
    float speed_change = 0;
    if (cap2.mode==1) {//开启电容 增加加速度
        speed_change = 0.0025;//最大加速度
    } else {
        switch (Referee.GameRobotStat.max_chassis_power) {//最大限制功率
            case 40: {
                speed_change=0.0015;
            }
                break;
            case 45: {
                speed_change=0.0018;
            }
                break;
            case 50: {
                speed_change=0.0018;
            }
                break;
            case 55: {
                speed_change=0.0018;
            }
                break;
            case 60: {
                speed_change=0.0018;
            }
                break;
            case 70: {
                speed_change=0.0020;
            }
                break;
            case 80: {
                speed_change=0.0020;
            }
                break;
            case 100: {
                speed_change=0.0023;
            }
                break;
            case 120: {
                speed_change=0.0025;
            }
                break;
            default:{
                speed_change=0.001;
            }break;
        }
    }
    return speed_change;
}

static void chassis_vector_powerControl(){
    cap_info_update();//根据电容开启和否限制底盘移动速度
    if(cap2.mode==1 && cap2.cap_voltage>14)//电容电压大于13 不限制底盘功率 通过增大pc_ctrl下的speed_change 提高底盘加速度;
    {
        VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED+2),(MAX_CHASSIS_VX_SPEED+2));
        VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED+2),(MAX_CHASSIS_VY_SPEED+2));
        VAL_LIMIT(chassis.vw,-6.5,6.5);
    }
    else {
        switch (Referee.GameRobotStat.max_chassis_power) {
            case 40:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.6),(MAX_CHASSIS_VX_SPEED-1.6));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.6),(MAX_CHASSIS_VY_SPEED-1.6));
                VAL_LIMIT(chassis.vw,-2.20,2.20);
            }break;
            case 45:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.500),(MAX_CHASSIS_VX_SPEED-1.500));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.500),(MAX_CHASSIS_VY_SPEED-1.500));
                VAL_LIMIT(chassis.vw,-2.25,2.25);
            }break;
            case 50:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.500),(MAX_CHASSIS_VX_SPEED-1.500));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.500),(MAX_CHASSIS_VY_SPEED-1.500));
                VAL_LIMIT(chassis.vw,-2.35,2.35);
            }break;
            case 55:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.400),(MAX_CHASSIS_VX_SPEED-1.400));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.400),(MAX_CHASSIS_VY_SPEED-1.400));
                VAL_LIMIT(chassis.vw,-2.50,2.50);
            }break;
            case 60:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.300),(MAX_CHASSIS_VX_SPEED-1.300));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.300),(MAX_CHASSIS_VY_SPEED-1.300));
                VAL_LIMIT(chassis.vw,-2.60,2.60);
            }break;
            case 70:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.100),(MAX_CHASSIS_VX_SPEED-1.100));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.100),(MAX_CHASSIS_VY_SPEED-1.100));
                VAL_LIMIT(chassis.vw,-2.75,2.75);
            }break;
            case 80:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.00),(MAX_CHASSIS_VX_SPEED-1.00));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.00),(MAX_CHASSIS_VY_SPEED-1.00));
                VAL_LIMIT(chassis.vw,-3.10,3.10);
            }break;
            case 100:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED+0.100),(MAX_CHASSIS_VX_SPEED+0.100));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED+0.100),(MAX_CHASSIS_VY_SPEED+0.100));
                VAL_LIMIT(chassis.vw,-3.30,3.30);
            }break;
            case 120:
            {
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED+0.200),(MAX_CHASSIS_VX_SPEED+0.200));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED+0.200),(MAX_CHASSIS_VY_SPEED+0.200));
                VAL_LIMIT(chassis.vw,-3.35,3.35);
            }break;
            default:{
                VAL_LIMIT(chassis.vx,-(MAX_CHASSIS_VX_SPEED-1.500),(MAX_CHASSIS_VX_SPEED-1.500));
                VAL_LIMIT(chassis.vy,-(MAX_CHASSIS_VY_SPEED-1.500),(MAX_CHASSIS_VY_SPEED-1.500));
                VAL_LIMIT(chassis.vw,-2.20,2.20);
            }
        }
    }

    //操作手主动慢速
    if(KeyBoard.CTRL.status==KEY_PRESS)
    {
        VAL_LIMIT(chassis.vx,-MAX_CHASSIS_VX_SPEED*0.2,MAX_CHASSIS_VX_SPEED*0.2);
        VAL_LIMIT(chassis.vy,-MAX_CHASSIS_VY_SPEED*0.2,MAX_CHASSIS_VY_SPEED*0.2);
        VAL_LIMIT(chassis.vw,-MAX_CHASSIS_VW_SPEED,MAX_CHASSIS_VW_SPEED);
    }
//    if(KeyBoard.SHIFT.status==KEY_PRESS){
//        VAL_LIMIT(chassis.vx,-MAX_CHASSIS_VX_SPEED*1.5,MAX_CHASSIS_VX_SPEED*1.5);
//        VAL_LIMIT(chassis.vy,-MAX_CHASSIS_VY_SPEED*1.5,MAX_CHASSIS_VY_SPEED*1.5);
//        VAL_LIMIT(chassis.vw,-MAX_CHASSIS_VW_SPEED,MAX_CHASSIS_VW_SPEED);
//    }
}