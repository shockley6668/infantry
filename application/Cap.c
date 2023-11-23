//
// Created by xhuanc on 2022/5/22.
//

#include "Cap.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "can_receive.h"
#include "Referee.h"
#include "key_board.h"
#include "bsp_led.h"
#include "bsp_adc.h"
#define CAP_MIN_VOLTAGE 14
#define CAP_MAX_VOLTAGE 21

cap_info_t cap_info;
cap2_info_t cap2;
extern key_board_t KeyBoard;


void cap_info_update(){

    if(KeyBoard.SHIFT.status == KEY_PRESS)//V����״̬Ϊ1 Ϊ��������
    {
        cap2.send_data[0]=(uint8_t)0xFF;//���ݿ���
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET); // 4��12�ոĵ���LED
    }
    else{
        cap2.send_data[0]=(uint8_t)0x00;//���ݹر�
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET); // 4��12�ոĵ���LED
    }

    if(cap2.mode==0) {
        cap2.send_data[1] = (uint8_t) Referee.GameRobotStat.max_chassis_power - 15;//��Ҫ������һЩ����
    }
    else{
        cap2.send_data[1] = (uint8_t) Referee.GameRobotStat.max_chassis_power - 5;//��Ҫ������һЩ����
    }

    if(Referee.GameRobotStat.remain_HP<=0){
        cap2.send_data[2]=ENABLE;
    }
    else{
        cap2.send_data[2]=DISABLE;
    }

    cap2.send_data[3]=(uint8_t)0x00;

    cap2.send_data[4]=Referee.GameState.stage_remain_time>>8;
    cap2.send_data[5]=Referee.GameState.stage_remain_time;

//    if(cap_info.ctrl_cmd==CAP_ON)//���ݿ�����
//        HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_RESET);
//    else
//        HAL_GPIO_WritePin( LED2_PORT,LED2_PIN,GPIO_PIN_SET);
//    if( cap2.send_data[0]==0xFF)//���ݿ�����
//        led.mode=CAP;


}
void cap_task(void const *pvParameters)
{
    osDelay(CAP_TASK_INIT_TIME);
    while (1)
    {
        cap_info_update();
        CAN_cmd_cap2(&cap2);
        osDelay(100);
    }
}