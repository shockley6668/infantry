#include <launcher.h>
#include <Detection.h>
#include "bsp_led.h"
#include "main.h"
#include "cmsis_os.h"
#include "Gimbal.h"
#include "Chassis.h"
#include "Auto.h"
#include "Cap.h"

extern chassis_t chassis;
extern TIM_HandleTypeDef htim5;
extern launcher_t launcher;
extern cap_info_t cap_info;
extern cap2_info_t cap2;
led_t led;

uint8_t led_flowing;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          显示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */
void aRGB_led_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red,green,blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}

#define LED_SET(n) HAL_GPIO_WritePin(LED##n##_PORT,LED##n##_PIN,GPIO_PIN_SET)
#define LED_RESET(n) HAL_GPIO_WritePin(LED##n##_PORT,LED##n##_PIN,GPIO_PIN_RESET)

void led_dashboard(){
    //switch(led.mode){
    //case SPIN:
    if(chassis.mode==CHASSIS_SPIN)//灯
    {
        LED_RESET(6);
    }
    else{
        LED_SET(6);
    }
    //break;

    //case SHOOT:
    if(ABS(launcher.fire_l.motor_measure->speed_rpm)>500&&ABS(launcher.fire_r.motor_measure->speed_rpm)>500)
        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_SET);
    // break;

    //case AUTO_AIM:
    if(detect_list[DETECT_AUTO_AIM].status==OFFLINE)
    {
        HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_RESET);
    };
    // break;

    // case CAP:
    if(cap2.send_data[0]==0xFF)//电容开亮灯
        HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin( LED2_PORT,LED2_PIN,GPIO_PIN_SET);
    // }
}


void led_init(){
    HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED3_PORT,LED3_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED7_PORT,LED7_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED4_PORT,LED4_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_SET);
}

void led_light(uint8_t led_1,uint8_t led_2,uint8_t led_3,uint8_t led_4,uint8_t led_5,uint8_t led_6,uint8_t led_7){
    if(led_1==1){
        HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(LED1_PORT,LED1_PIN,GPIO_PIN_SET);
    }

    if(led_2==1){
        HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(LED2_PORT,LED2_PIN,GPIO_PIN_SET);
    }

    if(led_3==1){
        HAL_GPIO_WritePin(LED3_PORT,LED3_PIN,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(LED3_PORT,LED3_PIN,GPIO_PIN_SET);
    }

    if(led_4==1){
        HAL_GPIO_WritePin(LED4_PORT,LED4_PIN,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(LED4_PORT,LED4_PIN,GPIO_PIN_SET);
    }

    if(led_5==1){
        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(LED5_PORT,LED5_PIN,GPIO_PIN_SET);
    }

    if(led_6==1){
        HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(LED6_PORT,LED6_PIN,GPIO_PIN_SET);
    }

    if(led_7==1){
        HAL_GPIO_WritePin(LED7_PORT,LED7_PIN,GPIO_PIN_RESET);
    }
    else{
        HAL_GPIO_WritePin(LED7_PORT,LED7_PIN,GPIO_PIN_SET);
    }
}


void led_flow(){
    led_flowing=1;
    led_light(1,0,0,0,0,0,1);
    osDelay(DELAY_TIME);

    led_light(0,0,0,0,0,0,0);
    osDelay(DELAY_TIME);

    led_light(0,0,1,0,1,0,0);
    osDelay(DELAY_TIME);

    led_light(0,0,0,1,0,0,0);
    osDelay(DELAY_TIME);

    led_light(1,0,0,1,0,0,1);
    osDelay(DELAY_TIME);

    led_light(0,1,0,1,0,1,0);
    osDelay(DELAY_TIME);

    led_light(0,0,1,1,1,0,0);
    osDelay(DELAY_TIME);

    led_light(1,0,1,1,1,0,1);
    osDelay(DELAY_TIME);

    led_light(0,1,1,1,1,1,0);
    osDelay(DELAY_TIME);

    led_light(1,1,1,1,1,1,1);
    osDelay(DELAY_TIME);

    led_light(0,0,0,1,0,0,0);
    osDelay(DELAY_TIME);

    led_light(0,0,1,0,1,0,0);
    osDelay(DELAY_TIME);

    led_light(0,1,0,0,0,1,0);
    osDelay(DELAY_TIME);

    led_light(1,0,0,0,0,0,1);
    osDelay(DELAY_TIME);

    led_light(1,1,0,0,0,1,1);
    osDelay(DELAY_TIME);

    led_light(1,1,1,0,1,1,1);
    osDelay(DELAY_TIME);

    led_light(1,1,1,1,1,1,1);
    osDelay(DELAY_TIME);

    led_light(0,0,0,0,0,0,0);
    osDelay(DELAY_TIME);

    led_light(1,1,1,1,1,1,1);
    osDelay(DELAY_TIME);

    led_light(0,0,0,0,0,0,0);
    osDelay(DELAY_TIME);

    led_light(1,1,1,1,1,1,1);
    osDelay(2000);

    led_light(0,0,0,0,0,0,0);
    led_flowing=0;
}

void led_off_flow(){
    led_flowing=1;
    led_light(1,1,1,1,1,1,1);
    osDelay(DELAY_TIME);

    led_light(0,0,0,0,0,0,0);
    osDelay(DELAY_TIME);

    led_light(1,1,1,1,1,1,1);
    osDelay(DELAY_TIME);

    led_light(0,1,1,1,1,1,0);
    osDelay(DELAY_TIME);

    led_light(1,0,1,1,1,0,1);
    osDelay(DELAY_TIME);

    led_light(1,1,0,1,0,1,1);
    osDelay(DELAY_TIME);

    led_light(1,1,1,0,1,1,1);
    osDelay(DELAY_TIME);

    led_light(0,1,1,0,1,1,0);
    osDelay(DELAY_TIME);

    led_light(1,0,1,0,1,0,1);
    osDelay(DELAY_TIME);

    led_light(1,1,0,0,0,1,1);
    osDelay(DELAY_TIME);

    led_light(0,1,0,0,0,1,0);
    osDelay(DELAY_TIME);

    led_light(1,0,0,0,0,0,1);
    osDelay(DELAY_TIME);

    led_light(0,1,0,0,0,1,0);
    osDelay(DELAY_TIME);

    led_light(0,0,1,0,1,0,0);
    osDelay(DELAY_TIME);

    led_light(0,0,0,1,0,0,0);
    osDelay(DELAY_TIME);

    led_light(0,0,0,0,0,0,0);
    osDelay(DELAY_TIME);

    led_light(0,0,1,0,1,0,0);
    osDelay(DELAY_TIME);

    led_light(0,1,0,0,0,1,0);
    osDelay(DELAY_TIME);

    led_light(1,0,0,0,0,0,1);
    osDelay(DELAY_TIME);

    led_light(0,0,0,0,0,0,0);
    osDelay(DELAY_TIME);

    led_light(1,0,0,0,0,0,1);
    osDelay(DELAY_LONG_TIME);

    led_light(0,0,0,0,0,0,0);
    led_flowing=0;
}



void led_task(void const *pvParameters){
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    led_init();

    while(1){
        if(chassis.last_mode==CHASSIS_RELAX && chassis.mode!=CHASSIS_RELAX){
            led_flow();
        }
        else if(chassis.last_mode!=CHASSIS_RELAX && chassis.mode==CHASSIS_RELAX){
            led_off_flow();
        }
        if(led_flowing==0){
            led_dashboard();
            vTaskDelay(10);
        }
    }


}

