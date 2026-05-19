#include "UI.h"
#include "usart.h"
#include "string.h"
#include "Driver_Judge.h"
#include "gpio.h"
#include "RcTask.h"
#include "CanBus_Task.h"
#include "CapInteract.h"
#include "ControlTask.h"
#include "ChassisControl.h"
#include "judge_interact.h"
#define  max_bullet	500

uint16_t last_bullet;
uint16_t now_bullet;
uint16_t add_bullet;
uint16_t all_bullet;


/**********************************************************************************************************************
  * @Func	 uint8_t RobotSendID(void)
  * @Brief   获取机器人自身ID(发送方)
  * @Param	 无
  * @Retval	 uint8_t：机器人ID
*********************************************************************************************************************/
uint8_t RobotSendID(void)
{
    return Rec_game_robot_status.robot_id;
}

/**********************************************************************************************************************
  * @Func	 uint16_t RobotReceiveID(void)
  * @Brief   获取对应客户端ID(接收方)
  * @Param	 无
  * @Retval	 uint16_t：客户端ID(0x0101~0x0169)
  * @Note    红方机器人返回0x0101~0x0105，蓝方返回0x0165~0x0169
*********************************************************************************************************************/
uint16_t RobotReceiveID(void)
{
    uint16_t return_id;
    switch(Rec_game_robot_status.robot_id)
    {
        case 1:return_id = 0x0101;break;
        case 2:return_id = 0x0102;break;
        case 3:return_id = 0x0103;break;
        case 4:return_id = 0x0104;break;
        case 5:return_id = 0x0105;break;
        case 101:return_id = 0x0165;break;
        case 102:return_id = 0x0166;break;
        case 103:return_id = 0x0167;break;
        case 104:return_id = 0x0168;break;
        case 105:return_id = 0x0169;break;
    }
    return return_id;
}
/**********************************************************************************************************************
  * @Func	 uint16_t RobotManReceiveID(void)
  * @Brief   获取操作手客户端ID
  * @Param	 无
  * @Retval	 uint16_t：操作手客户端ID(0x0007/0x0107)
*********************************************************************************************************************/
uint16_t RobotManReceiveID(void)
{
    uint16_t return_id;
    switch(Rec_game_robot_status.robot_id)
    {
        case 1:return_id = 0x0007;break;
        case 2:return_id = 0x0007;break;
        case 3:return_id = 0x0007;break;
        case 4:return_id = 0x0007;break;
        case 5:return_id = 0x0007;break;
        case 101:return_id = 0x0107;break;
        case 102:return_id = 0x0107;break;
        case 103:return_id = 0x0107;break;
        case 104:return_id = 0x0107;break;
        case 105:return_id = 0x0107;break;
    }
    return return_id;
}
/**********************************************************************************************************************
  * @Func	 void Ui_Struct_Init(void)
  * @Brief   UI结构体参数初始化
  * @Param	 无
  * @Retval	 无
  * @Note    初始化自瞄标志和运动模式图片状态
*********************************************************************************************************************/
void Ui_Struct_Init(void)
{
    ui_state.Aim_Flag = 0;//切换动态标志位，0为静态1为动态
    ui_state.PIC_STA[Motion_Pic] = 0;

}


/*UI主任务*/
/**********************************************************************************************************************
  * @Func	 void UI_ALL_Task(void)
  * @Brief   UI主任务
  * @Param	 无
  * @Retval	 无
  * @Note    绘制底盘指南针，根据宏INFANTRY/HERO/OMNIDIR分发到对应的UI子任务
*********************************************************************************************************************/
void UI_ALL_Task(void)
{
    ChassisGuidance(960,540,80);//指针坐标半径
    if(KEY_CTRL&&KEY_R)
    {
        HAL_Delay(100);
        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();

    }//复位，复位可以刷新UI


    if(xRobotMode.ChassisMode == CHASSIS_FOLLOW) ui_state.PIC_STA[Motion_Pic] = 0;//跟随
    if(xRobotMode.ChassisMode == CHASSIS_GYROSC) ui_state.PIC_STA[Motion_Pic] = 1;//陀螺
    if(xRobotMode.ChassisMode == CHASSIS_SEPARA) ui_state.PIC_STA[Motion_Pic] = 2;//分离


///*通过当前允许发弹量推算出弹舱的弹丸数，max_bullet为弹舱装满的弹丸数*/
    // if(Rec_game_state.game_progress ==3)
    // {
    //     last_bullet = 0;
    //     now_bullet = 0;
    //     Rec_bullet_remaining.bullet_remaining_num_17mm = 0;
    //     add_bullet = 0;·
    // }


    last_bullet = now_bullet;
    now_bullet = Rec_bullet_remaining.bullet_remaining_num_17mm;
    int delta = now_bullet - last_bullet;
    if (delta > 0) {
        add_bullet += delta;
    }
    all_bullet = max_bullet + now_bullet + delta - add_bullet;



///*因为ui显示数字会出现奇怪的bug所以做了以下处理*/
//	if(Commucation_Data.pitch_angle>200)
//	{
//		ui_state.pitch_angle=Commucation_Data.pitch_angle;
////		ui_state.pitch_angle=(360-Commucation_Data.pitch_angle)*10;
//		ui_state.Up_flag=0;
//	}
//	else
//	{
//		ui_state.pitch_angle=Commucation_Data.pitch_angle*10;
//		ui_state.Up_flag=1;
//	}




#if	( INFANTRY == 1 )
    UI_INFANTRY_Task();
#elif ( HERO == 1 )
    UI_Hero_Task();
#elif (OMNIDIR == 1 )
            UI_ENGINEER_Task();
#endif



}

/**********************************************************************************************************************
  * @Func	 void UI_Hero_Task(void)
  * @Brief   英雄机器人UI任务
  * @Param	 无
  * @Retval	 无
  * @Note    绘制F/G/B字符和子弹数显示
*********************************************************************************************************************/
void UI_Hero_Task(void)
{
    /* 进入主任务*/
    if(ui_state.Aim_Flag ==0)
    {

        //添加静态
        UI_NEW_INIT();

//		UI_MODE_BUILD();

        //添加动态，该函数循环10次后进入动态
        UI_TEST(1);

        //添加字符
        UI_TEST_Character('F',1341,774,0x30,1);
        UI_TEST_Character('G',1396,774,0x31,1);
        UI_TEST_Character('B',1451,774,0x32,1);

        UI_TEST_Character_BULLET(1);
    }

    else
    {
        //改变动态
        UI_TEST(2);
    }

}




/**********************************************************************************************************************
  * @Func	 void UI_INFANTRY_Task(void)
  * @Brief   步兵机器人UI任务
  * @Param	 无
  * @Retval	 无
  * @Note    绘制F/G/B/H/M/L字符和子弹数显示
*********************************************************************************************************************/
void UI_INFANTRY_Task(void)
{
    /* 进入主任务*/
    if(ui_state.Aim_Flag ==0)
    {

        //添加静态
        UI_NEW_INIT();

        //添加动态，该函数循环10次后进入动态
        UI_TEST(1);

        //添加字符
        UI_TEST_Character('F',1341,774,0x30,1);
        UI_TEST_Character('G',1396,774,0x31,1);
        UI_TEST_Character('B',1451,774,0x32,1);
        UI_TEST_Character('H',1341,400,0x33,1);
        UI_TEST_Character('M',1396,400,0x34,1);
        UI_TEST_Character('L',1451,400,0x35,1);
        UI_TEST_Character_BULLET(1);
    }

    else
    {
        //改变动态
        UI_TEST(2);
    }


}


/**********************************************************************************************************************
  * @Func	 void UI_ENGINEER_Task(void)
  * @Brief   工程机器人UI任务
  * @Param	 无
  * @Retval	 无
*********************************************************************************************************************/
void UI_ENGINEER_Task(void)
{


}