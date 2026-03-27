#include "UI.h"
#include "usart.h"
#include "string.h"
#include "Driver_Judge.h"
#include "gpio.h"
#include "RcTask.h"
#include "CanBus_Task.h"
#include "CapInteract.h"
#include "ControlTask.h"
#include "arm_math.h"
#include "cmsis_os.h"

/*文件说明：这里放打包好的UI结构体*/

/*ui变量结构体声明*/
uint8_t Tx_Data[Tx_Data_MaxLength];

ui_struct      ui_state;
frame_header_  Tx_Header;
ext_robot_interact_data_t  UIdata_Header;

jianshu_t jianshu;
interaction_figure_7t Line;
ext_client_custom_graphic_character_t  character;



void ChassisGuidance(uint16_t circle_point_x,uint16_t circle_point_y,uint8_t Radius)  //底盘指向指南针
{
    static int8_t direction_x,direction_y;



    if( -xRelativeAngle.receive_yaw_relative_angle >= 2048)
    {
        direction_x = -1;  //1
        direction_y = 1;  //1
    }
    else if(-xRelativeAngle.receive_yaw_relative_angle < 2048 && -xRelativeAngle.receive_yaw_relative_angle >= 0)
    {
        direction_x = -1; //-1
        direction_y = 1;  //1
    }

    if(-xRelativeAngle.receive_yaw_relative_angle <= -2048)
    {
        direction_x = -1;  //-1
        direction_y = 1;   //1
    }
    else if(-xRelativeAngle.receive_yaw_relative_angle > -2048 && -xRelativeAngle.receive_yaw_relative_angle <= 0)
    {
        direction_x = -1; //1
        direction_y = 1; //1
    }

    ui_state.pointer[X_ZHOU] = circle_point_x + Radius * arm_sin_f32(-xRelativeAngle.receive_yaw_relative_angle /4096 * 3.1415926f) * direction_x;
    ui_state.pointer[Y_ZHOU] = circle_point_y + Radius * arm_cos_f32(-xRelativeAngle.receive_yaw_relative_angle /4096 * 3.1415926f) * direction_y;
    ui_state.pointer[X_ZHOU] = circle_point_x + Radius * arm_sin_f32(-xRelativeAngle.receive_yaw_relative_angle /4096.f * 3.14f) * direction_x;
    ui_state.pointer[Y_ZHOU] = circle_point_y + Radius * arm_cos_f32(-xRelativeAngle.receive_yaw_relative_angle /4096.f * 3.14f) * direction_y;
//

//


}


void UI_TEST(uint8_t operate_)
{
    static uint8_t seq=0;
    uint16_t Cmd_ID;
    memset(&Line,0,sizeof(Line));
    memset(Tx_Data,0,128);
    if(seq==10)
    {
        seq = 0;
        ui_state.Aim_Flag =1;
    }

    /*帧头 5byte*/
    Tx_Header.Sof = 0xA5;
    Tx_Header.Seq = seq++;
    Tx_Header.Data_length = 111;//sizeof(UIdata_Header) + sizeof(UI_CAR_SATE)
    memcpy(Tx_Data,(uint8_t*)&Tx_Header,5);//5
    Append_CRC8_Check_Sum(Tx_Data,5); //帧头

    /*命令ID  2byte*/
    Cmd_ID = 0x0301;
    memcpy(Tx_Data+5,(uint8_t *)&Cmd_ID,2);//2


    /*数据内容的帧头 6byte*/
    UIdata_Header.data_cmd_id = 0x0104;//七个图形
    UIdata_Header.send_ID = SendID; //英雄（红）
    UIdata_Header.receiver_ID = ReceID;//客户端英雄（红）
    memcpy(Tx_Data+7,(uint8_t*)&UIdata_Header,sizeof(UIdata_Header));//6


    if(seq<=10)
    {



        /*自瞄框*/
        if(mouse_right.deshake_key)
        {
            Line.interaction_figure[0].color = 4;//紫红色
            Line.interaction_figure[0].width = 3;//图形线宽
        }
        else if(KEY_Z)
        {
           Line.interaction_figure[0].color = 3;//橙色
            Line.interaction_figure[0].width = 3;//图形线宽

        }
        else
        {
            Line.interaction_figure[0].color = 8;//白色
            Line.interaction_figure[0].width = 1;//图形线宽
        }
        Line.interaction_figure[0].figure_name[0] = 0x70;//名字是为了区分图片内容
        Line.interaction_figure[0].operate_tpye =  operate_ ;//增加图形
        Line.interaction_figure[0].figure_tpye =  1; //矩形
        Line.interaction_figure[0].layer = 7;//图层 7
//	Line.interaction_figure[0].color = 8;//白色
//	Line.interaction_figure[0].width = 2;//图形线宽
        Line.interaction_figure[0].start_x = 673;//起点图形坐标x
        Line.interaction_figure[0].start_y = 360;//起点图坐标y
//	Line.interaction_figure[0].details_a = 1000;//
//	Line.interaction_figure[0].details_b = 130;//
        Line.interaction_figure[0].details_d = 1251;//
        Line.interaction_figure[0].details_e = 754;//


        /*弹丸数*/
        if(all_bullet <= 50)
        {
            Line.interaction_figure[1].color = 3;//橙色
        }
        else
        {
            Line.interaction_figure[1].color = 8;//白色
        }


        Line.interaction_figure[1].figure_name[0] = 0x71;//名字是为了区分图片内容 ， 为了后面删除图片所用
        Line.interaction_figure[1].operate_tpye =  operate_ ;//增加图形
        Line.interaction_figure[1].figure_tpye =  6; //整型数
        Line.interaction_figure[1].layer = 4;//图层 4
//	Line.interaction_figure[1].color = 8;//白色
        Line.interaction_figure[1].width = 3;//图形线宽
        Line.interaction_figure[1].details_a = 30;//字体大小
        Line.interaction_figure[1].start_x = 1365;//图形坐标x
        Line.interaction_figure[1].start_y = 655;//图坐标y
        Line.interaction_figure[1].details_c =all_bullet;


        /*装甲板识别*/
        Line.interaction_figure[2].figure_name[0] = 0x72;//名字是为了区分图片内容
        Line.interaction_figure[2].operate_tpye =  operate_ ;//增加图形
        Line.interaction_figure[2].figure_tpye =  2; //圆
        Line.interaction_figure[2].layer = 7;//图层 7
        Line.interaction_figure[2].color = 2;//绿色
        Line.interaction_figure[2].width = 15;//图形线宽
//    	Line.interaction_figure[2].start_x = 673+MiniPC_ReceivePacket.x/1440*578;//图形坐标x
//        Line.interaction_figure[2].start_y = 360+(-MiniPC_ReceivePacket.y+1080)/1080*394-35 ;//图坐标y
//        Line.interaction_figure[2].details_c = MiniPC_ReceivePacket.r/40;//

        /*电容数*/
        if(Cap_Measure.cap_value>=180)
        {
            Line.interaction_figure[3].color = 2;
        }
        else if(Cap_Measure.cap_value>=140&&Cap_Measure.cap_value<180)
        {
            Line.interaction_figure[3].color = 1;
        }
        else
        {
            Line.interaction_figure[3].color = 3;
        }

        Line.interaction_figure[3].figure_name[0] = 0x73;//名字是为了区分图片内容 ， 为了后面删除图片所用
        Line.interaction_figure[3].operate_tpye =  operate_ ;//增加图形
        Line.interaction_figure[3].figure_tpye =  6; //整型数
        Line.interaction_figure[3].layer = 4;//图层 4
//	Line.interaction_figure[3].color = 4;//白色
        Line.interaction_figure[3].width = 3;//图形线宽
        Line.interaction_figure[3].details_a = 30;//字体大小
        Line.interaction_figure[3].start_x = 623;//图形坐标x
        Line.interaction_figure[3].start_y = 548;//图坐标y
        Line.interaction_figure[3].details_c =Cap_Measure.cap_value ;


///*模式转换*/
        if(ui_state.PIC_STA[Motion_Pic] ==0)//F
        {
            Line.interaction_figure[4].start_x = 1348;//图形坐标x
            Line.interaction_figure[4].start_y = 762;//图坐标y
        }
        else if(ui_state.PIC_STA[Motion_Pic] ==1)//G
        {
            Line.interaction_figure[4].start_x = 1403;//图形坐标x
            Line.interaction_figure[4].start_y = 762;//图坐标y
        }

        else if(ui_state.PIC_STA[Motion_Pic] == 2)//B
        {
            Line.interaction_figure[4].start_x = 1460;//图形坐标x
            Line.interaction_figure[4].start_y = 762;//图坐标y
        }

        Line.interaction_figure[4].figure_name[0] = 0x74;//名字是为了区分图片内容
        Line.interaction_figure[4].operate_tpye =  operate_ ;//增加图形
        Line.interaction_figure[4].figure_tpye =  2; //圆
        Line.interaction_figure[4].layer = 7;//图层 7
        Line.interaction_figure[4].color = 3;//橙色
        Line.interaction_figure[4].width = 10;//图形线宽
//	Line.interaction_figure[4].start_x = 960;//图形坐标x
//	Line.interaction_figure[4].start_y = 540;//图坐标y
//	Line.interaction_figure[4].details_a = 1000;//
//	Line.interaction_figure[4].details_b = 130;//
        Line.interaction_figure[4].details_c = 27;//
//  Line.interaction_figure[4].details_d = 400;//
//	Line.interaction_figure[4].details_e = 300;//

        /*底盘指针*/
        Line.interaction_figure[5].figure_name[0] = 0x75;
        Line.interaction_figure[5].operate_tpye =  operate_; //修改图形
        Line.interaction_figure[5].figure_tpye =  0; //直线
        Line.interaction_figure[5].layer = 7;//图层
        Line.interaction_figure[5].color = 3;//
        Line.interaction_figure[5].width = 4;
        Line.interaction_figure[5].start_x = 960;//图形坐标x960
        Line.interaction_figure[5].start_y = 540;//图形坐标y600
        Line.interaction_figure[5].details_d = ui_state.pointer[X_ZHOU];
        Line.interaction_figure[5].details_e = ui_state.pointer[Y_ZHOU];

        /*电容条*/
        if(Cap_Measure.cap_value>=180)
        {
            Line.interaction_figure[6].color = 2;
        }
        else if(Cap_Measure.cap_value>=140&&Cap_Measure.cap_value<180)
        {
            Line.interaction_figure[6].color = 1;
        }
        else
        {
            Line.interaction_figure[6].color = 3;
        }

        Line.interaction_figure[6].figure_name[0] = 0x76;//名字是为了区分图片内容
        Line.interaction_figure[6].operate_tpye =  operate_ ;//增加图形
        Line.interaction_figure[6].figure_tpye =  4;//圆弧
        Line.interaction_figure[6].layer = 4;//图层 4
//	Line.interaction_figure[6].color = 2;//紫红
        Line.interaction_figure[6].width = 15;//图形线宽
        Line.interaction_figure[6].start_x = 794;//
        Line.interaction_figure[6].start_y = 538;//
        Line.interaction_figure[6].details_a = 214;//起始角度
        Line.interaction_figure[6].details_b = 214+112* Cap_Measure.cap_value/240;//终止角度
        Line.interaction_figure[6].details_d = 219;
        Line.interaction_figure[6].details_e = 308;

    }

    memcpy(Tx_Data+13,(uint8_t*)&Line,105);

    /*帧尾整包校验*/
    Append_CRC16_Check_Sum(Tx_Data,120);
    /*DMA数据发送*/


    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TXE)==RESET);//不空标志位，RESET表示空闲
    HAL_UART_Transmit_DMA(&huartx,Tx_Data,120);
    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TC)==RESET);//传输完成标志位

    vTaskDelay(40);

}




/*UI发送一个字母*/
void UI_TEST_Character(char V1,uint32_t x ,uint32_t y,uint8_t name ,uint8_t operate_ )
{
    uint16_t Cmd_ID;
    memset(&character,0,sizeof(character));
    memset(Tx_Data,0,128);

    //uint8_t i=1;
    /*帧头 5byte*/
    Tx_Header.Sof = 0xA5;
    Tx_Header.Seq = 0;
    Tx_Header.Data_length = sizeof(UIdata_Header)+sizeof(character);//sizeof(UIdata_Header) + sizeof(One_BigBig_Circle)
    memcpy(Tx_Data,&Tx_Header,5);
    Append_CRC8_Check_Sum(Tx_Data,5);

    /*命令ID  2byte*/
    Cmd_ID = 0x0301;
    memcpy(Tx_Data+5,(uint8_t *)&Cmd_ID,2);

    /*数据内容的帧头 6byte*/
    UIdata_Header.data_cmd_id = 0x0110;//换整形->
    UIdata_Header.send_ID = SendID;
    UIdata_Header.receiver_ID = ReceID;
    memcpy(Tx_Data+7,&UIdata_Header,sizeof(UIdata_Header));//6

    /*图形数据 45byte*/
    character.grapic_data_struct.graphic_name[0] = name;
    character.grapic_data_struct.operate_tpye =  operate_; //
    character.grapic_data_struct.graphic_tpye =  7; //整形->字符
    character.grapic_data_struct.layer = 7;//图层 7
    character.grapic_data_struct.color=3;//橙色
    character.grapic_data_struct.start_angle  =  30;//字体大小
// character.grapic_data_struct.end_angle  =  12;//字体长度
    character.grapic_data_struct.width =3;//线条宽度
    character.grapic_data_struct.start_x =x;
    character.grapic_data_struct.start_y =y;

    character.data[0]=V1;



    memcpy(Tx_Data+13,&character,sizeof(character));//45->

    /*帧尾整包校验*/
    Append_CRC16_Check_Sum(Tx_Data,sizeof(Tx_Header)+2+sizeof(UIdata_Header)+sizeof(character)+2);//62

    /*DMA数据发送*/

    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TXE)==RESET);//不空标志位，RESET表示空闲
    HAL_UART_Transmit_DMA(&huartx,Tx_Data,60);
    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TC)==RESET);//传输完成标志位

    vTaskDelay(40);
}

void UI_TEST_Character_BULLET(uint8_t operate_)
{
    uint16_t Cmd_ID;
    memset(&character,0,sizeof(character));
    memset(Tx_Data,0,128);

    //uint8_t i=1;
    /*帧头 5byte*/
    Tx_Header.Sof = 0xA5;
    Tx_Header.Seq = 0;
    Tx_Header.Data_length = sizeof(UIdata_Header)+sizeof(character);//sizeof(UIdata_Header) + sizeof(One_BigBig_Circle)
    memcpy(Tx_Data,&Tx_Header,5);
    Append_CRC8_Check_Sum(Tx_Data,5);

    /*命令ID  2byte*/
    Cmd_ID = 0x0301;
    memcpy(Tx_Data+5,(uint8_t *)&Cmd_ID,2);

    /*数据内容的帧头 6byte*/
    UIdata_Header.data_cmd_id = 0x0110;//换整形->
    UIdata_Header.send_ID = SendID;
    UIdata_Header.receiver_ID = ReceID;
    memcpy(Tx_Data+7,&UIdata_Header,sizeof(UIdata_Header));//6

    /*图形数据 45byte*/


    character.grapic_data_struct.graphic_name[0] = 0x40;
    character.grapic_data_struct.operate_tpye =  operate_; //
    character.grapic_data_struct.graphic_tpye =  7; //整形->字符
    character.grapic_data_struct.layer = 7;//图层 7
    character.grapic_data_struct.start_angle  =  25;//字体大小
    character.grapic_data_struct.color=8;//白色
// character.grapic_data_struct.end_angle  =  10;//字体长度
    character.grapic_data_struct.width =3;//线条宽度
    character.grapic_data_struct.start_x =1340;
    character.grapic_data_struct.start_y =711;

    character.data[0]='B';
    character.data[1]='U';
    character.data[2]='L';
    character.data[3]='L';
    character.data[4]='E';
    character.data[5]='T';


    memcpy(Tx_Data+13,&character,sizeof(character));//45->

    /*帧尾整包校验*/
    Append_CRC16_Check_Sum(Tx_Data,sizeof(Tx_Header)+2+sizeof(UIdata_Header)+sizeof(character)+2);//62

    /*DMA数据发送*/
// __HAL_UNLOCK(&huartx);
// HAL_UART_Transmit_DMA(&huartx,Tx_Data,sizeof(Tx_Header)+2+sizeof(UIdata_Header)+sizeof(character)+2);//62
// __HAL_UNLOCK(&huartx);

    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TXE)==RESET);//不空标志位，RESET表示空闲
    HAL_UART_Transmit_DMA(&huartx,Tx_Data,60);
    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TC)==RESET);//传输完成标志位

    vTaskDelay(40);
}



uint16_t RobotManReceiveID(void);
void Robot_Interact(void)
{
    uint16_t Cmd_ID;
//	memset(&Line,0,sizeof(Line));
    memset(Tx_Data,0,23);


    /*帧头 5byte*/
    Tx_Header.Sof = 0xA5;
    Tx_Header.Seq = 0;
    Tx_Header.Data_length = 14;//数据帧帧头（6）+0x0200的内容数据长度（8）
    memcpy(Tx_Data,(uint8_t*)&Tx_Header,5);//5
    Append_CRC8_Check_Sum(Tx_Data,5); //帧头

    /*命令ID  2byte*/
    Cmd_ID = 0x0301;
    memcpy(Tx_Data+5,(uint8_t *)&Cmd_ID,2);//2


    /*数据内容的帧头 6byte*/
    UIdata_Header.data_cmd_id = 0x0200;
    UIdata_Header.send_ID = SendID; //英雄（红）
    UIdata_Header.receiver_ID = RobotManReceiveID();//客户端英雄（红）
    memcpy(Tx_Data+7,(uint8_t*)&UIdata_Header,sizeof(UIdata_Header));//

    memcpy(Tx_Data+13,(uint8_t*)&jianshu,8);//105

    /*帧尾整包校验*/
    Append_CRC16_Check_Sum(Tx_Data,23);//120
    /*DMA数据发送*/
//	__HAL_UNLOCK(&huartx);
//	HAL_UART_Transmit_DMA(&huart4,Tx_Data,120);//120
//	__HAL_UNLOCK(&huartx);


    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TXE)==RESET);//不空标志位，RESET表示空闲
    HAL_UART_Transmit_DMA(&huartx,Tx_Data,23);
    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TC)==RESET);//传输完成标志位

    vTaskDelay(40);
}


void UI_NEW_INIT(void)
{

    static uint8_t seq=0;
    uint16_t Cmd_ID;
    memset(&Line,0,sizeof(Line));
    memset(Tx_Data,0,128);

    /*帧头 5byte*/
    Tx_Header.Sof = 0xA5;
    Tx_Header.Seq = 0;
    Tx_Header.Data_length = 111;//数据帧帧头（6）+0x0104的内容数据长度（105）
    memcpy(Tx_Data,(uint8_t*)&Tx_Header,5);//5
    Append_CRC8_Check_Sum(Tx_Data,5); //帧头

    /*命令ID  2byte*/
    Cmd_ID = 0x0301;
    memcpy(Tx_Data+5,(uint8_t *)&Cmd_ID,2);//2


    /*数据内容的帧头 6byte*/
    UIdata_Header.data_cmd_id = 0x0104;
    UIdata_Header.send_ID = SendID; //英雄（红）
    UIdata_Header.receiver_ID = ReceID;//客户端英雄（红）
    memcpy(Tx_Data+7,(uint8_t*)&UIdata_Header,sizeof(UIdata_Header));//6
#if 1


/*左车体斜线*/
    Line.interaction_figure[0].figure_name[0] = 0x00;//名字是为了区分图片内容
    Line.interaction_figure[0].operate_tpye =  1 ;//增加图形
    Line.interaction_figure[0].figure_tpye =  0; //直线
    Line.interaction_figure[0].layer = 4;//图层 4
    Line.interaction_figure[0].color = 8;//白
    Line.interaction_figure[0].width = 3;//图形线宽
    Line.interaction_figure[0].start_x = 500;//起点x坐标
    Line.interaction_figure[0].start_y = 160;//起点y坐标
    Line.interaction_figure[0].details_d = 660;//终点x坐标
    Line.interaction_figure[0].details_e = 280;//终点y坐标

#endif

#if 1
    /*显示右车体斜线*/
    Line.interaction_figure[1].figure_name[0] = 0x01;//名字是为了区分图片内容 ， 为了后面删除图片所用
    Line.interaction_figure[1].operate_tpye =  1 ;//增加图形
    Line.interaction_figure[1].figure_tpye =  0; //直线
    Line.interaction_figure[1].layer = 4;//图层 4
    Line.interaction_figure[1].color = 8;//白
    Line.interaction_figure[1].width = 3;//图形线宽
    Line.interaction_figure[1].start_x = 1260;//起点坐标x
    Line.interaction_figure[1].start_y = 280;//起点坐标y
    Line.interaction_figure[1].details_d = 1420;//终点坐标x
    Line.interaction_figure[1].details_e = 160;//终点坐标y


    /*左车体直线*/
    Line.interaction_figure[2].figure_name[0] = 0x02;//名字是为了区分图片内容
    Line.interaction_figure[2].operate_tpye =  1 ;//增加图形
    Line.interaction_figure[2].figure_tpye =  0; //直线
    Line.interaction_figure[2].layer = 4;//图层 4
    Line.interaction_figure[2].color = 8;//白色
    Line.interaction_figure[2].width = 3;//图形线宽
    Line.interaction_figure[2].start_x = 660;//起点坐标x
    Line.interaction_figure[2].start_y = 280;//起点坐标y
    Line.interaction_figure[2].details_d = 830;//终点坐标x
    Line.interaction_figure[2].details_e =280 ;//终点坐标y

    /*右车体直线*/
    Line.interaction_figure[3].figure_name[0] = 0x03;//名字是为了区分图片内容
    Line.interaction_figure[3].operate_tpye =  1 ;//增加图形
    Line.interaction_figure[3].figure_tpye =  0; //直线
    Line.interaction_figure[3].layer = 4;//图层 4
    Line.interaction_figure[3].color = 8;//白
    Line.interaction_figure[3].width = 3;//图形线宽
    Line.interaction_figure[3].start_x = 1090;//起点坐标x
    Line.interaction_figure[3].start_y = 280;//起点坐标y
    Line.interaction_figure[3].details_d = 1260;//终点坐标x
    Line.interaction_figure[3].details_e =280 ;	//终点坐标y
#endif

#if 1
    /*中心瞄准线*/
    Line.interaction_figure[4].figure_name[0] = 0x04;//名字是为了区分图片内容
    Line.interaction_figure[4].operate_tpye =  1 ;//增加图形
    Line.interaction_figure[4].figure_tpye =  0; //直线
    Line.interaction_figure[4].layer = 4;//图层 4
    Line.interaction_figure[4].color = 8;//白色
    Line.interaction_figure[4].width = 2;//图形线宽
    Line.interaction_figure[4].start_x = 955;//起点坐标x
    Line.interaction_figure[4].start_y = 500;//起点坐标y
    Line.interaction_figure[4].details_d = 955;//终点坐标x
    Line.interaction_figure[4].details_e =530 ;//终点坐标y

    Line.interaction_figure[5].figure_name[0] = 0x05;//名字是为了区分图片内容
    Line.interaction_figure[5].operate_tpye =  1 ;//增加图形
    Line.interaction_figure[5].figure_tpye =  0; //直线
    Line.interaction_figure[5].layer = 4;//图层 4
    Line.interaction_figure[5].color = 8;//白色
    Line.interaction_figure[5].width = 2;//图形线宽
    Line.interaction_figure[5].start_x = 940;//起点坐标x
    Line.interaction_figure[5].start_y = 515;//起点坐标y
    Line.interaction_figure[5].details_d = 970;//终点坐标x
    Line.interaction_figure[5].details_e =515 ;//终点坐标y

#endif
    memcpy(Tx_Data+13,(uint8_t*)&Line,105);//105

    /*帧尾整包校验*/
    Append_CRC16_Check_Sum(Tx_Data,120);//120

    /*DMA数据发送*/
    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TXE)==RESET);//不空标志位，RESET表示空闲
    HAL_UART_Transmit_DMA(&huartx,Tx_Data,120);
    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TC)==RESET);//传输完成标志位

    vTaskDelay(40);
}


void UI_MODE_BUILD(void)
{
    static uint8_t seq=0;
    uint16_t Cmd_ID;
    memset(&Line,0,sizeof(Line));
    memset(Tx_Data,0,128);


    /*帧头 5byte*/
    Tx_Header.Sof = 0xA5;
    Tx_Header.Seq = 0;
    Tx_Header.Data_length = 111;//sizeof(UIdata_Header) + sizeof(One_BigBig_Circle)
    memcpy(Tx_Data,(uint8_t*)&Tx_Header,5);//5
    Append_CRC8_Check_Sum(Tx_Data,5); //帧头

    /*命令ID  2byte*/
    Cmd_ID = 0x0301;
    memcpy(Tx_Data+5,(uint8_t *)&Cmd_ID,2);//2


    /*数据内容的帧头 6byte*/
    UIdata_Header.data_cmd_id = 0x0104;
    UIdata_Header.send_ID = SendID; //英雄（红）
    UIdata_Header.receiver_ID = ReceID;//客户端英雄（红）
    memcpy(Tx_Data+7,(uint8_t*)&UIdata_Header,sizeof(UIdata_Header));//6
#if 1

//	/*B斜线*/
//	Line.interaction_figure[0].figure_name[0] = 0x20;//名字是为了区分图片内容
//	Line.interaction_figure[0].operate_tpye =  1 ;//增加图形
//    Line.interaction_figure[0].figure_tpye =  4;
//	Line.interaction_figure[0].layer = 4;//图层 4
//	Line.interaction_figure[0].color = 5;//紫红
//	Line.interaction_figure[0].width = 10;//图形线宽
//	Line.interaction_figure[0].details_a = 100;//半径
//	Line.interaction_figure[0].details_b = 130;//半径
//	Line.interaction_figure[0].start_x = 960;//图形坐标x
//	Line.interaction_figure[0].start_y = 540;//图坐标y
//    Line.interaction_figure[0].details_d = 100;//斜率?°
//	Line.interaction_figure[0].details_e = 100;
//
//	/*G斜线*/
//	Line.interaction_figure[1].figure_name[0] = 0x21;//名字是为了区分图片内容
//	Line.interaction_figure[1].operate_tpye =  1 ;//增加图形
//    Line.interaction_figure[1].figure_tpye =  4;
//	Line.interaction_figure[1].layer = 4;//图层 4
//	Line.interaction_figure[1].color = 5;//紫红
//	Line.interaction_figure[1].width = 10;//图形线宽
//	Line.interaction_figure[1].details_a = 50;//半径
//	Line.interaction_figure[1].details_b = 80;//半径
//	Line.interaction_figure[1].start_x = 960;//图形坐标x
//	Line.interaction_figure[1].start_y = 540;//图坐标y
//    Line.interaction_figure[1].details_d = 100;//斜率?°
//	Line.interaction_figure[1].details_e = 100;
//
//	/*F斜线*/
//	Line.interaction_figure[2].figure_name[0] = 0x22;//名字是为了区分图片内容
//	Line.interaction_figure[2].operate_tpye =  1 ;//增加图形
//    Line.interaction_figure[2].figure_tpye =  4;
//	Line.interaction_figure[2].layer = 4;//图层 4
//	Line.interaction_figure[2].color = 5;//紫红
//	Line.interaction_figure[2].width = 10;//图形线宽
//	Line.interaction_figure[2].details_a = 280;//半径
//	Line.interaction_figure[2].details_b = 310;//半径
//	Line.interaction_figure[2].start_x = 960;//图形坐标x
//	Line.interaction_figure[2].start_y = 540;//图坐标y
//    Line.interaction_figure[2].details_d = 100;//斜率?°
//	Line.interaction_figure[2].details_e = 100;

/*中心瞄准线*/
//	Line.interaction_figure[3].figure_name[0] = 0x23;//名字是为了区分图片内容
//	Line.interaction_figure[3].operate_tpye =  1 ;//增加图形
//    Line.interaction_figure[3].figure_tpye =  0;
//	Line.interaction_figure[3].layer = 4;//图层 4
//	Line.interaction_figure[3].color = 1;//白色
//	Line.interaction_figure[3].width = 1;//图形线宽
//	Line.interaction_figure[3].start_x = 935;//起点坐标x
//	Line.interaction_figure[3].start_y = 540;//起点坐标y
//    Line.interaction_figure[3].details_d = 985;//终点坐标x
//	Line.interaction_figure[3].details_e = 540 ;	//终点坐标y

    Line.interaction_figure[4].figure_name[0] = 0x24;//名字是为了区分图片内容
    Line.interaction_figure[4].operate_tpye =  1 ;//增加图形
    Line.interaction_figure[4].figure_tpye =  0; //直线
    Line.interaction_figure[4].layer = 4;//图层 4
    Line.interaction_figure[4].color = 8;//白色
    Line.interaction_figure[4].width = 3;//图形线宽
    Line.interaction_figure[4].start_x = 960;//起点坐标x
    Line.interaction_figure[4].start_y = 418;//起点坐标y
    Line.interaction_figure[4].details_d = 960;//终点坐标x
    Line.interaction_figure[4].details_e = 481 ;	//终点坐标y

    Line.interaction_figure[5].figure_name[0] = 0x25;//名字是为了区分图片内容
    Line.interaction_figure[5].operate_tpye =  1 ;//增加图形
    Line.interaction_figure[5].figure_tpye =  0; //直线
    Line.interaction_figure[5].layer = 4;//图层 4
    Line.interaction_figure[5].color = 8;//白色
    Line.interaction_figure[5].width = 3;//图形线宽
    Line.interaction_figure[5].start_x = 930;//起点坐标x
    Line.interaction_figure[5].start_y = 448;//起点坐标y
    Line.interaction_figure[5].details_d = 993;//终点坐标x
    Line.interaction_figure[5].details_e = 448 ;	//终点坐标y

#endif
    memcpy(Tx_Data+13,(uint8_t*)&Line,105);//105

    /*帧尾整包校验*/
    Append_CRC16_Check_Sum(Tx_Data,120);//120


    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TXE)==RESET);//不空标志位，RESET表示空闲
    HAL_UART_Transmit_DMA(&huartx,Tx_Data,120);
    while(__HAL_UART_GET_FLAG(&huartx,UART_FLAG_TC)==RESET);//传输完成标志位

    vTaskDelay(40);
}