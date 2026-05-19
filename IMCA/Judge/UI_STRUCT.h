#ifndef UI_STRUCT_H
#define UI_STRUCT_H
#include "main.h"
/*文件说明：这里放官方给的结构体*/


#if 0
UI图形  ↓
#endif
/*客户端删除图形*/
typedef struct
{
	uint8_t operate_tpye;
	uint8_t layer;
}__attribute__ ((packed)) ext_client_custom_graphic_delete_t;

/*新图形结构体*/
typedef struct
{
	uint8_t figure_name[3];
	uint32_t operate_tpye:3;
	uint32_t figure_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	int32_t details_a:9;
	int32_t details_b:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t details_c:10;
	uint32_t details_d:11;
	uint32_t details_e:11;
}__attribute__ ((packed))interaction_figure_t;

/*旧结构体，给字符用，因为看官方串口协议字符还是用旧结构体，但是实操发现两个结构体都能混着来用*/
typedef struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}__attribute__ ((packed))graphic_data_struct_t;



/*1.6.1的官方结构体*/
typedef struct
{
	interaction_figure_t interaction_figure[7];
}__attribute__ ((packed))interaction_figure_7t;

typedef struct
{
	uint8_t w;
	uint8_t s;
	uint8_t a;
	uint8_t d;
	uint8_t f;
	uint8_t g;
	uint8_t res1;
	uint8_t res2;
}__attribute__ ((packed))jianshu_t;


/*客户端绘制字符*/
typedef struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} __attribute__ ((packed))ext_client_custom_graphic_character_t;



#if 0
裁判系统接收数据协议 ↓
#endif
/*发送数据 数据帧头*/
typedef struct{

	uint8_t   Sof;  //数据帧起始字节，固定值为 0xA5
	uint16_t  Data_length;//数据帧中 data 的长度
	uint8_t   Seq;	//包序号
	uint8_t   Crc8;	//帧头 CRC8 校验

}__attribute__ ((packed))frame_header_;

#endif

