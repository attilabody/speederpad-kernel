#ifndef __MACH_MY_ROCKCHIP_GPIOH
#define __MACH_MY_ROCKCHIP_GPIOH

#include <linux/proc_fs.h>
#include <linux/memory.h>


// /proc/rockchip 目录
extern struct proc_dir_entry *my_proc_root;
#if defined CONFIG_EEPROM_AT24
#define NOAH_SN_TYPE 								0x00
#define SN_START   							0x00
#define SN_OFFSET  							0x06

#define NOAH_WIFI_TYPE 								0x01
#define WIFI_START   							0x10
#define WIFI_OFFSET  							0x03

#define NOAH_GSENSOR_TYPE								0x02
#define GSENSOR_START								0x20
#define GSENSOR_OFFSET							0X04

#define NOAH_LIGHT_SENSOR_TYPE					0x03
#define LIGHT_SENSOR_START 				0X30
#define LIGHT_SENSOR_OFFSET 			0x08

#define NOAH_DISTANCE_V_SENSOR_TYPE 			0x04
#define DISTANCE_V_SENSOR_START			0X40
#define DISTANCE_V_SENSOR_OFFSET 		0x05

#define NOAH_DISTANCE_H_SENSOR_TYPE      0x05
#define DISTANCE_H_SENSOR_START			0X50
#define DISTANCE_H_SENSOR_OFFSET 		0x05
//-------------------------------------------
#define NOAH_DISTANCE_V_180_SENSOR_TYPE      0x06
#define DISTANCE_V_180_SENSOR_START			0X55
#define DISTANCE_V_180_SENSOR_OFFSET 		0x05

#define NOAH_PEN_TYPE 							0x07   //xinhb@20180828 主动笔，被动笔标志,   0x18字节表征， 
#define PEN_TYPE_START   					    0x18
#define PEN_TYPE_OFFSET  					    0x01

//-------------------------------------------
struct eeprom_hw_info{
	unsigned char all_data[257] ;
	unsigned char sys_sn_data[8];
	unsigned char sys_wifi_data[16] ;
	unsigned char sys_gsensor_data[16] ;	
	unsigned char sys_lightsensor_data[16] ;
	unsigned char sys_v_dissensor_data[16];
	unsigned char sys_h_dissensor_data[16];
	unsigned char sys_v_180_dissensor_data[16];
	unsigned char sys_pen_type[2];
	
};
							
#define WIFI_CREATEK_BCM4330  "B30"
#define WIFI_CREATEK_BCM4329  "B29"
#define WIFI_AMPAK "APK"

extern int noah_eeprom_setdata(int type,char * buf);	//type:NOAH_WIFI_TYPE,GSENSOR_TYPE,LIGHT_SENSOR_TYPE,DISTANCE_V_SENSOR_TYPE,DISTANCE_H_SENSOR_TYPE
extern int get_wifi_module_id(void);
extern char *noah_eeprom_readdata(int type);
void my_eeprom_memory_accessor(struct memory_accessor *mem_a, void *context);
#endif

#endif
