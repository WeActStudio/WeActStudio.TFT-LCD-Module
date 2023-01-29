#ifndef __touch_H__
#define __touch_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define TP_PRES_DOWN 0x80 //触屏被按下
#define TP_CATH_PRES 0x40 //有按键按下了

// bit2 = 1,enable 2.5V Vref output
#define TP_CMD_RDX (0xD2)
#define TP_CMD_RDY (0x92)
#define TP_CMD_VBAT (0xA6)
#define TP_CMD_TEMP0 (0x86)
#define TP_CMD_TEMP1 (0xF6)
#define TP_CMD_AUXIN (0xE6)

    //触摸屏控制器
    typedef struct
    {
        uint8_t (*init)(void);    //初始化触摸屏控制器
        uint8_t (*scan)(uint8_t); //扫描触摸屏.0,屏幕扫描;1,物理坐标;
        void (*adjust)(void);     //触摸屏校准
        uint16_t x0;              //原始坐标(第一次按下时的坐标)
        uint16_t y0;
        uint16_t x; //当前坐标(此次扫描时,触屏的坐标)
        uint16_t y;
        uint8_t sta; //笔的状态
                     // b7:按下1/松开0;
                     // b6:0,没有按键按下;1,有按键按下.
        ////////////////////////触摸屏校准参数/////////////////////////
        float xfac;
        float yfac;
        short xoff;
        short yoff;
        //新增的参数,当触摸屏的左右上下完全颠倒时需要用到.
        // touchtype=0的时候,适合左右为X坐标,上下为Y坐标的TP.
        // touchtype=1的时候,适合左右为Y坐标,上下为X坐标的TP.
        uint8_t touchtype;
    } _m_tp_dev;

    extern _m_tp_dev tp_dev; //触屏控制器在touch.c里面定义

    uint16_t TP_Read_AD(uint8_t cmd);
    uint8_t TP_Init(void);
    void TP_Adjust(void);
    uint8_t TP_Scan(uint8_t tp);
    uint32_t TP_Read_Vbat(void);
    uint16_t TP_Read_AUXIN(void);
    uint16_t TP_Read_Temp(void);
    
#ifdef __cplusplus
}
#endif
#endif
