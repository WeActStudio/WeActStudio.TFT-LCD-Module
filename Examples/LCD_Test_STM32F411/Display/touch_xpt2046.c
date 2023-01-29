/*---------------------------------------
- WeAct Studio Official Link
- taobao: weactstudio.taobao.com
- aliexpress: weactstudio.aliexpress.com
- github: github.com/WeActTC
- gitee: gitee.com/WeAct-TC
- blog: www.weact-tc.cn
---------------------------------------*/

#include "touch_xpt2046.h"

#include <stdlib.h>
#include <math.h>
#include "gpio.h"
#include "spi.h"
#include "dwt_stm32_delay.h"
#include "lcd.h"

// xpt2046
// tpen pin  -> pb10
// tcs pin   -> pb12
// tsck pin  -> pb13
// tmiso pin -> pb14
// tmosi pin -> pb15

#define TP_CS_CLR HAL_GPIO_WritePin(TCS_GPIO_Port, TCS_Pin, GPIO_PIN_RESET)
#define TP_CS_SET HAL_GPIO_WritePin(TCS_GPIO_Port, TCS_Pin, GPIO_PIN_SET)
#define TP_PEN (HAL_GPIO_ReadPin(TPEN_GPIO_Port, TPEN_Pin))
#define DELAY HAL_Delay

static uint8_t CMD_RDX = TP_CMD_RDX;
static uint8_t CMD_RDY = TP_CMD_RDY;

_m_tp_dev tp_dev =
    {
        TP_Init,
        TP_Scan,
        TP_Adjust,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
};

uint16_t TP_Read_AD(uint8_t cmd)
{
    uint8_t rxbuf[2];
    uint8_t txbuf[2];
    uint16_t result;
    txbuf[0] = 0;
    txbuf[1] = 0;
    TP_CS_CLR;
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 10);
    DWT_Delay_us(6);
    HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, 2, 10);
    TP_CS_SET;
    result = rxbuf[0] << 8 | rxbuf[1];
    result >>= 3;
    result &= 0xfff;
    return result;
}

/*****************************************************************************
 * @name       :uint16_t TP_Read_XOY(uint8_t xy)
 * @date       :2022-11-20
 * @function   :Read the touch screen coordinates (x or y),
                                Read the READ_TIMES secondary data in succession
                                and sort the data in ascending order,
                                Then remove the lowest and highest number of LOST_VAL
                                and take the average
 * @parameters :xy:Read command(CMD_RDX/CMD_RDY)
 * @retvalue   :Read data
******************************************************************************/
#define READ_TIMES 5 //读取次数
#define LOST_VAL 1   //丢弃值
static uint16_t TP_Read_Data(uint8_t xy)
{
    uint16_t i, j;
    uint16_t buf[READ_TIMES];
    uint16_t sum = 0;
    uint16_t temp;
    for (i = 0; i < READ_TIMES; i++)
        buf[i] = TP_Read_AD(xy);
    for (i = 0; i < READ_TIMES - 1; i++) //排序
    {
        for (j = i + 1; j < READ_TIMES; j++)
        {
            if (buf[i] > buf[j]) //升序排列
            {
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }
    sum = 0;
    for (i = LOST_VAL; i < READ_TIMES - LOST_VAL; i++)
        sum += buf[i];
    temp = sum / (READ_TIMES - 2 * LOST_VAL);
    return temp;
}

uint32_t TP_Read_Vbat(void)
{
    uint32_t res;
    res = TP_Read_Data(TP_CMD_VBAT);
    res = res * 2500 * 4 / 4096;
    return res;
}

uint16_t TP_Read_AUXIN(void)
{
    uint32_t res;
    res = TP_Read_Data(TP_CMD_AUXIN);
    return res;
}

uint16_t TP_Read_Temp(void)
{
    uint32_t res;
    res = TP_Read_Data(TP_CMD_TEMP0);
    return res;
}

/*****************************************************************************
 * @name       :uint8_t TP_Read_XY(uint16_t *x,uint16_t *y)
 * @date       :2022-11-20
 * @function   :Read touch screen x and y coordinates,
                                The minimum value can not be less than 100
 * @parameters :x:Read x coordinate of the touch screen
                                y:Read y coordinate of the touch screen
 * @retvalue   :0-fail,1-success
******************************************************************************/
uint8_t TP_Read_XY(uint16_t *x, uint16_t *y)
{
    uint16_t xtemp, ytemp;
    xtemp = TP_Read_Data(CMD_RDX);
    ytemp = TP_Read_Data(CMD_RDY);
    // if(xtemp<100||ytemp<100)return 0;//读数失败
    *x = xtemp;
    *y = ytemp;
    return 1; //读数成功
}

/*****************************************************************************
 * @name       :uint8_t TP_Read_XY2(uint16_t *x,uint16_t *y)
 * @date       :2022-11-20
 * @function   :Read the touch screen coordinates twice in a row,
                                and the deviation of these two times can not exceed ERR_RANGE,
                                satisfy the condition, then think the reading is correct,
                                otherwise the reading is wrong.
                                This function can greatly improve the accuracy.
 * @parameters :x:Read x coordinate of the touch screen
                                y:Read y coordinate of the touch screen
 * @retvalue   :0-fail,1-success
******************************************************************************/
#define ERR_RANGE 50 //误差范围
uint8_t TP_Read_XY2(uint16_t *x, uint16_t *y)
{
    uint16_t x1, y1;
    uint16_t x2, y2;
    uint8_t flag;
    flag = TP_Read_XY(&x1, &y1);
    if (flag == 0)
        return (0);
    flag = TP_Read_XY(&x2, &y2);
    if (flag == 0)
        return (0);
    if (((x2 <= x1 && x1 < x2 + ERR_RANGE) || (x1 <= x2 && x2 < x1 + ERR_RANGE)) //前后两次采样在+-50内
        && ((y2 <= y1 && y1 < y2 + ERR_RANGE) || (y1 <= y2 && y2 < y1 + ERR_RANGE)))
    {
        *x = (x1 + x2) / 2;
        *y = (y1 + y2) / 2;
        return 1;
    }
    else
        return 0;
}

/*****************************************************************************
 * @name       :uint8_t TP_Scan(uint8_t tp)
 * @date       :2022-11-20
 * @function   :Scanning touch event
 * @parameters :tp:0-screen coordinate
                                     1-Physical coordinates(For special occasions such as calibration)
 * @retvalue   :Current touch screen status,
                                0-no touch
                                1-touch
******************************************************************************/
uint8_t TP_Scan(uint8_t tp)
{
    if (TP_PEN == 0) //有按键按下
    {
        if (tp)
            TP_Read_XY2(&tp_dev.x, &tp_dev.y);      //读取物理坐标
        else if (TP_Read_XY2(&tp_dev.x, &tp_dev.y)) //读取屏幕坐标
        {
            tp_dev.x = tp_dev.xfac * tp_dev.x + tp_dev.xoff; //将结果转换为屏幕坐标
            tp_dev.y = tp_dev.yfac * tp_dev.y + tp_dev.yoff;
        }
        if ((tp_dev.sta & TP_PRES_DOWN) == 0) //之前没有被按下
        {
            tp_dev.sta = TP_PRES_DOWN | TP_CATH_PRES; //按键按下
            tp_dev.x0 = tp_dev.x;                     //记录第一次按下时的坐标
            tp_dev.y0 = tp_dev.y;
        }
    }
    else
    {
        if (tp_dev.sta & TP_PRES_DOWN) //之前是被按下的
        {
            tp_dev.sta &= ~(1 << 7); //标记按键松开
        }
        else //之前就没有被按下
        {
            tp_dev.x0 = 0;
            tp_dev.y0 = 0;
            tp_dev.x = 0xffff;
            tp_dev.y = 0xffff;
        }
    }
    return tp_dev.sta & TP_PRES_DOWN; //返回当前的触屏状态
}

//////////////////////////////////////////////////////////////////////////
//保存在EEPROM里面的地址区间基址,占用13个字节(RANGE:SAVE_ADDR_BASE~SAVE_ADDR_BASE+12)
#define SAVE_ADDR_BASE 40
/*****************************************************************************
 * @name       :void TP_Save_Adjdata(void)
 * @date       :2022-11-20
 * @function   :Save calibration parameters
 * @parameters :None
 * @retvalue   :None
 ******************************************************************************/
void TP_Save_Adjdata(void)
{
    // uint32_t temp;
    // //保存校正结果!
    // temp=tp_dev.xfac*100000000;//保存x校正因素
    // AT24CXX_WriteLenByte(SAVE_ADDR_BASE,temp,4);
    // temp=tp_dev.yfac*100000000;//保存y校正因素
    // AT24CXX_WriteLenByte(SAVE_ADDR_BASE+4,temp,4);
    // //保存x偏移量
    // AT24CXX_WriteLenByte(SAVE_ADDR_BASE+8,tp_dev.xoff,2);
    // //保存y偏移量
    // AT24CXX_WriteLenByte(SAVE_ADDR_BASE+10,tp_dev.yoff,2);
    // //保存触屏类型
    // AT24CXX_WriteOneByte(SAVE_ADDR_BASE+12,tp_dev.touchtype);
    // temp=0X0A;//标记校准过了
    // AT24CXX_WriteOneByte(SAVE_ADDR_BASE+13,temp);
}

/*****************************************************************************
 * @name       :uint8_t TP_Get_Adjdata(void)
 * @date       :2022-11-20
 * @function   :Gets the calibration values stored in the EEPROM
 * @parameters :None
 * @retvalue   :1-get the calibration values successfully
                                0-get the calibration values unsuccessfully and Need to recalibrate
******************************************************************************/
uint8_t TP_Get_Adjdata(void)
{
    // uint32_ttempfac;
    // tempfac=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+13);//读取标记字,看是否校准过！
    // if(tempfac==0X0A)//触摸屏已经校准过了
    // {
    // 	tempfac=AT24CXX_ReadLenByte(SAVE_ADDR_BASE,4);
    // 	tp_dev.xfac=(float)tempfac/100000000;//得到x校准参数
    // 	tempfac=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+4,4);
    // 	tp_dev.yfac=(float)tempfac/100000000;//得到y校准参数
    //     //得到x偏移量
    // 	tp_dev.xoff=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+8,2);
    //     //得到y偏移量
    // 	tp_dev.yoff=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+10,2);
    // 	tp_dev.touchtype=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+12);//读取触屏类型标记
    // 	if(tp_dev.touchtype)//X,Y方向与屏幕相反
    // 	{
    // 		CMD_RDX=0X90;
    // 		CMD_RDY=0XD0;
    // 	}else				   //X,Y方向与屏幕相同
    // 	{
    // 		CMD_RDX=0XD0;
    // 		CMD_RDY=0X90;
    // 	}
    // 	return 1;
    // }
    return 0;
}

/*****************************************************************************
 * @name       :void TP_Drow_Touch_Point(uint16_t x,uint16_t y,uint16_t color)
 * @date       :2022-11-20
 * @function   :Draw a touch point,Used to calibrate
 * @parameters :x:Read x coordinate of the touch screen
                                y:Read y coordinate of the touch screen
                                color:the color value of the touch point
 * @retvalue   :None
******************************************************************************/
void TP_Drow_Touch_Point(uint16_t x, uint16_t y, uint16_t color)
{

    LCD_DrawLine(x - 12, y, x + 13, y, color); //横线
    LCD_DrawLine(x, y - 12, x, y + 13, color); //竖线
    LCD_DrawPoint(x + 1, y + 1, color);
    LCD_DrawPoint(x - 1, y + 1, color);
    LCD_DrawPoint(x + 1, y - 1, color);
    LCD_DrawPoint(x - 1, y - 1, color);
    LCD_DrawCircle(x, y, 6, color); //画中心圈
}

/*****************************************************************************
 * @name       :void TP_Draw_Big_Point(uint16_t x,uint16_t y,uint16_t color)
 * @date       :2022-11-20
 * @function   :Draw a big point(2*2)
 * @parameters :x:Read x coordinate of the point
                                y:Read y coordinate of the point
                                color:the color value of the point
 * @retvalue   :None
******************************************************************************/
void TP_Draw_Big_Point(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_DrawPoint(x, y, color); //中心点
    LCD_DrawPoint(x + 1, y, color);
    LCD_DrawPoint(x, y + 1, color);
    LCD_DrawPoint(x + 1, y + 1, color);
}

//提示字符串
const uint8_t *TP_REMIND_MSG_TBL = (uint8_t *)"Please use the stylus click the cross on the screen.The cross will always move until the screen adjustment is completed.";

/*****************************************************************************
 * @name       :void TP_Adj_Info_Show(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2,uint16_t x3,uint16_t y3,uint16_t fac)
 * @date       :2018-08-09
 * @function   :Display calibration results
 * @parameters :x0:the x coordinates of first calibration point
                                y0:the y coordinates of first calibration point
                                x1:the x coordinates of second calibration point
                                y1:the y coordinates of second calibration point
                                x2:the x coordinates of third calibration point
                                y2:the y coordinates of third calibration point
                                x3:the x coordinates of fourth calibration point
                                y3:the y coordinates of fourth calibration point
                                fac:calibration factor
 * @retvalue   :None
******************************************************************************/
void TP_Adj_Info_Show(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t fac)
{
    LCD_ShowString(40, 140, (uint8_t *)"x1:", RED, WHITE, 16, 1);
    LCD_ShowString(40 + 80, 140, (uint8_t *)"y1:", RED, WHITE, 16, 1);
    LCD_ShowString(40, 160, (uint8_t *)"x2:", RED, WHITE, 16, 1);
    LCD_ShowString(40 + 80, 160, (uint8_t *)"y2:", RED, WHITE, 16, 1);
    LCD_ShowString(40, 180, (uint8_t *)"x3:", RED, WHITE, 16, 1);
    LCD_ShowString(40 + 80, 180, (uint8_t *)"y3:", RED, WHITE, 16, 1);
    LCD_ShowString(40, 200, (uint8_t *)"x4:", RED, WHITE, 16, 1);
    LCD_ShowString(40 + 80, 200, (uint8_t *)"y4:", RED, WHITE, 16, 1);
    LCD_ShowString(40, 220, (uint8_t *)"fac is:", RED, WHITE, 16, 1);
    LCD_ShowIntNum(40 + 24, 140, x0, 4, RED, WHITE, 16);      //显示数值
    LCD_ShowIntNum(40 + 24 + 80, 140, y0, 4, RED, WHITE, 16); //显示数值
    LCD_ShowIntNum(40 + 24, 160, x1, 4, RED, WHITE, 16);      //显示数值
    LCD_ShowIntNum(40 + 24 + 80, 160, y1, 4, RED, WHITE, 16); //显示数值
    LCD_ShowIntNum(40 + 24, 180, x2, 4, RED, WHITE, 16);      //显示数值
    LCD_ShowIntNum(40 + 24 + 80, 180, y2, 4, RED, WHITE, 16); //显示数值
    LCD_ShowIntNum(40 + 24, 200, x3, 4, RED, WHITE, 16);      //显示数值
    LCD_ShowIntNum(40 + 24 + 80, 200, y3, 4, RED, WHITE, 16); //显示数值
    LCD_ShowIntNum(40 + 56, 220, fac, 3, RED, WHITE, 16);     //显示数值,该数值必须在95~105范围之内.
}

/*****************************************************************************
 * @name       :void TP_Adjust(void)
 * @date       :2022-11-20
 * @function   :Calibration touch screen and Get 4 calibration parameters
 * @parameters :None
 * @retvalue   :None
 ******************************************************************************/
void TP_Adjust(void)
{
    uint16_t pos_temp[4][2]; //坐标缓存值
    uint8_t cnt = 0;
    uint16_t d1, d2;
    uint32_t tem1, tem2;
    double fac;
    uint16_t outtime = 0;
    cnt = 0;
    uint16_t POINT_COLOR = BLUE;
    uint16_t BACK_COLOR = WHITE;
    LCD_Clear(WHITE);  //清屏
    POINT_COLOR = RED; //红色
    LCD_Clear(WHITE);  //清屏
    POINT_COLOR = BLACK;
    LCD_ShowString(10, 40, (uint8_t *)"Please use the stylus click", POINT_COLOR, BACK_COLOR, 16, 1); //显示提示信息
    LCD_ShowString(10, 56, (uint8_t *)"the cross on the screen.", POINT_COLOR, BACK_COLOR, 16, 1);    //显示提示信息
    LCD_ShowString(10, 72, (uint8_t *)"The cross will always move", POINT_COLOR, BACK_COLOR, 16, 1);  //显示提示信息
    LCD_ShowString(10, 88, (uint8_t *)"until the screen adjustment", POINT_COLOR, BACK_COLOR, 16, 1); //显示提示信息
    LCD_ShowString(10, 104, (uint8_t *)"is completed.", POINT_COLOR, BACK_COLOR, 16, 1);              //显示提示信息

    TP_Drow_Touch_Point(20, 20, RED); //画点1
    tp_dev.sta = 0;                   //消除触发信号
    tp_dev.xfac = 0;                  // xfac用来标记是否校准过,所以校准之前必须清掉!以免错误
    while (1)                         //如果连续10秒钟没有按下,则自动退出
    {
        tp_dev.scan(1);                          //扫描物理坐标
        if ((tp_dev.sta & 0xc0) == TP_CATH_PRES) //按键按下了一次(此时按键松开了.)
        {
            outtime = 0;
            tp_dev.sta &= ~(1 << 6); //标记按键已经被处理过了.

            pos_temp[cnt][0] = tp_dev.x;
            pos_temp[cnt][1] = tp_dev.y;
            cnt++;
            switch (cnt)
            {
            case 1:
                TP_Drow_Touch_Point(20, 20, WHITE);              //清除点1
                TP_Drow_Touch_Point(lcddev.width - 20, 20, RED); //画点2
                break;
            case 2:
                TP_Drow_Touch_Point(lcddev.width - 20, 20, WHITE); //清除点2
                TP_Drow_Touch_Point(20, lcddev.height - 20, RED);  //画点3
                break;
            case 3:
                TP_Drow_Touch_Point(20, lcddev.height - 20, WHITE);              //清除点3
                TP_Drow_Touch_Point(lcddev.width - 20, lcddev.height - 20, RED); //画点4
                break;
            case 4:                                          //全部四个点已经得到
                                                             //对边相等
                tem1 = abs(pos_temp[0][0] - pos_temp[1][0]); // x1-x2
                tem2 = abs(pos_temp[0][1] - pos_temp[1][1]); // y1-y2
                tem1 *= tem1;
                tem2 *= tem2;
                d1 = sqrt(tem1 + tem2); //得到1,2的距离

                tem1 = abs(pos_temp[2][0] - pos_temp[3][0]); // x3-x4
                tem2 = abs(pos_temp[2][1] - pos_temp[3][1]); // y3-y4
                tem1 *= tem1;
                tem2 *= tem2;
                d2 = sqrt(tem1 + tem2); //得到3,4的距离
                fac = (float)d1 / d2;
                if (fac < 0.95 || fac > 1.05 || d1 == 0 || d2 == 0) //不合格
                {
                    cnt = 0;
                    TP_Drow_Touch_Point(lcddev.width - 20, lcddev.height - 20, WHITE);                                                                                           //清除点4
                    TP_Drow_Touch_Point(20, 20, RED);                                                                                                                            //画点1
                    TP_Adj_Info_Show(pos_temp[0][0], pos_temp[0][1], pos_temp[1][0], pos_temp[1][1], pos_temp[2][0], pos_temp[2][1], pos_temp[3][0], pos_temp[3][1], fac * 100); //显示数据
                    continue;
                }
                tem1 = abs(pos_temp[0][0] - pos_temp[2][0]); // x1-x3
                tem2 = abs(pos_temp[0][1] - pos_temp[2][1]); // y1-y3
                tem1 *= tem1;
                tem2 *= tem2;
                d1 = sqrt(tem1 + tem2); //得到1,3的距离

                tem1 = abs(pos_temp[1][0] - pos_temp[3][0]); // x2-x4
                tem2 = abs(pos_temp[1][1] - pos_temp[3][1]); // y2-y4
                tem1 *= tem1;
                tem2 *= tem2;
                d2 = sqrt(tem1 + tem2); //得到2,4的距离
                fac = (float)d1 / d2;
                if (fac < 0.95 || fac > 1.05) //不合格
                {
                    cnt = 0;
                    TP_Drow_Touch_Point(lcddev.width - 20, lcddev.height - 20, WHITE);                                                                                           //清除点4
                    TP_Drow_Touch_Point(20, 20, RED);                                                                                                                            //画点1
                    TP_Adj_Info_Show(pos_temp[0][0], pos_temp[0][1], pos_temp[1][0], pos_temp[1][1], pos_temp[2][0], pos_temp[2][1], pos_temp[3][0], pos_temp[3][1], fac * 100); //显示数据
                    continue;
                } //正确了

                //对角线相等
                tem1 = abs(pos_temp[1][0] - pos_temp[2][0]); // x1-x3
                tem2 = abs(pos_temp[1][1] - pos_temp[2][1]); // y1-y3
                tem1 *= tem1;
                tem2 *= tem2;
                d1 = sqrt(tem1 + tem2); //得到1,4的距离

                tem1 = abs(pos_temp[0][0] - pos_temp[3][0]); // x2-x4
                tem2 = abs(pos_temp[0][1] - pos_temp[3][1]); // y2-y4
                tem1 *= tem1;
                tem2 *= tem2;
                d2 = sqrt(tem1 + tem2); //得到2,3的距离
                fac = (float)d1 / d2;
                if (fac < 0.95 || fac > 1.05) //不合格
                {
                    cnt = 0;
                    TP_Drow_Touch_Point(lcddev.width - 20, lcddev.height - 20, WHITE);                                                                                           //清除点4
                    TP_Drow_Touch_Point(20, 20, RED);                                                                                                                            //画点1
                    TP_Adj_Info_Show(pos_temp[0][0], pos_temp[0][1], pos_temp[1][0], pos_temp[1][1], pos_temp[2][0], pos_temp[2][1], pos_temp[3][0], pos_temp[3][1], fac * 100); //显示数据
                    continue;
                } //正确了
                //计算结果
                tp_dev.xfac = (float)(lcddev.width - 40) / (pos_temp[1][0] - pos_temp[0][0]);       //得到xfac
                tp_dev.xoff = (lcddev.width - tp_dev.xfac * (pos_temp[1][0] + pos_temp[0][0])) / 2; //得到xoff

                tp_dev.yfac = (float)(lcddev.height - 40) / (pos_temp[2][1] - pos_temp[0][1]);       //得到yfac
                tp_dev.yoff = (lcddev.height - tp_dev.yfac * (pos_temp[2][1] + pos_temp[0][1])) / 2; //得到yoff
                if (fabs(tp_dev.xfac) > 2 || fabs(tp_dev.yfac) > 2)                                  //触屏和预设的相反了.
                {
                    cnt = 0;
                    TP_Drow_Touch_Point(lcddev.width - 20, lcddev.height - 20, WHITE); //清除点4
                    TP_Drow_Touch_Point(20, 20, RED);                                  //画点1
                    LCD_ShowString(40, 26, (uint8_t *)"TP Need readjust!", POINT_COLOR, BACK_COLOR, 16, 1);
                    tp_dev.touchtype = !tp_dev.touchtype; //修改触屏类型.
                    if (tp_dev.touchtype)                 // X,Y方向与屏幕相反
                    {
                        CMD_RDX = 0X90;
                        CMD_RDY = 0XD0;
                    }
                    else // X,Y方向与屏幕相同
                    {
                        CMD_RDX = 0XD0;
                        CMD_RDY = 0X90;
                    }
                    continue;
                }
                POINT_COLOR = BLUE;
                LCD_Clear(WHITE);                                                                              //清屏
                LCD_ShowString(35, 110, (uint8_t *)"Touch Screen Adjust OK!", POINT_COLOR, BACK_COLOR, 16, 1); //校正完成
                DELAY(1000);
                TP_Save_Adjdata();
                LCD_Clear(WHITE); //清屏
                return;           //校正完成
            }
        }
        DELAY(10);
        outtime++;
        if (outtime > 1000)
        {
            TP_Get_Adjdata();
            break;
        }
    }
}

/*****************************************************************************
 * @name       :uint8_t TP_Init(void)
 * @date       :2022-11-20
 * @function   :Initialization touch screen
 * @parameters :None
 * @retvalue   :0-no calibration
                                1-Has been calibrated
******************************************************************************/
uint8_t TP_Init(void)
{
    TP_Read_XY(&tp_dev.x, &tp_dev.y); //第一次读取初始化
    if (TP_Get_Adjdata())
        return 0; //已经校准
    else          //未校准?
    {
        TP_Adjust(); //屏幕校准
        TP_Save_Adjdata();
    }
    TP_Get_Adjdata();
    return 1;
}
