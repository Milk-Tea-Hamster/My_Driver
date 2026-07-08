#ifndef __KEY_HANDLER_H__
#define __KEY_HANDLER_H__

/* 按键 ID */
#define BTN_ID_DUTY  0   /* K1, PE9: 占空比调节（仅方波） */
#define BTN_ID_FREQ  1   /* K2, PE7: 频率调节 */
#define BTN_ID_WAVE  2   /* K3, PB0: 波形切换 */

void Key_Init(void);
void Key_Ticks(void);   /* 每 5ms 调用一次，在主循环或定时中断中调用 */

#endif /* __KEY_HANDLER_H__ */
