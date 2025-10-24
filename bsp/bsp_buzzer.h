#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H
#include "struct_typedef.h"
#define MAX_PSC                 1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000


// �趨����ӳ���ϵͳʱ�ӣ���λ��Hz��
#define TIMER_CLK 84000000
#define BUZZER_PERIOD 20999  // �̶�ARR = 1000����pwm���ã�

// ������Ƶ�ʱ�C�������λHz��
typedef struct {
    uint16_t freq;
    uint16_t duration; // ms
} Note;



#define MELODY_LEN (sizeof(melody_max) / sizeof(Note))
extern void buzzer_on(uint16_t psc, uint16_t pwm);
extern void buzzer_off(void);
extern uint16_t freq_to_psc(uint16_t freq);
extern void play_max(void);
#endif
