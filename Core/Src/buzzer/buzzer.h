#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {          /* C++ 兼容 */
#endif
extern int i2;
/* 提示音枚举 --------------------------------------------------------------*/
typedef enum {
    TONE_BEEP_SHORT = 0,
    TONE_BEEP_DOUBLE,
    TONE_ASCEND,
    TONE_DESCEND,
    TONE_ERROR,
    TONE_MAX
} BuzzerTone_t;

/* 旋律枚举 --------------------------------------------------------------*/
typedef enum {
    MELODY_PIRATES = 0,      /* 加勒比海盗开场（16×2） */
    MELODY_STARTUP,          /* 开机三音 */
    MELODY_MAX
} BuzzerMelody_t;

/* API --------------------------------------------------------------------*/
void Buzzer_Init(void);
void Buzzer_PlayFreq (uint16_t hz, uint16_t ms);
void Buzzer_PlayTone (BuzzerTone_t id);
void Buzzer_PlayMelody(BuzzerMelody_t id);
void Buzzer_Stop(void);

#ifdef __cplusplus
}
#endif
#endif /* __BUZZER_H */
