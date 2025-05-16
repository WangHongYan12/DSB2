#include "main.h"
#include "buzzer.h"
#include "stm32f1xx_hal_tim.h"
int i2 = 0;
/*============ TIM 句柄&基础参数 ==========================================*/
extern TIM_HandleTypeDef htim8;        /* CubeMX 生成 */

#define CNT_CLK_HZ   1000000UL         /* 1 MHz (PSC=71) */
#define DUTY_NUM     1                 /* 50 % 占空 */
#define DUTY_DEN     2

/*============ 全局节拍 ===================================================*/
#define TEMPO_BPM    180U              /* ♩ = 180  */
#define UNIT_MS      (60000U/TEMPO_BPM/4U)   /* 十六分音符 83 ms */

/*============ 音阶 (Hz) ==================================================*/
#define F4 349U
#define G4 392U
#define A4 440U
#define BB4 466U
#define C5 523U
#define D5 587U
#define E5 659U

typedef struct { uint16_t f; uint8_t d; } note_t;

/*====================== 5 组提示音 =======================================*/
static const note_t tone_beep_short[] = { {A4,1}, {0,1} };
static const note_t tone_beep_double[] = { {A4,1}, {0,1}, {A4,1}, {0,1} };
static const note_t tone_ascend[]      = { {D5,1}, {E5,1}, {F4,1}, {G4,2}, {0,1} };
static const note_t tone_descend[]     = { {G4,1}, {F4,1}, {E5,1}, {D5,2}, {0,1} };
static const note_t tone_error[]       = { {BB4,2}, {0,1}, {BB4,2}, {0,1}, {A4,4} };

static const note_t* const tone_tab[TONE_MAX] = {
        tone_beep_short, tone_beep_double, tone_ascend, tone_descend, tone_error
};
static const uint8_t tone_len[TONE_MAX] = {
        sizeof(tone_beep_short)/sizeof(note_t),
        sizeof(tone_beep_double)/sizeof(note_t),
        sizeof(tone_ascend     )/sizeof(note_t),
        sizeof(tone_descend    )/sizeof(note_t),
        sizeof(tone_error      )/sizeof(note_t)
};

/*====================== 加勒比海盗 · 主旋律开场 ==========================*/
/* 简谱：6 6 1 1 7 5 | 5 6 6 1 1 2 |（×2）                               */
static const note_t melody_pirates[] = {
        {D5,1},{D5,1},{F4,1},{F4,1},{E5,1},{C5,1},
        {C5,1},{D5,1},{D5,1},{F4,1},{F4,1},{G4,1},
        {E5,1},{0 ,1},{0 ,1},{0 ,1},            /* 小节休止，凑满 16 音 */
        /* --- 再来一次 --- */
        {D5,1},{D5,1},{F4,1},{F4,1},{E5,1},{C5,1},
        {C5,1},{D5,1},{D5,1},{F4,1},{F4,1},{G4,1},
        {E5,1},{0 ,1},{0 ,1},{0 ,1}
};

/* 简单开机音 */
static const note_t melody_startup[] = { {E5,2}, {G4,1}, {A4,3}, {0,2} };

static const note_t* const mel_tab[MELODY_MAX] = {
        melody_pirates,
        melody_startup
};
static const uint16_t mel_len[MELODY_MAX] = {
        sizeof(melody_pirates)/sizeof(note_t),
        sizeof(melody_startup)/sizeof(note_t)
};

/*===================== 内部函数：设定频率 ================================*/
static inline void set_freq(uint16_t hz)
{
    if (hz == 0) { HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3); return; }

    uint32_t arr = CNT_CLK_HZ / hz;
    __HAL_TIM_SET_AUTORELOAD(&htim8, arr-1);
    __HAL_TIM_SET_COMPARE (&htim8, TIM_CHANNEL_3, arr*DUTY_NUM/DUTY_DEN);
    htim8.Instance->EGR = TIM_EGR_UG;          /* 立即生效 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

/*===================== 对外 API 实现 =====================================*/
void Buzzer_Init(void) { HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3); }

void Buzzer_Stop(void) { HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3); }

void Buzzer_PlayFreq(uint16_t hz, uint16_t ms)
{
    set_freq(hz);
    HAL_Delay(ms);
    Buzzer_Stop();
}

void Buzzer_PlayTone(BuzzerTone_t id)
{
    if (id >= TONE_MAX) return;
    const note_t *s = tone_tab[id];  uint8_t n = tone_len[id];
    for (uint8_t i=0;i<n;++i) Buzzer_PlayFreq(s[i].f, s[i].d*UNIT_MS);
}

void Buzzer_PlayMelody(BuzzerMelody_t id)
{
    if (id >= MELODY_MAX) return;
    const note_t *s = mel_tab[id];  uint16_t n = mel_len[id];
    for (uint16_t i=0;i<n;++i) Buzzer_PlayFreq(s[i].f, s[i].d*UNIT_MS);
}
