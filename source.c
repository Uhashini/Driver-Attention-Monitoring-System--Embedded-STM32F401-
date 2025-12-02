#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>

#define PCF8574_ADDR 0x4E   // I2C LCD address (0x27 << 1)
#define SAMPLE_WINDOW     500
#define BPM_SAMPLES       5
#define MIN_BPM           30
#define MAX_BPM           200
#define MIN_BEAT_INTERVAL 300
#define MAX_BEAT_INTERVAL 2000

/* LCD Defines */
#define LCD_BACKLIGHT 0x08
#define ENABLE        0x04
#define RS_DATA       0x01
#define LCD_CMD       0
#define LCD_DATA      RS_DATA

volatile uint32_t millis = 0;
void SysTick_Handler(void){ millis++; }

static void SysTick_Init(void){
    SysTick->LOAD = 16000 - 1; // 1ms tick (16MHz)
    SysTick->VAL = 0;
    SysTick->CTRL = 0x07;
}

static void delay_ms(uint32_t ms){
    uint32_t s = millis;
    while((millis - s) < ms);
}

/* ---------- I2C1 Init ---------- */
static void I2C1_Init_custom(void){
    RCC->AHB1ENR |= (1<<1);
    RCC->APB1ENR |= (1<<21);

    GPIOB->MODER &= ~((3<<(8*2))|(3<<(9*2)));
    GPIOB->MODER |=  ((2<<(8*2))|(2<<(9*2)));
    GPIOB->OTYPER |= (1<<8)|(1<<9);
    GPIOB->OSPEEDR |= (3<<(8*2))|(3<<(9*2));
    GPIOB->PUPDR |= (1<<(8*2))|(1<<(9*2));
    GPIOB->AFR[1] &= ~((0xF<<0)|(0xF<<4));
    GPIOB->AFR[1] |= (4<<0)|(4<<4);

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

static void I2C_StartAddr(uint8_t addr){
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = addr;
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
}

static void I2C_WriteByte(uint8_t b){
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = b;
    while(!(I2C1->SR1 & I2C_SR1_BTF));
}

static void I2C_Stop(void){ I2C1->CR1 |= I2C_CR1_STOP; }

/* ---------- LCD ---------- */
static void LCD_Write4(uint8_t data_with_ctrl){
    I2C_StartAddr(PCF8574_ADDR);
    I2C_WriteByte(data_with_ctrl);
    I2C_Stop();
}

static void LCD_Send(uint8_t data, uint8_t mode){
    uint8_t high = data & 0xF0;
    uint8_t low  = (data << 4) & 0xF0;

    LCD_Write4(high | LCD_BACKLIGHT | ENABLE | mode);
    delay_ms(1);
    LCD_Write4(high | LCD_BACKLIGHT | mode);
    delay_ms(1);
    LCD_Write4(low | LCD_BACKLIGHT | ENABLE | mode);
    delay_ms(1);
    LCD_Write4(low | LCD_BACKLIGHT | mode);
    delay_ms(1);
}

static void LCD_Cmd(uint8_t cmd){ LCD_Send(cmd, LCD_CMD); delay_ms(2); }
static void LCD_Char(char c){ LCD_Send((uint8_t)c, LCD_DATA); }
static void LCD_String(const char *s){ while(*s) LCD_Char(*s++); }
static void LCD_Clear(void){ LCD_Cmd(0x01); delay_ms(3); }

static void LCD_SetCursor(uint8_t row, uint8_t col){
    uint8_t addr = (row==0)?(0x80+col):(0xC0+col);
    LCD_Cmd(addr);
}

static void LCD_Init_custom(void){
    delay_ms(50);
    LCD_Cmd(0x33); LCD_Cmd(0x32);
    LCD_Cmd(0x28); LCD_Cmd(0x0C);
    LCD_Cmd(0x06); LCD_Clear();
}

/* ---------- ADC1 (PA1) ---------- */
static void ADC1_Init_custom(void){
    RCC->AHB1ENR |= (1<<0);
    RCC->APB2ENR |= (1<<8);
    GPIOA->MODER |= (3<<(1*2));
    ADC1->CR2 = 0;
    ADC1->SMPR2 |= (7<<3);
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 1;
    ADC1->CR2 |= (1<<0);
    delay_ms(1);
}

static uint16_t ADC1_Read_custom(void){
    ADC1->CR2 |= (1<<30);
    while(!(ADC1->SR & (1<<1)));
    return (uint16_t)ADC1->DR;
}

/* ---------- GPIO ---------- */
static void GPIO_Init_custom(void){
    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2);
    GPIOA->MODER &= ~(3<<(0*2));
    GPIOB->MODER |= (1<<(0*2));
    GPIOC->MODER |= (1<<(13*2));
}

static inline void Buzzer_On(void){ GPIOB->BSRR = (1<<0); }
static inline void Buzzer_Off(void){ GPIOB->BSRR = (1<<(0+16)); }
static inline void LED_On(void){ GPIOC->BSRR = (1<<13); }
static inline void LED_Off(void){ GPIOC->BSRR = (1<<(13+16)); }

/* ---------- Pulse Detection ---------- */
typedef struct {
    uint16_t min, max, threshold;
    uint32_t last_beat_time;
    uint32_t intervals[BPM_SAMPLES];
    uint8_t index, valid, beat_detected;
    uint32_t bpm, last_update;
} PulseSensor_t;

static void Pulse_Init(PulseSensor_t *p){
    p->min = 4095; p->max = 0; p->threshold = 2048;
    p->last_beat_time = 0; p->index = 0; p->valid = 0;
    p->beat_detected = 0; p->bpm = 0; p->last_update = millis;
    for(int i=0;i<BPM_SAMPLES;i++) p->intervals[i]=0;
}

static void Pulse_Process(PulseSensor_t *p, uint16_t adc){
    uint32_t now = millis;
    if(now - p->last_update > SAMPLE_WINDOW){
        p->min = 4095; p->max = 0; p->last_update = now;
    }
    if(adc < p->min) p->min = adc;
    if(adc > p->max) p->max = adc;
    if(p->max > p->min + 50)
        p->threshold = p->min + ((p->max - p->min) * 7)/10;

    if(!p->beat_detected && adc > p->threshold){
        uint32_t interval = now - p->last_beat_time;
        if(p->last_beat_time && interval >= MIN_BEAT_INTERVAL && interval <= MAX_BEAT_INTERVAL){
            p->intervals[p->index] = interval;
            p->index = (p->index + 1) % BPM_SAMPLES;
            if(p->valid < BPM_SAMPLES) p->valid++;

            uint32_t sum=0;
            for(int i=0;i<p->valid;i++) sum += p->intervals[i];
            uint32_t avg = sum / p->valid;

            p->bpm = (avg>0)?60000/avg:0;
            if(p->bpm<MIN_BPM) p->bpm=MIN_BPM;
            if(p->bpm>MAX_BPM) p->bpm=MAX_BPM;
        }
        p->last_beat_time = now;
        p->beat_detected = 1;
    }

    if(p->beat_detected && adc < (p->threshold - 50)) p->beat_detected = 0;
    if(p->last_beat_time && (now - p->last_beat_time > 4000)){
        p->bpm = 0; p->valid = 0;
    }
}

/* ---------- Main ---------- */
int main(void){
    SysTick_Init();
    GPIO_Init_custom();
    I2C1_Init_custom();
    LCD_Init_custom();
    ADC1_Init_custom();

    PulseSensor_t pulse;
    Pulse_Init(&pulse);

    LCD_Clear();
    LCD_SetCursor(0,0); LCD_String("System Starting");
    delay_ms(1000);
    LCD_Clear();

    uint32_t last_display = 0;

    /* NEW VARIABLES FOR 5-SECOND EYE CLOSURE */
    static uint32_t eye_closed_start = 0;
    static uint8_t is_drowsy = 0;
    const uint32_t DROWSY_THRESHOLD = 5000; // 5 seconds

    while(1){
        uint16_t adc = ADC1_Read_custom();
        uint8_t eyes_open = (GPIOA->IDR & (1<<0)) ? 1 : 0;
        Pulse_Process(&pulse, adc);

        /* -------- 5 SECOND EYE-CLOSE DETECTION -------- */
        if(!eyes_open){
            if(eye_closed_start == 0)
                eye_closed_start = millis;

            if(!is_drowsy && (millis - eye_closed_start >= DROWSY_THRESHOLD)){
                is_drowsy = 1;
                LED_Off();
                Buzzer_On();
            }
        } else {
            eye_closed_start = 0;
            is_drowsy = 0;
            Buzzer_Off();
            LED_On();
        }

        /* -------- LCD Update -------- */
        if(millis - last_display >= 300){
            last_display = millis;
            LCD_Clear();

            LCD_SetCursor(0,0);
            char line1[17];
            const char *status = " ";
            if(pulse.bpm > 120)
                status = " High";
            else if(pulse.bpm < 60)
                status = " Low";
            else if(pulse.bpm != 0)
                status = " Norm";
            else
                status = "";

            if(pulse.bpm == 0)
                snprintf(line1, sizeof(line1), "BPM: --- %s", status);
            else
                snprintf(line1, sizeof(line1), "BPM:%3lu%s", (unsigned long)pulse.bpm, status);

            LCD_String(line1);

            LCD_SetCursor(1,0);
            if(is_drowsy)
                LCD_String("Eyes:Drowsy ");
            else
                LCD_String("Eyes:Normal ");
        }

        delay_ms(20);
    }
}
