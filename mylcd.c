#include "Nano100Series.h"
void lcd_print(uint8_t pos, char *s);
void lcd_init(void);
 
//------------------- LCD ------------------
#define RS  PC15
#define RW  PC14
#define E   PB15
 
static void delay(uint32_t t){
    int i;
    for(i=0; i < t; i++){}
}
//---
void lcd_cmd(uint8_t c){
    RS=0; RW=0;
    PD->DOUT = c;
    delay( 10 ); E=1; delay( 10 ); E=0;
    delay( 100 );
}

//---
void lcd_put(uint8_t c){
    RS=1; RW=0;
    PD->DOUT = c;
    delay( 10 ); E=1; delay( 10 ); E=0;
    delay( 100 );
}
#define LCD_ADDR(x)    lcd_cmd( 0x80 | x )
void lcd_print(uint8_t pos, char *s){
  lcd_cmd(0x80 | pos);
  while(*s){ lcd_put(*s++); }
}
 
//---
void lcd_init(void){
    int i;
    //--- GPIO
    PB->PMD |= 0x40000000;
    PC->PMD |= 0x50000000;
    PD->PMD |= 0x00005555;
    PD->DMASK |= 0x0000FF00;
    RS=0; RW=0; E=0;
    //--- example
    for(i=0;i<1000000;i++){} //step 1
    lcd_cmd(0x38);      //step 2
    lcd_cmd(0x0C);      //step 3
    lcd_cmd(0x06);      //step 4
    lcd_cmd(0x80);      //step 4.5
    //---
    lcd_print(0x00, "                ");
    lcd_print(0x40, "                ");
}
//---
void lcd_make(void){
    const char cc[56]={
        0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,        // 0
        0x1F,0x10,0x10,0x10,0x10,0x10,0x10,0x1F,        // 1
        0x1F,0x18,0x18,0x18,0x18,0x18,0x18,0x1F,        // 2
        0x1F,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1F,        // 3
        0x1F,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1F,        // 4
        0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,        // 5
        0x10,0x10,0x1C,0x1C,0x1C,0x1C,0x10,0x10         // 6
    };
    int i;
    lcd_cmd(0x40);
    for(i=0;i<56;i++){ lcd_put(cc[i]); }
    lcd_cmd(0x80);
}
//---
// val: 0~8
void lcd_bat(uint8_t pos, uint8_t val){
    int a,b;
    val ++;
    if(val>9){ val=9; }
    if(val > 5){ b = val -5; a = 5;}
    else { b=0; a=val; }
    lcd_cmd(0x80 | pos);
    lcd_put(a);
    lcd_put(b);
    lcd_put(6);
}
