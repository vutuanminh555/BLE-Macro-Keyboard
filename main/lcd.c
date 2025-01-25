#include "lcd.h"

void LCD_String (char *str){  
    for(int i = 0; str[i]!=0; i++){
        LCD_Data (str[i]);
    }
}

void LCD_String_xy (uint8_t row, uint8_t pos, char *str){
    uint8_t offset = 0;
	if (row == 0 && pos < 20){
	offset = 0x00;	
	}
	else if (row == 1 && pos < 20){
	offset = 0x40;	 
	}
	else if (row == 2 && pos < 20){
	offset = 0x14;
	}
	else if (row == 3 && pos < 20){
	offset = 0x54;
	}
	LCD_Command( (pos + offset) | (1ULL << 7)); // MSB = 1: cursor control
	LCD_String(str);
}

void LCD_Command (uint8_t cmnd){ // RS = 0
    gpio_set_level(RS,0);

// MSB 4 bit
    gpio_set_level(D4,((cmnd >> 4) & 1ULL));
    gpio_set_level(D5,((cmnd >> 5) & 1ULL));
    gpio_set_level(D6,((cmnd >> 6) & 1ULL));
    gpio_set_level(D7,((cmnd >> 7) & 1ULL));

    gpio_set_level(EN,0);
    ets_delay_us(1);
    gpio_set_level(EN,1);
    ets_delay_us(1); // enable pulse must be > 450 ns
    gpio_set_level(EN,0);
    ets_delay_us(100); // commands need > 37 us to settle 

// LSB 4 bit
    gpio_set_level(D4,(cmnd & 1ULL));
    gpio_set_level(D5,((cmnd >> 1) & 1ULL));
    gpio_set_level(D6,((cmnd >> 2) & 1ULL));
    gpio_set_level(D7,((cmnd >> 3) & 1ULL));


    gpio_set_level(EN,0);
    ets_delay_us(1);
    gpio_set_level(EN,1);
    ets_delay_us(1); // enable pulse must be > 450 ns
    gpio_set_level(EN,0);
    ets_delay_us(100); // commands need > 37 us to settle
}

void LCD_Data (uint8_t data){ // RS = 1
    gpio_set_level(RS,1);

// MSB 4 bit

    gpio_set_level(D4,((data >> 4) & 1ULL));
    gpio_set_level(D5,((data >> 5) & 1ULL));
    gpio_set_level(D6,((data >> 6) & 1ULL));
    gpio_set_level(D7,((data >> 7) & 1ULL));


    gpio_set_level(EN,0);
    ets_delay_us(1);
    gpio_set_level(EN,1);
    ets_delay_us(1); // enable pulse must be > 450 ns
    gpio_set_level(EN,0);
    ets_delay_us(100); // commands need > 37 us to settle

// LSB 4 bit
    gpio_set_level(D4,(data & 1ULL));
    gpio_set_level(D5,((data >> 1) & 1ULL));
    gpio_set_level(D6,((data >> 2) & 1ULL));
    gpio_set_level(D7,((data >> 3) & 1ULL));

    gpio_set_level(EN,0);
    ets_delay_us(1);
    gpio_set_level(EN,1);
    ets_delay_us(1); // enable pulse must be > 450 ns
    gpio_set_level(EN,0);
    ets_delay_us(100); // commands need > 37 us to settle
}

void LCD_Init(){
    ets_delay_us(50000); // startup delay min 40ms for power to rise above 2.7V 
    
    //gpio_set_level(EN, 0); // enable pin disabled at first
    ets_delay_us(1);

    // start in 8bit mode, try to set 4 bit mode
    LCD_Command(0x03); // 1st try 
    ets_delay_us(4500); // wait min 4.1ms 
    LCD_Command(0x03); // 2nd try 
    ets_delay_us(4500); // wait min 4.1ms
    LCD_Command(0x03); // 3rd try
    ets_delay_us(150);
    LCD_Command(0x02); // set to 4-bit interface


	LCD_Command(0x20 | 0x08 | 0x00); // LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS 
    LCD_Command(0x04 | 0x08); // turn the display on with no cursor or blinking default
    LCD_Command(0x01); // clear display
    ets_delay_us(2000); // clear display take long time
    LCD_Command(0x02 | 0x00 | 0x04); // initialize text direction and set entry mode

	LCD_String_xy(0, 0, " BLE Macro Keyboard ");
    LCD_String_xy(1, 0, "Mode: ");
    LCD_String_xy(2, 0, "Button: ");
}

void LCD_Deinit(){
    LCD_Command(0x08); // Clear Display 
    gpio_set_level(D4, 0);
    gpio_set_level(D5, 0);
    gpio_set_level(D6, 0);
    gpio_set_level(D7, 0);
    gpio_set_level(EN, 0);
    gpio_set_level(RS, 0);
    gpio_set_level(BL, 0);
}