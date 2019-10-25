#include "stm32f4xx_hal.h"

void lcd_init(void);   // initialize lcd
void lcd_send_cmd(char cmd);  // send command to the lcd
void lcd_send_data(char data);  // send data to the lcd
void lcd_send_string(char *str);  // send string to the lcd
void lcd_clear_display(void); // clear display
void lcd_goto_1_line(void); // cursor goto 1 line
void lcd_goto_2_line(void); // cursor goto 2 line
void lcd_gotoxy(uint8_t x, uint8_t y); //cursor goto x,y

void lcd_display_off_cursor_on(void);
void lcd_display_off(void);
void lcd_display_on(void) ;
void lcd_cursor_off(void);
void lcd_cursor_on(void);
void lcd_cursor_blink_off(void);
void lcd_cursor_blink_on(void);
void lcd_backlight_off(void);
void lcd_backlight_on(void);


/** Display size ------------------------------------------------------------ */
// Number of visible lines of the display (1 or 2/4)
#define LCD_LINES					2
// Visible characters per line of the display
#define LCD_DISP_LENGTH		16
// DDRAM address of first char of line 1
#define LCD_START_LINE_1	0x00
// DDRAM address of first char of line 2
#define LCD_START_LINE_2	0x40
// DDRAM address of first char of line 3
#define LCD_START_LINE_3	0x14
// DDRAM address of first char of line 4
#define LCD_START_LINE_4	0x54

#define LCD_SET_DDRAM_ADDRESS 0x80 //Set DDRAM address or coursor position on display

// commands
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_SHIFT_CURSOR_TO_LEFT 0x04
#define LCD_SHIFT_DISPLAY_TO_RIGHT 0x05
#define LCD_SHIFT_CURSOR_TO_RIGHT 0x06
#define LCD_SHIFT_DISPLAY_TO_LEFT 0x07
#define LCD_DISPLAY_OFF_CURSOR_OFF 0x08
#define LCD_DISPLAY_OFF_CURSOR_ON 0x0A
#define LCD_DISPLAY_ON_CURSOR_OFF 0x0C
#define LCD_DISPLAY_ON_CURSOR_BLINK 0x0E
#define LCD_DISPLAY_OFF_CURSOR_BLINK 0x0F
#define LCD_SHIFT_CURSOR_POSITION_TO_LEFT 0x10
#define LCD_SHIFT_CURSOR_POSITION_TO_RIGHT 0x14
#define LCD_GOTO_BEGINNING_OF_FIRST_LINE 0x80
#define LCD_GOTO_BEGINNING_OF_SECOND_LINE 0xC0
#define LCD_4_BIT_1_LINE_5_7_MATRIX 0x38 //	Function Set: 4-bit, 1 Line, 5x7 Dots
#define LCD_4_BIT_2_LINE_5_7_MATRIX 0x28 //Function Set: 4-bit, 2 Line, 5x7 Dots

#define LCD_DISPLAYCONTROL 0x08



// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

uint8_t _displaycontrol;
uint8_t _backlightval;
