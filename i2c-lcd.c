/** Put this in the src folder **/

#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  	// change your handler here accordingly
extern GPIO_InitTypeDef GPIO_InitStruct;  	// change your handler here accordingly
#define GPIO_PORT_LED GPIOD; 		//LCD Backlight GPIO PORT
#define GPIO_PIN_LED GPIO_PIN_13  	//LCD BAcklight GPIO PIN

#define SLAVE_ADDRESS_LCD 0x70 // change this according to ur setup

void lcd_send_cmd(char cmd) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	data_t[0] = data_u | 0x0C;  //en=1, rs=0
	data_t[1] = data_u | 0x08;  //en=0, rs=0
	data_t[2] = data_l | 0x0C;  //en=1, rs=0
	data_t[3] = data_l | 0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t*) data_t, 4,
			100);
}

void lcd_send_data(char data) {
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D;  //en=1, rs=0
	data_t[1] = data_u | 0x09;  //en=0, rs=0
	data_t[2] = data_l | 0x0D;  //en=1, rs=0
	data_t[3] = data_l | 0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t*) data_t, 4,
			100);
}

void lcd_init(void) {
	lcd_send_cmd(LCD_RETURN_HOME);
	lcd_send_cmd(LCD_4_BIT_2_LINE_5_7_MATRIX);
	lcd_send_cmd(LCD_DISPLAY_ON_CURSOR_OFF);
	lcd_send_cmd(LCD_GOTO_BEGINNING_OF_FIRST_LINE);

	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
}

void lcd_send_string(char *str) {
	while (*str)
		lcd_send_data(*str++);
}

void lcd_clear_display(void) {
	lcd_send_cmd(LCD_CLEAR_DISPLAY);
}

void lcd_goto_1_line(void) {
	lcd_send_cmd(LCD_GOTO_BEGINNING_OF_FIRST_LINE);
}

void lcd_goto_2_line(void) {
	lcd_send_cmd(LCD_GOTO_BEGINNING_OF_SECOND_LINE);
}

void lcd_display_off_cursor_on(void) {
	lcd_send_cmd(LCD_DISPLAY_OFF_CURSOR_ON);
}

// Turn the display on/off (quickly)
void lcd_display_off(void) {
	_displaycontrol &= ~LCD_DISPLAYON;
	lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_display_on(void) {
	_displaycontrol |= LCD_DISPLAYON;
	lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void lcd_cursor_off(void) {
	_displaycontrol &= ~LCD_CURSORON;
	lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor_on(void) {
	_displaycontrol |= LCD_CURSORON;
	lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_cursor_blink_off(void) {
	_displaycontrol &= ~LCD_BLINKON;
	lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor_blink_on(void) {
	_displaycontrol |= LCD_BLINKON;
	lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn the (optional) backlight off/on
void lcd_backlight_off(void) {
	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_RESET);
}

void lcd_backlight_on(void) {
	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_SET);
}
/**
 ******************************************************************************
 * @brief	Set LCD cursor to specific position.
 * @param	LCD column (x)
 * @param	LCD row (y)
 * @retval	None
 ******************************************************************************
 */
void lcd_gotoxy(uint8_t x, uint8_t y) {
#if LCD_LINES == 1
	lcd_send_cmd(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_1 + x));
#elif LCD_LINES == 2
	if (y == 0)
		lcd_send_cmd(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_1 + x));
	else
		lcd_send_cmd(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_2 + x));
#elif LCD_LINES == 4
	if (y == 0)
		lcd_send_cmd(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_1 + x));
	else if (y == 1)
		lcd_send_cmd(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_2 + x));
	else if (y == 2)
		lcd_send_cmd(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_3 + x));
	else
		lcd_send_cmd(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_4 + x));
#endif
}
