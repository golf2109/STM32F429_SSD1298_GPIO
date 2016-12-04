/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "NewFont.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
  uint16_t Display_ID;   
  uint32_t X;  // Эти две переменные у меня глобальные
  uint32_t Y; // от них ведется отсчет для следующего вывода букв
  uint32_t paper=0x0000, ink=0xffff; // Глобальные переменные - цвет фона и буквы  0000 - black  ffff - white
  
  uint16_t value[16]={0};
  uint16_t read_data;  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Delay_mks(void){
//uint32_t delay = 5;
//while(delay!=0){delay--;}
}


void MX_GPIO_Init_input(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin 
                           D8_Pin D9_Pin D10_Pin D11_Pin 
                           D12_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|D9_Pin|D10_Pin|D11_Pin 
                          |D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D13_Pin D14_Pin D15_Pin D0_Pin 
                           D1_Pin D2_Pin D3_Pin */
  GPIO_InitStruct.Pin = D13_Pin|D14_Pin|D15_Pin|D0_Pin 
                          |D1_Pin|D2_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);    
}

void MX_GPIO_Init_out(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|D9_Pin|D10_Pin|D11_Pin 
                          |D12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, D13_Pin|D14_Pin|D15_Pin|RS_Pin 
                          |D0_Pin|D1_Pin|D2_Pin|D3_Pin 
                          , GPIO_PIN_RESET);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin 
                           D8_Pin D9_Pin D10_Pin D11_Pin 
                           D12_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|D9_Pin|D10_Pin|D11_Pin 
                          |D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D13_Pin D14_Pin D15_Pin RS_Pin 
                           D0_Pin D1_Pin D2_Pin D3_Pin 
                           RD_Pin WR_Pin CS_Pin */
  GPIO_InitStruct.Pin = D13_Pin|D14_Pin|D15_Pin|RS_Pin 
                          |D0_Pin|D1_Pin|D2_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

// Процедура записи адреса регистра
void LCD_WriteIndex(uint16_t index){
HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);//  LCD_RS = 0; // Тип данных - команда (в данном случае адрес)
HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, GPIO_PIN_SET);//LCD_RD = 1; // При записи RD должен быть стопудово равен 1, а то не запишет
  
if((index & 0x0001) == 0x0001){
HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_RESET);}
if((index & 0x0002) == 0x0002){
HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);}
if((index & 0x0004) == 0x0004){
HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);}
if((index & 0x0008) == 0x0008){
HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);}
if((index & 0x0010) == 0x0010){
HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);}
if((index & 0x0020) == 0x0020){
HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);}
if((index & 0x0040) == 0x0040){
HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_RESET);}
if((index & 0x0080) == 0x0080){
HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_RESET);}
if((index & 0x0100) == 0x0100){
HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, GPIO_PIN_RESET);}
if((index & 0x0200) == 0x0200){
HAL_GPIO_WritePin(D9_GPIO_Port, D9_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D9_GPIO_Port, D9_Pin, GPIO_PIN_RESET);}
if((index & 0x0400) == 0x0400){
HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_RESET);}
if((index & 0x0800) == 0x0800){
HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, GPIO_PIN_RESET);}
if((index & 0x1000) == 0x1000){
HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, GPIO_PIN_RESET);}
if((index & 0x2000) == 0x2000){
HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, GPIO_PIN_RESET);}
if((index & 0x4000) == 0x4000){
HAL_GPIO_WritePin(D14_GPIO_Port, D14_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D14_GPIO_Port, D14_Pin, GPIO_PIN_RESET);}
if((index & 0x8000) == 0x8000){
HAL_GPIO_WritePin(D15_GPIO_Port, D15_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D15_GPIO_Port, D15_Pin, GPIO_PIN_RESET);}

HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_RESET);//  LCD_WR = 0; // Следующие три строки - строб /WR
HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_SET);//LCD_WR = 1;
}
 
// Процедура записи данных в регистр или ОЗУ
void LCD_WriteData(uint16_t data){
HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);//  LCD_RS = 1; // Тип данных - данные
HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, GPIO_PIN_SET);//LCD_RD = 1; // При записи RD должен быть стопудово равен 1, а то не запишет
  
if((data & 0x0001) == 0x0001){
HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_RESET);}
if((data & 0x0002) == 0x0002){
HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);}
if((data & 0x0004) == 0x0004){
HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);}
if((data & 0x0008) == 0x0008){
HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);}
if((data & 0x0010) == 0x0010){
HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);}
if((data & 0x0020) == 0x0020){
HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);}
if((data & 0x0040) == 0x0040){
HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_RESET);}
if((data & 0x0080) == 0x0080){
HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_RESET);}
if((data & 0x0100) == 0x0100){
HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, GPIO_PIN_RESET);}
if((data & 0x0200) == 0x0200){
HAL_GPIO_WritePin(D9_GPIO_Port, D9_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D9_GPIO_Port, D9_Pin, GPIO_PIN_RESET);}
if((data & 0x0400) == 0x0400){
HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, GPIO_PIN_RESET);}
if((data & 0x0800) == 0x0800){
HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, GPIO_PIN_RESET);}
if((data & 0x1000) == 0x1000){
HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, GPIO_PIN_RESET);}
if((data & 0x2000) == 0x2000){
HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, GPIO_PIN_RESET);}
if((data & 0x4000) == 0x4000){
HAL_GPIO_WritePin(D14_GPIO_Port, D14_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D14_GPIO_Port, D14_Pin, GPIO_PIN_RESET);}
if((data & 0x8000) == 0x8000){
HAL_GPIO_WritePin(D15_GPIO_Port, D15_Pin, GPIO_PIN_SET);}
else{HAL_GPIO_WritePin(D15_GPIO_Port, D15_Pin, GPIO_PIN_RESET);}  
  
HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_RESET);//  LCD_WR = 0; // Следующие три строки - строб /WR
HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_SET);//LCD_WR = 1;
}

void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue){
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);//  LCD_CS = 0; // Активируем Chip Select
  LCD_WriteIndex(LCD_Reg);  // Пишем адрес регистра
  LCD_WriteData(LCD_RegValue);  // Пишем в него данные
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);//  LCD_CS = 1; // Деактивируем Chip Select
}

// Процедура чтения из регистра или ОЗУ
uint16_t LCD_ReadReg(uint16_t LCD_Reg){
  uint16_t reg_val; 
  LCD_WriteIndex(LCD_Reg); // Пишем адрес регистра
  
HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_SET);//  LCD_WR = 1; // Вывод записи должен быть стопудово в 1   
HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, GPIO_PIN_RESET);//  LCD_RD = 0; // Выставляем сигнал чтения  
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);//LCD_CS = 0; // Активируем Chip Select 
MX_GPIO_Init_input();   
//Delay_mks();  
value[0] = HAL_GPIO_ReadPin(D0_GPIO_Port, D0_Pin);
value[1] = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin);
value[2] = HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin);
value[3] = HAL_GPIO_ReadPin(D3_GPIO_Port, D3_Pin);
value[4] = HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin);
value[5] = HAL_GPIO_ReadPin(D5_GPIO_Port, D5_Pin);
value[6] = HAL_GPIO_ReadPin(D6_GPIO_Port, D6_Pin);
value[7] = HAL_GPIO_ReadPin(D7_GPIO_Port, D7_Pin);
value[8] = HAL_GPIO_ReadPin(D8_GPIO_Port, D8_Pin);
value[9] = HAL_GPIO_ReadPin(D9_GPIO_Port, D9_Pin);
value[10] = HAL_GPIO_ReadPin(D10_GPIO_Port, D10_Pin);
value[11] = HAL_GPIO_ReadPin(D11_GPIO_Port, D11_Pin);
value[12] = HAL_GPIO_ReadPin(D12_GPIO_Port, D12_Pin);
value[13] = HAL_GPIO_ReadPin(D13_GPIO_Port, D13_Pin);  
value[14] = HAL_GPIO_ReadPin(D14_GPIO_Port, D14_Pin);  
value[15] = HAL_GPIO_ReadPin(D15_GPIO_Port, D15_Pin);  
read_data = 0;
read_data = (value[0]) || read_data;
read_data = (value[1] << 1) | read_data;
read_data = (value[2] << 2) | read_data;
read_data = (value[3] << 3) | read_data;
read_data = (value[4] << 4) | read_data;
read_data = (value[5] << 5) | read_data;
read_data = (value[6] << 6) | read_data;
read_data = (value[7] << 7) | read_data;
read_data = (value[8] << 8) | read_data;
read_data = (value[9] << 9) | read_data;
read_data = (value[10] << 10) | read_data;
read_data = (value[11] << 11) | read_data;
read_data = (value[12] << 12) | read_data;
read_data = (value[13] << 13) | read_data;
read_data = (value[14] << 14) | read_data;
read_data = (value[15] << 15) | read_data;

HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);//  LCD_CS = 1; // Деактивируем Chip Select           
MX_GPIO_Init_out();
HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, GPIO_PIN_SET);// LCD_RD = 1;   // Сбрасываем линию RD
//return read_data;  
reg_val = read_data;  

  return reg_val;
}

////////////////////////////////////////////////
void LCD_Initializtion(void){
  HAL_Delay(100);  // Выжидаем некоторое время (может потребоваться на больших частотах)
//  LCD_RES = 1;    // Деактивируем аппаратный сброс, который был активирован при настройке портов МК
  // Теперь записываем данные для настройки контроллера дисплея
  // Эти параметры я брал в интернете. Сначала происходит настрока внутреннего преобразователя
  // для питания стекляшки. Эти параметры я менять не советую...
  LCD_WriteReg(0x0028,0x0006);  // VCOM OTP
  LCD_WriteReg(0x0000,0x0001);  // Oscillation start
  LCD_WriteReg(0x0003,0xaea4);  // power control 1---line frequency and VHG,VGL voltage
  LCD_WriteReg(0x000c,0x0004);  // power control 2---VCIX2 output voltage
  LCD_WriteReg(0x000d,0x000c);  // power control 3---Vlcd63 voltage
  LCD_WriteReg(0x000e,0x2800);  // power control 4---VCOMA voltage VCOML=VCOMH*0.9475-VCOMA
  LCD_WriteReg(0x001e,0x00b5);  // POWER CONTROL 5---VCOMH voltage
  // Следующие параметры описаны в ДШ на стр. 30-31. Они определяют порядок сканирования строк,
  // порядок вывода цветов и др.
  LCD_WriteReg(0x0001,0x3B3F);  // Driver Output Control
  // Следующую строку также менять не советую
  LCD_WriteReg(0x0002,0x0600);  // LCD Drive AC Control
  // Регистр управления режимом Sleep. Может понадобиться для устройств с батарейным питанием
  // Для справки: Дисплей потребляет от 2,5 до 8 мА в активном режиме
  // и от 70 до 300 мкА в спящем (стр. 63 ДШ)
  LCD_WriteReg(0x0010,0x0000);  // Sleep Mode
  // Режим позиционирования и направления сканирования дисплея.
  // Для данных значений при расположении дисплея в Landscape контроллер справа
  // нижний левый угол будет иметь координаты 0,0
  // Я такие значения применил для вывода изображений из BMP файлов
 
  // Для Portrait положения дисплея строка будет иметь вид LCD_WriteReg(0x0011,0x6830);
  // Более подробно по данной установке можно прочитать на стр. 43-46 ДШ
  LCD_WriteReg(0x0011,0x6838);  // Entry Mode
  // Не нашел этих параметров в ДШ, но переписывать не стал...
  LCD_WriteReg(0x0005,0x0000);  // ???
  LCD_WriteReg(0x0006,0x0000);  // ???
  LCD_WriteReg(0x0016,0xef1c);  // ???
  // Следующие настройки можно посмотреть на стр. 39 ДШ
  LCD_WriteReg(0x0007,0x0033);  // Display control 1
    /* when GON=1 and DTE=0,all gate outputs become VGL */
    /* when GON=1 and DTE=0,all gate outputs become VGH */
    /* non-selected gate wires become VGL */
  // Далее расположены настройки размера видимой области дисплея. Их можно менять
  // по ходу работы при необходимости вывода в определенную область экрана
  LCD_WriteReg(0x000b,0x0000);  // Frame Cycle Control
  LCD_WriteReg(0x000f,0x0000);  // Gate Scan Start Position
  LCD_WriteReg(0x0041,0x0000);  // Vertical Scroll Control 1
  LCD_WriteReg(0x0042,0x0000);  // Vertical Scroll Control 2
  LCD_WriteReg(0x0048,0x0000);  // First Window Start
  LCD_WriteReg(0x0049,0x013f);  // First Window End
  LCD_WriteReg(0x004a,0x0000);  // Second Window Start
  LCD_WriteReg(0x004b,0x0000);  // Second Window End
  LCD_WriteReg(0x0044,0xef00);  // Horizontal RAM start and end address
  LCD_WriteReg(0x0045,0x0000);  // Vertical RAM start address
  LCD_WriteReg(0x0046,0x013f);  // Vertical RAM end address
  // Счетчики строк и столбцов. Я их меняю функцией LCD_SetCursor
  LCD_WriteReg(0x004e,0x0000);  // Set GDDRAM X address counter
  LCD_WriteReg(0x004f,0x0000);  // Set GDDRAM Y address counter
  // Управление гаммой цветов. Для предотвращения искажения не рекомендую менять их
  LCD_WriteReg(0x0030,0x0707);
  LCD_WriteReg(0x0031,0x0202);
  LCD_WriteReg(0x0032,0x0204);
  LCD_WriteReg(0x0033,0x0502);
  LCD_WriteReg(0x0034,0x0507);
  LCD_WriteReg(0x0035,0x0204);
  LCD_WriteReg(0x0036,0x0204);
  LCD_WriteReg(0x0037,0x0502);
  LCD_WriteReg(0x003a,0x0302);
  LCD_WriteReg(0x003b,0x0302);
  LCD_WriteReg(0x0023,0x0000);
  LCD_WriteReg(0x0024,0x0000);
  LCD_WriteReg(0x0025,0x8000);
  LCD_WriteReg(0x0026,0x7000);
  LCD_WriteReg(0x0020,0xb0eb);
  LCD_WriteReg(0x0027,0x007c);
  // После выполнения этой процедуры дисплей должен включиться и на экране появится
  // мусор в виде случайным образом засвеченных точек
}

// Процедура установки адресных регистров контроллера дисплея
void LCD_SetCursor(unsigned int Xpos, unsigned int Ypos ){

  X = Xpos; // Обновляем их новыми значениями
  Y = Ypos;
  LCD_WriteReg(0x004e, Ypos );  // Записываем новые значения в счетчики строк
  LCD_WriteReg(0x004f, Xpos );  // и столбцов
}

// Функция очистки дисплея заливает весь экран цветом, указанным в аргументе Color
void LCD_Clear(uint16_t Color)
{
  unsigned long index=0;
  LCD_SetCursor(0,0);		// Устанавливаем курсор в начало
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);//  LCD_CS = 0;			// Здесь запись организована "атомарно" для ускорения процедуры
  LCD_WriteIndex(0x0022); 	// Запись этого адреса в регистр индекса означает, что
                          	// сейчас будут выводиться данные в экранное ОЗУ
                          	// Как себя будут вести счетчики строк и столбцов определяется
                          	// регистром с адресом 0x0011
  for(index=0;index < 0x12C00;index++)
	{ 			// цикл с количеством итераций 320*240
    	LCD_WriteData(Color); 	// ставим точки заданного цвета
 	}
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);//  LCD_CS = 1;         		// Отцепляемся от дисплея
}

// Одна из сложнейших функций - вывод буквы в текущую позицию дисплея
void LCD_Putchar(uint16_t chr){  // Аргумент - код буквы 
  uint8_t i, a, tmp;
  uint16_t addr;
  switch(chr){  // Так я организовал переход на новую строку 
    case 10:
      Y -= 9;
    break;
    case 13:    // И "возврат каретки"
      X = 3;
    break;
    default:
      addr = chr << 3;
      tmp = Y;  // сохраняем позицию для возврата при отображении новой строки точек
      for(i=0;i<8;i++){     //Счетчик строки знакоместа
        LCD_WriteReg(0x004e,Y); // Установка координаты Y
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);//        LCD_CS = 0;         // Выводим строку точек
        LCD_WriteIndex(0x0022);
        for(a=0;a<8;a++){   //Счетчик столбцов знакоместа
          if(NewFont8x8[addr+a]&(0x80>>i)){ // Если в знакоместе точка = 1
            LCD_WriteData(ink);     // Выводим цвет ink
          }else{
            LCD_WriteData(paper);   // Иначе цвет фона
          }
        }
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);//        LCD_CS = 1;
        Y++;  // Переходим на новую строку знакоместа
      }
      Y = tmp;  // Восстанавливаем значение буквы по вертикали
      // Ниже расположено вычисление ширины буквы для более плотного расположения текста на экране
      i = 3;    
      for(a=7;a!=0;a--){
        if(NewFont8x8[addr+a] != 0x00){
          i = a + 2;
          break;
        }
      }
      // Переходим на новую строку если достигли правого края экрана
      X += i;
      if(X > 310){
        Y -= 9;
        X = 3;
      }
    break;
  }
  LCD_SetCursor(X, Y); // Координаты следует сохранить для вывода последующего символа
}
 
// Вывод строки на экран. В качестве аргумента - указатель на строку
void LCD_PutString(uint8_t* str){//
  uint16_t i=0;
  while(str[i]){  // Пока не встретим терминатора...
    LCD_Putchar(str[i]);
    i++;
  }
}
 
// Это я написал ввиду наличия строковых констант
void putString(const char* str){//
  uint16_t i=0;
  while(str[i]){
    LCD_Putchar(str[i]);
    i++;
  }
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
//while(1){  
//HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
//HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);  
//}
  
HAL_Delay(100);  
HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(EN_LED_GPIO_Port, EN_LED_Pin, GPIO_PIN_RESET);  

HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
  
HAL_Delay(500);
HAL_GPIO_WritePin(EN_LED_GPIO_Port, EN_LED_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);  
HAL_GPIO_WritePin(EN_LED_GPIO_Port, EN_LED_Pin, GPIO_PIN_SET);  

LCD_Initializtion();
LCD_Clear(0x1234);
HAL_Delay(100);
LCD_Clear(0x4321);
HAL_Delay(100);
Display_ID = LCD_ReadReg(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  {    
for(uint16_t qqq=0; qqq <= 0xfffe; qqq++){
paper=4*qqq;
ink=0xFFFF-4*qqq;  
  for(uint8_t iii=0;iii<=0xfe;iii++){  
  LCD_Putchar(iii);}
}

//HAL_Delay(10);       
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|D9_Pin|D10_Pin|D11_Pin 
                          |D12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, D13_Pin|D14_Pin|D15_Pin|RS_Pin 
                          |D0_Pin|D1_Pin|D2_Pin|D3_Pin 
                          |RD_Pin|WR_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RESET_Pin|EN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin 
                           D8_Pin D9_Pin D10_Pin D11_Pin 
                           D12_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin 
                          |D8_Pin|D9_Pin|D10_Pin|D11_Pin 
                          |D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D13_Pin D14_Pin D15_Pin RS_Pin 
                           D0_Pin D1_Pin D2_Pin D3_Pin 
                           RD_Pin WR_Pin CS_Pin */
  GPIO_InitStruct.Pin = D13_Pin|D14_Pin|D15_Pin|RS_Pin 
                          |D0_Pin|D1_Pin|D2_Pin|D3_Pin 
                          |RD_Pin|WR_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin EN_LED_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|EN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
