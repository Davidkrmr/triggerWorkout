#include <EEPROM.h>
#include <SPI.h>

#define NUM_OUTPUTS 			4
#define OUT_REG 				DDRD
#define OUT_PORT 				PORTD
#define ALL_OFF 				(OUT_PORT &= ~(B11111100))
#define ALL_ON 					(OUT_PORT |= (B11111100))
#define NO_BUTTON_PRESSED 		0xFF
#define ADC_BITS 				10
#define ADC_DIV_SHIFT 			6
#define ADC_PROB_SHIFT 			3
#define NUM_DIVS 				(((1<<ADC_BITS))>>ADC_DIV_SHIFT)
#define RAND_SEED_ADC_PIN 		5
#define SW1 					B11111110
#define SW2 					B11111101
#define SW3 					B11111011
#define SW4 					B11110111
#define D_CLICK_TIME 			250
#define EEPROM_WRITE 			B11110011
#define EEPROM_READ 			B11111100
#define DIV_EEPROM_ADDR_ST 		0
#define PROB_EEPROM_ADDR_OFFSET 5
#define SW_DEBOUNCE 			25
#define GATE_LENGTH				B11110110
#define EEPROM_WRITE_FLAG		1
#define EEPROM_READ_FLAG		2
#define GATE_PROB_TOGGLE		4

//SPI defines
#define ADDR_ENABLE				0b00001000
#define OPCODEW					0b01000000
#define OPCODER       			0b01000001
#define IOCON					0x0A
#define ADDRESS 				0
#define IODIRA					0x00
#define GPPUA					0x0C
#define GPIOA					0x12
#define GPIOB					0x13


uint8_t div_table[NUM_DIVS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 32, 64, 128};

unsigned long clk_prev_time;
unsigned long clk_curr_time;
unsigned long out_prev_time[4];

//Random variables
uint8_t rand_num;
uint8_t prob[NUM_OUTPUTS] = {0, 0, 0, 0};

//Output pins
const uint8_t clk_out = 2;
const uint8_t o1 = 3;
const uint8_t o2 = 4;
const uint8_t o3 = 5;
const uint8_t o4 = 6;
uint8_t out[NUM_OUTPUTS] = {o1, o2, o3, o4};

//Select switches pins
const uint8_t s1 = 0;
const uint8_t s2 = 1;
const uint8_t s3 = 2;
const uint8_t s4 = 3;

//Array holding division for each output
uint8_t divs[NUM_OUTPUTS] = {1, 1, 1, 1};

uint8_t out_state[4];
uint8_t clk_out_state;
uint8_t output;

uint32_t trig_time[4] = {20, 20, 20, 20};
uint16_t trig_adc[4];
uint16_t clk_trig_time = 20;
uint8_t clk_in;

unsigned long cntr;

unsigned long d_click_int1;
unsigned long d_click_int2;

uint16_t clock_adc;
uint8_t scaled_div_adc;
uint8_t scaled_prob_adc;
uint16_t val;

uint8_t compare_div;
uint8_t compare_prob;

struct b_type
{
	uint8_t curr_btn_state;
	uint8_t prev_btn_state;
	uint8_t button_pressed;
	uint8_t d_click_detected;
}buttons, *pButtons;

uint8_t button_index;
unsigned long db_timer;

uint8_t prob_length_toggle;
uint8_t flag_reg;
uint16_t bpm;

void setup()
{
	randomSeed(analogRead(RAND_SEED_ADC_PIN));
	OUT_REG |= B11111100;
	init_spi();
	pButtons = &buttons;
}

void loop()
{

	clk_curr_time = millis();

	clock_adc = ((1<<10) - analogRead(2));
	scaled_div_adc = div_table[(analogRead(0)>>ADC_DIV_SHIFT)];
	scaled_prob_adc = map((analogRead(1)>>ADC_PROB_SHIFT), 0, 127, 0, 100);

	for(output = 0; output<NUM_OUTPUTS; output++){
		trig_time[output] = map(trig_adc[output], 0, 1023, 10, (clock_adc -10)) * divs[output];
	}

	
/*******************************************************************
*Monitor button read register. If any b is pressed, indicate with 
*buttonPress and set wait flags. These will keep the current settings
*for the selected outport until a pot is changed.
*Convert value of register to index.
*******************************************************************/

	if(checkButtons()){

		if (pButtons->curr_btn_state != pButtons->prev_btn_state) {
			db_timer = millis();
		}

		if((millis() - db_timer) > SW_DEBOUNCE) {

			button_index = get_button();

			if(!pButtons->button_pressed) {
				pButtons->button_pressed = true;
				compare_div = scaled_div_adc;
				compare_prob = scaled_prob_adc;
				val = divs[button_index];
			}

			if(compare_div != scaled_div_adc) {
				divs[button_index] = scaled_div_adc;
				compare_div = scaled_div_adc;
				val = scaled_div_adc;
			}

			if(compare_prob != scaled_prob_adc) {
				if(!prob_length_toggle) {
					prob[button_index] = scaled_prob_adc;
					compare_prob = scaled_prob_adc;
					val = scaled_prob_adc;
					} 
				else
					trig_adc[button_index] = analogRead(1);

				}
			writeDisplay(val);
			}
		}
	else{
		d_click_int2 = d_click_int1;
		pButtons->button_pressed = false;
		flag_reg = 0;
		writeDisplay(60000 / clock_adc);
	}

	//Poll eeprom write/read
	if(pButtons->curr_btn_state == EEPROM_WRITE){
		if (!(flag_reg & EEPROM_WRITE_FLAG)) {
			eepromWrite();
			flag_reg |= EEPROM_WRITE_FLAG;
		}
	}

	if(pButtons->curr_btn_state == EEPROM_READ){
		if (!(flag_reg & EEPROM_READ_FLAG)) {
			eepromRead();
			flag_reg |= EEPROM_READ_FLAG;
		}
	}

	if(pButtons->curr_btn_state == GATE_LENGTH){
		if (!(flag_reg & GATE_PROB_TOGGLE)) {
			prob_length_toggle ^= 1;
			flag_reg |= GATE_PROB_TOGGLE;
		}
	}
	
	//Master clk_in
	if((clk_curr_time - clk_prev_time) > clock_adc){
		clk_in = HIGH;
		clk_prev_time = clk_curr_time;
	}

/*********************************************************************************************
*If a clk_in is received, loop through outputs, check the divisions selected for the current out,
*check the probability stored for current out, if all ok -> turn on and set the out_state to HIGH
**********************************************************************************************/ 

	if(clk_in == HIGH){
		clk_in = LOW;

		rand_num = random(100);

		OUT_PORT |= (1<<clk_out);
		clk_out_state = HIGH;

		for(output=0; output<NUM_OUTPUTS; output++)
		{
			if(((cntr % divs[output]) == 0) && checkProb(rand_num, prob[output])){
				OUT_PORT |= (1<<out[output]);
				out_state[output] = HIGH;
				out_prev_time[output] = clk_prev_time;
			}
		}
		cntr++;
	}	

/*********************************************************************************************
*Loop through outputs, check if ON, then compare trig_time. If the trig/gate has been ON longer
*than the trig_time stored in trig_time array, turn output OFF
*
*The master clk_in out has a fixed trig length.
**********************************************************************************************/

	for(output = 0; output<NUM_OUTPUTS; output++)
	{
		if ((out_state[output] == HIGH) && (clk_curr_time - out_prev_time[output]) > trig_time[output]){
			OUT_PORT &= ~(1<<out[output]);
			out_state[output] = LOW;
		}
	}

	if((clk_out_state == HIGH) && (clk_curr_time - clk_prev_time) > clk_trig_time){
		if (!prob_length_toggle) {
			OUT_PORT &= ~(1<<clk_out);
		}
	}

//Store last state of buttons/switches
	pButtons->prev_btn_state = buttons.curr_btn_state;
}

/*********************************************************************************************
*SPI FUNCTIONS
**********************************************************************************************/
void spi_write_byte(uint8_t reg, uint8_t value)
{
  	PORTB &= 0b11111011;
  	SPI.transfer(OPCODEW | (ADDRESS << 1));
  	SPI.transfer(reg);                   
  	SPI.transfer(value);                   
  	PORTB |= 0b00000100;
}

void spi_write_word(uint8_t reg, uint16_t value) 
{
  PORTB &= 0b11111011;
  SPI.transfer(OPCODEW | (ADDRESS << 1));
  SPI.transfer(reg);
  SPI.transfer((uint8_t) value);
  SPI.transfer((uint8_t) (value >> 8));
  PORTB |= 0b00000100;
}

uint8_t spi_read_byte(uint8_t reg) {
  uint8_t value = 0;
  PORTB &= 0b11111011;
  SPI.transfer(OPCODER | (ADDRESS << 1));
  SPI.transfer(reg);
  value = SPI.transfer(0x00);
  PORTB |= 0b00000100;
  return value;
}

void init_spi()
{
	SPI.begin();
	SPI.setClockDivider(2);
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	spi_write_byte(IOCON, ADDR_ENABLE);
	spi_write_word(IODIRA, 0xFF00);
	spi_write_word(GPPUA, 0xFF00);
}

/*********************************************************************************************
*EEPROM FUNCTIONS
**********************************************************************************************/

void eepromRead()
{
	uint8_t i;

	for(i=0; i<NUM_OUTPUTS; i++){
		divs[i] = EEPROM.read(i);
		prob[i] = EEPROM.read(i+PROB_EEPROM_ADDR_OFFSET);
	}
	runLights();
}

void eepromWrite()
{
	uint8_t i;

	for(i=0; i<NUM_OUTPUTS; i++){
		EEPROM.write(i, divs[i]);
		EEPROM.write(i + PROB_EEPROM_ADDR_OFFSET, prob[i]);
	}
	runLights();
}


/*********************************************************************************************
*OTHER
**********************************************************************************************/

uint8_t checkButtons()
{
	pButtons->curr_btn_state = spi_read_byte(GPIOB);
	return pButtons->curr_btn_state != NO_BUTTON_PRESSED;
}

uint8_t checkProb(uint8_t num, uint8_t comp)
{
	return (num >= comp) ? true : false;
}


void runLights()
{
	uint8_t i;

	for(i=0; i<NUM_OUTPUTS; i++){
		OUT_PORT |= (1<<out[i]);
		delay(100);
	}
	for(i=0; i<NUM_OUTPUTS; i++){
		OUT_PORT &= ~(1<<out[i]);
		delay(100);
	}
}

uint8_t get_button()
{
	switch(pButtons->curr_btn_state){
		case SW1:
			return 0;
		break;
		case SW2:
			return 1;
		break;
		case SW3:
			return 2;
		break;
		case SW4:
			return 3;
		break;
	}
}

void writeDisplay(uint16_t num)
{
	static uint8_t i = 0;

	if(i >= 20)
	{
		i = 0;

		uint8_t d1 = num % 10;
		uint8_t d2 = (num/10) % 10;
		uint8_t d3 = (num/100)%10;
		uint8_t d4 = num/1000;

		if(d4)
	  		spi_write_byte(GPIOA, d4 | (1<<4));   
	  	if(d3 || (num > 999))
	  		spi_write_byte(GPIOA, d3 | (1<<5));
	  	if(d2 || (num > 99))   
	  		spi_write_byte(GPIOA, d2 | (1<<6));  
	  	
	  	spi_write_byte(GPIOA, d1 | (1<<7));
		spi_write_byte(GPIOA, 0);   
	}

	i += 1;
}
