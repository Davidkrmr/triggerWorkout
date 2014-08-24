#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include "MCP23S17.h"
int checkProb(uint8_t num, uint8_t comp);
void eepromRead();
void eepromWrite();
void runLightsDown();
uint8_t buttonReg2index(uint8_t button);
void writeDisplay(uint16_t num);
uint16_t deJitter(uint16_t v, uint16_t test);
void setup();
void loop();
#line 1 "src/trigger_riot.ino"
//Todo
//Debounce switches
//Moving average on raw ADC val instead of monitoring change of scaled val
//#include <EEPROM.h>
//#include <SPI.h>
//#include "MCP23S17.h"

#define NUM_OUTPUTS 4
/*#define SW_REG DDRB
#define SW_PORT PORTB
#define SW_READ_PORT PINB*/
#define OUT_REG DDRD
#define OUT_PORT PORTD
#define ALL_OFF (OUT_PORT &= ~(B11111100))
#define ALL_ON (OUT_PORT |= (B11111100))
#define NO_BUTTON_PRESSED 0xFF
#define ADC_BITS 10
#define ADC_DIV_SHIFT 6
#define ADC_PROB_SHIFT 3
#define NUM_DIVS (((1<<ADC_BITS))>>ADC_DIV_SHIFT)
#define RAND_SEED_ADC_PIN 5
#define SW1 B11111110
#define SW2 B11111101
#define SW3 B11111011
#define SW4 B11110111
#define D_CLICK_TIME 250
#define DIV_EEPROM_ADDR_ST 0;
#define PROB_EEPROM_ADDR_OFFSET 5
#define SW_DEBOUNCE 25


uint8_t divtable[NUM_DIVS] = {1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 16, 32, 64, 128};

unsigned long clkPrevTime;
unsigned long clkCurrTime;
unsigned long outPrevTime[4];

//Random variables
uint8_t randNum;
uint8_t prob[NUM_OUTPUTS] = {0, 0, 0, 0};

//Output pins
const uint8_t clockOut = 2;
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

uint8_t outState[4];
uint8_t clkOutState;
uint8_t output;

uint32_t trigTime[4] = {20, 20, 20, 20};
uint16_t trigAdc[4];
uint16_t clkTrigTime = 20;
uint8_t clock;

unsigned long counter;

unsigned long dClickInt1;
unsigned long dClickInt2;

uint16_t clockAdc;
uint8_t scaledDivAdc;
uint8_t scaledProbAdc;

uint8_t compareDiv;
uint8_t compareProb;
uint8_t divWait;
uint8_t probWait;

struct buttonType
{
	uint8_t currButtonState;
	uint8_t prevButtonState;
	uint8_t buttonPressed;
	uint8_t dClickDetected;
}buttons;

uint8_t b2i;
unsigned long dbTimer;

uint8_t eepromReadFlag;
uint8_t eepromWriteFlag;

uint8_t probLengthToggle;
uint8_t probLengthToggleFlag;

uint16_t bpm;

int checkProb(uint8_t num, uint8_t comp)
{
	return (num >= comp) ? true : false;
}

void eepromRead()
{
	uint8_t i;

	for(i=0; i<NUM_OUTPUTS; i++){
		divs[i] = EEPROM.read(i);
		prob[i] = EEPROM.read(i+PROB_EEPROM_ADDR_OFFSET);
	}
	runLightsDown();
}

void eepromWrite()
{
	uint8_t i;

	for(i=0; i<NUM_OUTPUTS; i++){
		EEPROM.write(i, divs[i]);
		EEPROM.write(i + PROB_EEPROM_ADDR_OFFSET, prob[i]);
	}
	runLightsDown();
}

void runLightsDown()
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


uint8_t buttonReg2index(uint8_t button)
{
	uint8_t i;

	switch(button){
		case SW1:
			i = 0;
		break;
		case SW2:
			i = 1;
		break;
		case SW3:
			i = 2;
		break;
		case SW4:
			i = 3;
		break;
	}
	return i;
}

MCP expander(0);

void writeDisplay(uint16_t num)
{
  expander.byteWrite(GPIOA, (num/1000) | (1<<4));   // write the output chip in word-mode, using our variable "value" as the argument
  expander.byteWrite(GPIOA, ((num/100)%10) | (1<<5));   // write the output chip in word-mode, using our variable "value" as the argument
  expander.byteWrite(GPIOA, ((num/10)%10) | (1<<6));   // write the output chip in word-mode, using our variable "value" as the argument
  expander.byteWrite(GPIOA, (num%10) | (1<<7));   // write the output chip in word-mode, using our variable "value" as the argument
  expander.byteWrite(GPIOA, 0);   // write the output chip in word-mode, using our variable "value" as the argument

}

uint16_t deJitter(uint16_t v, uint16_t test)
{
  if (abs(v - test) > 8) {
    return v;
  }
  return test;
}

void setup()
{
	randomSeed(analogRead(RAND_SEED_ADC_PIN));

	//clockOut
	OUT_REG |= B11111100;
	//SW_REG |= B11110000;
	//SW_PORT |= B00001111;

	expander.pinMode(0xFF00);
	expander.pullupMode(0xFF00);

	
}

void loop()
{

	clkCurrTime = millis();
	clockAdc = deJitter(((1<<10) - analogRead(2)), clockAdc);

	scaledDivAdc = divtable[(analogRead(0)>>ADC_DIV_SHIFT)];
	scaledProbAdc = (analogRead(1)>>ADC_PROB_SHIFT);
	scaledProbAdc = map(scaledProbAdc, 0, 127, 0, 100);

	for(output = 0; output<NUM_OUTPUTS; output++){
		trigTime[output] = map(trigAdc[output], 0, 1023, 10, (clockAdc -10)) * divs[output];
	}

	if(compareDiv != scaledDivAdc){
		divWait = false;
	}

	if(compareProb != scaledProbAdc){
		probWait = false;
	}

	// //Get trigTimes. Use prob ADC and commented out prob. Just for test
	// for(output = 0; output<NUM_OUTPUTS; output++){
	// 	trigTime[output] = (map(analogRead(1), 0, 1023, 1, (clockAdc - 1))) * divs[output];
	// }

/*******************************************************************
*Monitor button read register. If any b is pressed, indicate with 
*buttonPress and set wait flags. These will keep the current settings
*for the selected outport until a pot is changed.
*Convert value of register to index.
*******************************************************************/
	buttons.currButtonState = expander.byteRead(GPIOB);

	if(buttons.currButtonState != NO_BUTTON_PRESSED){

		if (buttons.currButtonState != buttons.prevButtonState) {
			dbTimer = millis();
		}

		if((millis() - dbTimer) > SW_DEBOUNCE) {

			b2i = buttonReg2index(buttons.currButtonState);

			dClickInt1 = millis();

			if(((dClickInt1 - dClickInt2) < D_CLICK_TIME) && !buttons.dClickDetected){
				buttons.dClickDetected = true;
				divs[b2i] = 1;
				prob[b2i] = 0;
			}

			else{
				if(!buttons.buttonPressed){
					buttons.buttonPressed = true;
					compareDiv = scaledDivAdc;
					compareProb = scaledProbAdc;
					divWait = true;
					probWait = true;
				}

				if(!divWait){
					divs[b2i] = scaledDivAdc;
					writeDisplay(scaledDivAdc);
				}
				if(!probWait){
					if(!probLengthToggle) {
						prob[b2i] = scaledProbAdc;
						writeDisplay(scaledProbAdc);
						}
					else
						trigAdc[b2i] = analogRead(1);

				}
			}
		}
	}
	else{
		dClickInt2 = dClickInt1;
		buttons.buttonPressed = false;
		buttons.dClickDetected = false;
		writeDisplay(bpm);
	}


	//Write to eeprom if button 3&4 are pressed.
	if((!(buttons.currButtonState & (1<<s4))) && (!(buttons.currButtonState & (1<<s3)))){
		if (!eepromWriteFlag) {
			eepromWrite();
			eepromWriteFlag = true;
		}
	}
	else {
		eepromWriteFlag = false;
	}

	if((!(buttons.currButtonState & (1<<s2))) && (!(buttons.currButtonState & (1<<s1)))){
		if (!eepromReadFlag) {
			eepromRead();
			eepromReadFlag = true;
		}
	}
	else {
		eepromReadFlag = false;
	}

	if((!(buttons.currButtonState & (1<<s4))) && (!(buttons.currButtonState & (1<<s1)))){
		if (!probLengthToggleFlag) {
			probLengthToggle ^= 1;
			probLengthToggleFlag = true;
		}
	}
	else {
		probLengthToggleFlag = false;
	}



	//Master clock
	if((clkCurrTime - clkPrevTime) > clockAdc){
		clock = HIGH;
		clkPrevTime = clkCurrTime;
	}

/*********************************************************************************************
*If a clock is received, loop through outputs, check the divisions selected for the current out,
*check the probability stored for current out, if all ok -> turn on and set the outstate to HIGH
**********************************************************************************************/ 

	if(clock == HIGH){
		clock = LOW;

		randNum = random(100);

		OUT_PORT |= (1<<clockOut);
		clkOutState = HIGH;


		for(output=0; output<NUM_OUTPUTS; output++)
		{
			if(((counter % divs[output]) == 0) && checkProb(randNum, prob[output])){
				OUT_PORT |= (1<<out[output]);
				outState[output] = HIGH;
				outPrevTime[output] = clkPrevTime;
			}
		}
		counter++;
	}	

/*********************************************************************************************
*Loop through outputs, check if ON, then compare trigTime. If the trig/gate has been ON longer
*than the trigTime stored in trigTime array, turn output OFF
*
*The master clock out has a fixed trig length.
**********************************************************************************************/

	for(output = 0; output<NUM_OUTPUTS; output++)
	{
		if ((outState[output] == HIGH) && (clkCurrTime - outPrevTime[output]) > trigTime[output]){
			OUT_PORT &= ~(1<<out[output]);
			outState[output] = LOW;
		}
	}

	if ((clkOutState == HIGH) && (clkCurrTime - clkPrevTime) > clkTrigTime){
		if (!probLengthToggle) {
			OUT_PORT &= ~(1<<clockOut);
		}
	}

//Store last state of buttons/switches
	buttons.prevButtonState = buttons.currButtonState;
	bpm = 60000 / clockAdc;
}

