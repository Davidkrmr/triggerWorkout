//Todo
//
//BUGS:
//FIX !divWait etc. When both pots moved on same button press, both IFs are true..



#include <EEPROM.h>
#include <SPI.h>
#include "MCP23S17.h"

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


uint8_t divtable[NUM_DIVS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 32, 64, 128};

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
uint8_t inClock;

unsigned long counter;

unsigned long dClickInt1;
unsigned long dClickInt2;

uint16_t clockAdc;
uint8_t scaledDivAdc;
uint8_t scaledProbAdc;
uint16_t val;

uint8_t compareDiv;
uint8_t compareProb;

struct buttonType
{
	uint8_t currButtonState;
	uint8_t prevButtonState;
	uint8_t buttonPressed;
	uint8_t dClickDetected;
}buttons, *pButtons;

uint8_t b2i;
unsigned long dbTimer;

uint8_t probLengthToggle;
uint8_t flagReg;
uint16_t bpm;

MCP expander(0);

void setup()
{
	randomSeed(analogRead(RAND_SEED_ADC_PIN));

	OUT_REG |= B11111100;

	expander.pinMode(0xFF00);
	expander.pullupMode(0xFF00);

	pButtons = &buttons;
}

void loop()
{

	clkCurrTime = millis();

	clockAdc = deJitter(((1<<10) - analogRead(2)), clockAdc);
	scaledDivAdc = divtable[(analogRead(0)>>ADC_DIV_SHIFT)];
	scaledProbAdc = map((analogRead(1)>>ADC_PROB_SHIFT), 0, 127, 0, 100);

	for(output = 0; output<NUM_OUTPUTS; output++){
		trigTime[output] = map(trigAdc[output], 0, 1023, 10, (clockAdc -10)) * divs[output];
	}

	
/*******************************************************************
*Monitor button read register. If any b is pressed, indicate with 
*buttonPress and set wait flags. These will keep the current settings
*for the selected outport until a pot is changed.
*Convert value of register to index.
*******************************************************************/

	if(checkButtons()){

		if (pButtons->currButtonState != pButtons->prevButtonState) {
			dbTimer = millis();
		}

		if((millis() - dbTimer) > SW_DEBOUNCE) {

			b2i = buttonReg2index();

			dClickInt1 = millis();

			if(((dClickInt1 - dClickInt2) < D_CLICK_TIME) && !pButtons->dClickDetected){
				pButtons->dClickDetected = true;
				divs[b2i] = 1;
				prob[b2i] = 0;
			}

			else{
				if(!pButtons->buttonPressed){
					pButtons->buttonPressed = true;
					compareDiv = scaledDivAdc;
					compareProb = scaledProbAdc;
					val = divs[b2i];
				}

				if(compareDiv != scaledDivAdc){
					divs[b2i] = scaledDivAdc;
					compareDiv = scaledDivAdc;
					val = scaledDivAdc;
				}
				if(compareProb != scaledProbAdc){
					if(!probLengthToggle) {
						prob[b2i] = scaledProbAdc;
						compareProb = scaledProbAdc;
						val = scaledProbAdc;
						}
					else
						trigAdc[b2i] = analogRead(1);

				}
				writeDisplay(val);
			}
		}
	}
	else{
		dClickInt2 = dClickInt1;
		pButtons->buttonPressed = false;
		pButtons->dClickDetected = false;
		flagReg = 0;
		writeDisplay(bpm);
	}

	//Write to eeprom if button 3&4 are pressed.
	if(pButtons->currButtonState == EEPROM_WRITE){
		if (!(flagReg & EEPROM_WRITE_FLAG)) {
			eepromWrite();
			flagReg |= EEPROM_WRITE_FLAG;
		}
	}

	if(pButtons->currButtonState == EEPROM_READ){
		if (!(flagReg & EEPROM_READ_FLAG)) {
			eepromRead();
			flagReg |= EEPROM_READ_FLAG;
		}
	}

	if(pButtons->currButtonState == GATE_LENGTH){
		if (!(flagReg & GATE_PROB_TOGGLE)) {
			probLengthToggle ^= 1;
			flagReg |= GATE_PROB_TOGGLE;
		}
	}
	
	//Master inClock
	if((clkCurrTime - clkPrevTime) > clockAdc){
		inClock = HIGH;
		clkPrevTime = clkCurrTime;
	}

/*********************************************************************************************
*If a inClock is received, loop through outputs, check the divisions selected for the current out,
*check the probability stored for current out, if all ok -> turn on and set the outstate to HIGH
**********************************************************************************************/ 

	if(inClock == HIGH){
		inClock = LOW;

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
*The master inClock out has a fixed trig length.
**********************************************************************************************/

	for(output = 0; output<NUM_OUTPUTS; output++)
	{
		if ((outState[output] == HIGH) && (clkCurrTime - outPrevTime[output]) > trigTime[output]){
			OUT_PORT &= ~(1<<out[output]);
			outState[output] = LOW;
		}
	}

	if((clkOutState == HIGH) && (clkCurrTime - clkPrevTime) > clkTrigTime){
		if (!probLengthToggle) {
			OUT_PORT &= ~(1<<clockOut);
		}
	}

//Store last state of buttons/switches
	pButtons->prevButtonState = buttons.currButtonState;
	bpm = 60000 / clockAdc;
}

/*********************************************************************************************
*FUNCTIONS
**********************************************************************************************/

uint8_t checkButtons()
{
	pButtons->currButtonState = expander.byteRead(GPIOB);
	return pButtons->currButtonState != NO_BUTTON_PRESSED;
}

uint8_t checkProb(uint8_t num, uint8_t comp)
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

uint8_t buttonReg2index()
{
	switch(pButtons->currButtonState){
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
	uint8_t d1 = num % 10;
	uint8_t d2 = (num/10) % 10;
	uint8_t d3 = (num/100)%10;
	uint8_t d4 = num/1000;

	if(d4)
  		expander.byteWrite(GPIOA, d4 | (1<<4));   
  	if(d3 || (num > 999))
  		expander.byteWrite(GPIOA, d3 | (1<<5));
  	if(d2 || (num > 99))   
  		expander.byteWrite(GPIOA, d2 | (1<<6));  
  	
  	expander.byteWrite(GPIOA, d1 | (1<<7));
	expander.byteWrite(GPIOA, 0);   

}

uint16_t deJitter(uint16_t v, uint16_t test)
{
  if (abs(v - test) > 8) {
    return v;
  }
  return test;
}