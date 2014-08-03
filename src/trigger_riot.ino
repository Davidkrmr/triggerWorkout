//Todo
//Debounce switches
//Moving average on raw ADC val instead of monitoring change of scaled val
#include <EEPROM.h>

#define NUM_OUTPUTS 4
#define SW_REG DDRB
#define SW_PORT PORTB
#define SW_READ_PORT PINB
#define OUT_REG DDRD
#define OUT_PORT PORTD
#define ALL_OFF (OUT_PORT &= ~(B11111100))
#define ALL_ON (OUT_PORT |= (B11111100))
#define NO_BUTTON_PRESSED B00001111
#define ADC_BITS 10
#define ADC_DIV_SHIFT 6
#define ADC_PROB_SHIFT 3
#define NUM_DIVS (((1<<ADC_BITS))>>ADC_DIV_SHIFT)
#define RAND_SEED_ADC_PIN 5
#define SW1 B00001110
#define SW2 B00001101
#define SW3 B00001011
#define SW4 B00000111
#define D_CLICK_TIME 250
#define DIV_EEPROM_ADDR_ST 0;
#define PROB_EEPROM_ADDR_OFFSET 5
#define SW_DEBOUNCE 25

#define max(a,b) (((a)>(b))?(a):(b))
#define min(a,b) (((a)<(b))?(a):(b))

#define IN_RANGE(min,max,val) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

uint8_t divtable[NUM_DIVS] = {1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 32, 64, 128};

unsigned long prevTime;
unsigned long currTime;

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

uint8_t outState;
uint8_t output;

uint8_t trigTime = 20;
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
uint8_t buttonPressed;
	
uint8_t check_buttons;
uint8_t lastCheckButtons;
uint8_t button;
uint8_t b2i;
uint8_t dClickDetected;
unsigned long dbTimer;

int checkProb(uint8_t num, uint8_t comp)
{
	return (num >= comp) ? 1 : 0;
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

void setup()
{
	randomSeed(analogRead(RAND_SEED_ADC_PIN));

	//clockOut
	OUT_REG |= B11111100;
	SW_REG |= B11110000;
	SW_PORT |= B00001111;
}

void loop()
{

	currTime = millis();
	clockAdc = ((1<<10) - analogRead(2));

	scaledDivAdc = divtable[(analogRead(0)>>ADC_DIV_SHIFT)];
	scaledProbAdc = (analogRead(1)>>ADC_PROB_SHIFT);

	if(compareDiv != scaledDivAdc){
		divWait = 0;
	}

	if(compareProb != scaledProbAdc){
		probWait = 0;
	}

	//trigTime = IN_RANGE(5, (clockAdc - 10), analogRead(3));

/*******************************************************************
*Monitor button read register. If any b is pressed, indicate with 
*buttonPress and set wait flags. These will keep the current settings
*for the selected outport until a pot is changed.
*Convert value of register to index.
*******************************************************************/
	check_buttons = (SW_READ_PORT & B00001111);

	if(check_buttons != NO_BUTTON_PRESSED){

		if (check_buttons != lastCheckButtons) {
			dbTimer = millis();
		}

		if((millis() - dbTimer) > SW_DEBOUNCE) {

			b2i = buttonReg2index(check_buttons);

			dClickInt1 = millis();

			if(((dClickInt1 - dClickInt2) < D_CLICK_TIME) && !dClickDetected){
				dClickDetected = 1;
				divs[b2i] = 1;
				prob[b2i] = 0;
			}

			else{
				if(!buttonPressed){
					buttonPressed = 1;
					compareDiv = scaledDivAdc;
					compareProb = scaledProbAdc;
					divWait = 1;
					probWait = 1;
				}

				if(!divWait){
					divs[b2i] = scaledDivAdc;
				}
				if(!probWait){
					prob[b2i] = scaledProbAdc;
				}
			}
		}
	}
	else{
		dClickInt2 = dClickInt1;
		buttonPressed = 0;
		dClickDetected = 0;
	}


	//Write to eeprom if button 3&4 are pressed.
	if((!(SW_READ_PORT & (1<<s4))) && (!(SW_READ_PORT & (1<<s3)))){
		eepromWrite();
	}

	if((!(SW_READ_PORT & (1<<s1))) && (!(SW_READ_PORT & (1<<s2)))){
		eepromRead();
	}

	//Master clock
	if((currTime - prevTime) > clockAdc){
		clock = HIGH;
		prevTime = currTime;
	}

	//Turn on 
	if(clock == HIGH){
		clock = LOW;
		outState = HIGH;

		randNum = random(127);

		OUT_PORT |= (1<<clockOut);


		for(output=0; output<NUM_OUTPUTS; output++)
		{
			if(((counter % divs[output]) == 0) && checkProb(randNum, prob[output])){
				OUT_PORT |= (1<<out[output]);
			}
		}
		counter++;
	}	
	//Turn off
	if((outState == HIGH) && (currTime - prevTime) > trigTime){		
			OUT_PORT = ALL_OFF;
			outState = LOW;
	}

	lastCheckButtons = check_buttons;
}

