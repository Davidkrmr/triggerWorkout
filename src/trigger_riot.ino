//Todo
//Dubbelklick = reset för varje out
//Spara sista vid nedstängning (håll inne knapp?)
#include <EEPROM.h>

#define HIGH 1
#define LOW 0
#define NUM_OUTPUTS 4
#define SW_READ_PORT PINB
#define OUT_PORT PORTD
#define ALL_OFF (OUT_PORT &= ~(B11111100))
#define ALL_ON (OUT_PORT |= (B11111100))
#define NO_BUTTON_PRESSED 0b00001111
#define ADC_BITS 10
#define ADC_DIV_SHIFT 6
#define ADC_PROB_SHIFT 3
#define NUM_DIVS (((1<<ADC_BITS))>>ADC_DIV_SHIFT)
#define RAND_SEED_ADC_PIN 5
#define SW1 B00001110
#define SW2 B00001101
#define SW3 B00001011
#define SW4 B00000111

#define max(a,b) (((a)>(b))?(a):(b))
#define min(a,b) (((a)<(b))?(a):(b))

#define IN_RANGE(min,max,val) (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))


uint16_t divtable[NUM_DIVS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 16, 32, 64, 128, 128};

long prevTime;
long currTime;

//Random variables
uint8_t randNum;
uint8_t randComp[NUM_OUTPUTS] = {0, 0, 0, 0};

//Output pins
uint8_t clockOut = 2;
uint8_t o1 = 3;
uint8_t o2 = 4;
uint8_t o3 = 5;
uint8_t o4 = 6;
uint8_t out[NUM_OUTPUTS] = {o1, o2, o3, o4};

//Select switches pins
uint8_t s1 = 0;
uint8_t s2 = 1;
uint8_t s3 = 2;
uint8_t s4 = 3;

//Array holding division for each output
uint8_t divs[NUM_OUTPUTS] = {1, 1, 1, 1};

uint8_t outState;

uint8_t trigTime = 20;
uint8_t clock;
long counter;

uint16_t clockAdc;
uint8_t scaledDivAdc;
uint8_t scaledProbAdc;

uint8_t tempDiv;
uint8_t tempComp;
uint8_t divWait;
uint8_t compWait;
uint8_t buttonPressed;

uint8_t i;
uint8_t eepromReadFlag;
uint8_t button;

int checkProb(uint8_t num, uint8_t comp)
{
	return (num >= comp) ? 1:0;
}

void eepromRead()
{
	for(i=0; i<NUM_OUTPUTS; i++){
		divs[i] = EEPROM.read(i);
	}
	for(i=0; i<NUM_OUTPUTS; i++){
		randComp[i] = EEPROM.read(i+5);
	}
}

void flash()
{
	OUT_PORT = ALL_ON;
	delay(500);
	OUT_PORT = ALL_OFF;
	delay(500);
	OUT_PORT = ALL_ON;
	delay(500);
	OUT_PORT = ALL_OFF;
}

int buttonReg2index(uint8_t button)
{
	uint8_t index;

	if (!divWait){
		switch(button){
			case SW1:
				index = 0;
			break;
			case SW2:
				index = 1;
			break;
			case SW3:
				index = 2;
			break;
			case SW4:
				index = 3;
			break;
		}
	}
	return index;
}

void setup()
{
	randomSeed(analogRead(RAND_SEED_ADC_PIN));

	//clockOut
	pinMode(2, OUTPUT);

	//Div outs
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	//counter = 1;

	//select buttons
	pinMode(8, INPUT);
	pinMode(9, INPUT);
	pinMode(10, INPUT);
	pinMode(11, INPUT);
	digitalWrite(8, HIGH);
	digitalWrite(9, HIGH);
	digitalWrite(10, HIGH);
	digitalWrite(11, HIGH);

	Serial.begin(9600);
	eepromReadFlag = 0;
}

void loop()
{

	if(!eepromReadFlag){
		eepromRead();
		eepromReadFlag = 1;
	}

	currTime = millis();
	clockAdc = ((1<<10) - analogRead(2));

	scaledDivAdc = divtable[(analogRead(0)>>ADC_DIV_SHIFT)];
	scaledProbAdc = (analogRead(1)>>ADC_PROB_SHIFT);

	//Monitor select buttons. If pressed, store adc val
	if(!buttonPressed){
		tempDiv = scaledDivAdc;
		tempComp = scaledProbAdc;
	}

	//If stored adc val != current -> Pot has changed, ok to write val to array.
	if(tempDiv != scaledDivAdc){
		divWait = 0;
	}

	if(tempComp != scaledProbAdc){
		compWait = 0;
	}

	//trigTime = IN_RANGE(5, (clockAdc - 10), analogRead(3));

	//Monitor all buttons, if any is pressed, make adc write procedure wait for pot movement.

	if((button = (SW_READ_PORT & 0b00001111)) != NO_BUTTON_PRESSED){
		if(!buttonPressed){
			buttonPressed = 1;
			divWait = 1;
			compWait = 1;
		}

		if (!divWait){
			divs[buttonReg2index(button)] = scaledDivAdc;
		}

		if (!compWait){
			randComp[buttonReg2index(button)] = scaledProbAdc;
		}
	}
	
	else
		buttonPressed = 0;

	/*//Check buttons and change if pot is moved
	if(!(SW_READ_PORT & (1<<s1))){
		if(!divWait)
			divs[0] = scaledDivAdc;
		
		if(!compWait)
			randComp[0] = scaledProbAdc;
	}

	else if(!(SW_READ_PORT & (1<<s2))){
		if(!divWait)
			divs[1] = scaledDivAdc;
		
		if(!compWait)
			randComp[1] = scaledProbAdc;
	}

	else if(!(SW_READ_PORT & (1<<s3))){
		if(!divWait)
			divs[2] = scaledDivAdc;
		
		if(!compWait)
			randComp[2] = scaledProbAdc;
	}

	else if(!(SW_READ_PORT & (1<<s4))){
		if(!divWait)
			divs[3] = scaledDivAdc;
		
		if(!compWait)
			randComp[3] = scaledProbAdc;;
	}*/


	//Write to eeprom if button 3&4 are pressed.
	if((!(SW_READ_PORT & (1<<s4))) && (!(SW_READ_PORT & (1<<s3)))){
		flash();
		for(i=0; i<NUM_OUTPUTS; i++){
			EEPROM.write(i, divs[i]);
		}
		for(i=0; i<NUM_OUTPUTS; i++){
			EEPROM.write(i + 5, randComp[i]);
		}
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

		for(i=0; i<NUM_OUTPUTS; i++)
		{
			if(((counter % divs[i]) == 0) && checkProb(randNum, randComp[i])){
				OUT_PORT |= (1<<out[i]);
			}
		}
		counter++;
	}	
	//Turn off
	if((outState == HIGH) && (currTime - prevTime) > trigTime){		
			OUT_PORT = ALL_OFF;
			outState = LOW;
	}
}

