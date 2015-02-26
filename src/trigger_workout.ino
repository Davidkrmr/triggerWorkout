#include <EEPROM.h>
#include <SPI.h>

#define NUM_OUTPUTS             4
#define OUT_REG                 DDRD
#define OUT_PORT                PORTD
#define ALL_OFF                 (OUT_PORT &= ~(B11111100))
#define ALL_ON                  (OUT_PORT |= (B11111100))
#define NO_BUTTON_PRESSED       0xFF
#define ADC_BITS                10
#define ADC_DIV_SHIFT           6
#define NUM_DIVS                (((1<<ADC_BITS))>>ADC_DIV_SHIFT)
#define RAND_SEED_ADC_PIN       5
#define EEPROM_WRITE            B11110011
#define EEPROM_READ             B11111100
#define DIV_EEPROM_ADDR_ST      0
#define PROB_EEPROM_ADDR_OFFSET 5
#define SW_DEBOUNCE             25
#define EEPROM_WRITE_FLAG       1
#define EEPROM_READ_FLAG        2
#define GATE_PROB_TOGGLE        4

//SPI defines
#define ADDR_ENABLE             0b00001000
#define OPCODEW                 0b01000000
#define OPCODER                 0b01000001
#define IOCON                   0x0A
#define ADDRESS                 0
#define IODIRA                  0x00
#define GPPUA                   0x0C
#define GPIOA                   0x12
#define GPIOB                   0x13

//Output pins
const uint8_t clk_out = 2;
const uint8_t o1 = 3;
const uint8_t o2 = 4;
const uint8_t o3 = 5;
const uint8_t o4 = 6;
const uint8_t out[NUM_OUTPUTS] = {o1, o2, o3, o4};

const uint8_t sw1 = 0x01;
const uint8_t sw2 = 0x02;
const uint8_t sw3 = 0x04;
const uint8_t sw4 = 0x08;

const uint16_t min_trig_time = 10;

const uint8_t div_table[NUM_DIVS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 32, 64, 128};

unsigned long clk_prev_time;
unsigned long clk_curr_time;

uint8_t clk_out_state;
uint8_t mstr_clk;

unsigned long db_timer;

//Rand num for probability check
uint8_t rand_num;

//Main cntr to keep track of divisions
unsigned long cntr;

//Display val
uint16_t val;

uint8_t flag_reg;

struct button_type
{
    uint8_t curr_btn_state;
    uint8_t prev_btn_state;
    uint8_t button_pressed;
}buttons;

uint8_t button;

struct output_type
{
    uint8_t divs;
    uint8_t div_compare;
    uint8_t prob;
    uint8_t prob_compare;
    unsigned long pos_edge;
    uint8_t state;
} output[NUM_OUTPUTS];

struct adc_type
{
    uint16_t clk;
    uint8_t scaled_div;
    uint16_t prob;
} adc;

void setup()
{
    OUT_REG |= B11111100;
    init_spi();
    init_outputs();
    randomSeed(analogRead(RAND_SEED_ADC_PIN));
}

void loop()
{

    uint8_t i;

    clk_curr_time = millis();

    //Master clock
    adc.clk = ((1<<10) - analogRead(2));

    //To be used for gate length
    // for(i = 0; i<NUM_OUTPUTS; i++){
    // 	output[i].trig_time = map(trig_adc[i], 0, 1023, 10, (adc.clk -10)) * output[i].divs;
    // }

    if(checkButtons()){

        if (buttons.curr_btn_state != buttons.prev_btn_state) {
            db_timer = millis();
        }

        if((millis() - db_timer) > SW_DEBOUNCE) {

            //Only read if we are actually going to change vals
            adc.scaled_div = div_table[(analogRead(0) >> ADC_DIV_SHIFT)];
            adc.prob = analogRead(1)>>2;

            button = get_button();

            if(!buttons.button_pressed) {
                buttons.button_pressed = true;
                output[button].div_compare = adc.scaled_div;
                output[button].prob_compare = adc.prob;
                val = output[button].divs;
            }

            if(output[button].div_compare != adc.scaled_div) {
                output[button].divs = adc.scaled_div;
                output[button].div_compare = adc.scaled_div;
                val = adc.scaled_div;
            }

            if(output[button].prob_compare != adc.prob) {
                output[button].prob = adc.prob;
                output[button].prob_compare = adc.prob;
                val = (100 * (adc.prob + 1)) >> 8;
                } 
            }
        }
    else{
        buttons.button_pressed = false;
        flag_reg = 0;
        val = calc_bmp();
    }


    //Poll eeprom write/read
    if(buttons.curr_btn_state == EEPROM_WRITE) {
        if(!(flag_reg & EEPROM_WRITE_FLAG)) {
            eepromWrite();
            flag_reg |= EEPROM_WRITE_FLAG;
        }
    }

    if(buttons.curr_btn_state == EEPROM_READ) {
        if(!(flag_reg & EEPROM_READ_FLAG)) {
            eepromRead();
            flag_reg |= EEPROM_READ_FLAG;
        }
    }

    //Master clock
    if((clk_curr_time - clk_prev_time) > adc.clk) {
        mstr_clk = HIGH;
        clk_prev_time = clk_curr_time;
    }

/*********************************************************************************************
*If a mstr_clk is received, loop through outputs, check the divisions selected for the current out,
*check the probability stored for current out, if all ok -> turn on and set the out_state to HIGH
**********************************************************************************************/ 

    if(mstr_clk == HIGH) {
        mstr_clk = LOW;

        rand_num = random(255);

        OUT_PORT |= (1<<clk_out);
        clk_out_state = HIGH;

        for(i = 0; i < NUM_OUTPUTS; i++) {
            if(((cntr % output[i].divs) == 0) && checkProb(rand_num, output[i].prob)) {
                OUT_PORT |= (1<<out[i]);
                output[i].state = HIGH;
                output[i].pos_edge = clk_prev_time;
            }
        }
        cntr++;
    }


    for(i = 0; i<NUM_OUTPUTS; i++) {
        if((output[i].state == HIGH) && (clk_curr_time - output[i].pos_edge) > min_trig_time) {
            OUT_PORT &= ~(1<<out[i]);
            output[i].state = LOW;
        }
    }

    if((clk_out_state == HIGH) && (clk_curr_time - clk_prev_time) > min_trig_time) {
            OUT_PORT &= ~(1<<clk_out);
    }

    buttons.prev_btn_state = buttons.curr_btn_state;
    writeDisplay(val);
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

    for(i = 0; i < NUM_OUTPUTS; i++) {
        output[i].divs = EEPROM.read(i);
        output[i].prob = EEPROM.read(i+PROB_EEPROM_ADDR_OFFSET);
    }
    runLights();
}

void eepromWrite()
{
    uint8_t i;

    for( i = 0; i < NUM_OUTPUTS; i++) {
        EEPROM.write(i, output[i].divs);
        EEPROM.write(i + PROB_EEPROM_ADDR_OFFSET, output[i].prob);
    }
    runLights();
}


/*********************************************************************************************
*OTHER
**********************************************************************************************/

void init_outputs()
{
    uint8_t i;

    for(i = 0; i < NUM_OUTPUTS; i++) {
        output[i] = {1, 1, 0, 0, 0, LOW};
    }
}

uint8_t checkButtons()
{
    buttons.curr_btn_state = spi_read_byte(GPIOB);
    return buttons.curr_btn_state != NO_BUTTON_PRESSED;
}

uint8_t checkProb(uint16_t num, uint16_t comp)
{
    return (num >= comp) ? true : false;
}


void runLights()
{
    uint8_t i;

    for(i = 0; i < NUM_OUTPUTS; i++){
        OUT_PORT |= (1<<out[i]);
        delay(100);
    }
    for(i = 0; i < NUM_OUTPUTS; i++){
        OUT_PORT &= ~(1<<out[i]);
        //Just test, will not use delay later on.
        delay(100);
    }
}

uint8_t get_button()
{
    buttons.curr_btn_state = ~buttons.curr_btn_state;
    switch(buttons.curr_btn_state){
        case sw1:
            return 0;
        break;
        case sw2:
            return 1;
        break;
        case sw3:
            return 2;
        break;
        case sw4:
            return 3;
        break;
    }
}


void writeDisplay(uint16_t val)
{
    static uint8_t intvl;
    uint16_t num[4] = {1000,100,10,1};
    uint8_t i;
    uint8_t cnt;

    if(intvl > 10){
        intvl = 0;
        for(i = 0; i < 4; i++)
        {
            cnt = 0;
            
            while(val >= num[i]){
                cnt += 1;
                val -= num[i];
            }
            spi_write_byte(GPIOA, cnt | (1<<(i + 4)));
        }
        spi_write_byte(GPIOA, 0);
    }
    intvl++;
}  

uint16_t calc_bmp()
{
    static uint16_t bpm;
    static uint8_t i;

    if(i >= 50)
    {
        bpm = (60000 / adc.clk);
        i = 0;
    }

    i++;
    return bpm;
}
