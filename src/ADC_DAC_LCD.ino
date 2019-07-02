#define OFF         "\033[0m"
#define BOLD        "\033[1m"
#define LOWINTENS   "\033[2m"
#define UNDERLINE   "\033[4m"
#define BLINK       "\033[5m"
#define REVERSE     "\033[7m"
#define RESCURS     "\033[0;0H"   // Esc[Line;ColumnH
#define CLS         "\033[2J"     // Esc[2J Clear entire screen

#define STRING_SIZE 9

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>

#include <Adafruit_ADS1015.h>
#include <Adafruit_MCP4725.h>
#include <IoAbstraction.h>
#include <IoAbstractionWire.h>
#include <Encoder.h>
#include <U8g2lib.h>

#define P(X) (Serial.println(X))

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_MCP4725 dacu;
Adafruit_MCP4725 daci;
U8G2_ST7920_128X64_1_HW_SPI u8g2(U8G2_R2, /* CS=*/ 10, /* reset=*/ 9);
IoAbstractionRef ioExpander = ioFrom8574(0x27);
Encoder enc(3, 2);

const float cal_u_low  = 2; //V
const float cal_u_high = 30; //V
const float cal_i_low  = 0.2; //A
const float cal_i_high = 2; //A

const float imax = 6;
const float umax = 36;
const float constrainVal[] = {imax, umax};
const uint16_t int16max = 0xFFFF;


uint16_t ringbuf[128][2];
uint8_t idx = 0;

byte menu_page = 0;
int edit_val = -1;
int edit_digit = 0;

float cal_v_ref;
float calibration_data[5][2];
/*
 * 1- DAC V
 * 2- DAC I
 * 3- ADC U
 * 4- ADC I
 * 5- ADC UEXT
 */
float set_data[2] {0.1, 5};
bool output_state = 0;
bool output_cc = 0;

enum dacport {
        dac_i = 0,
        dac_u = 1,
};

enum buttons {
        S2,
        S3,
        S4,
        S5,
        S1
};

enum bufItems {
        CURRENT,
        VOLTAGE,
        VOLTAGE_IN,
        VOLTAGE_REF,
        POWER,
        RESISTANCE,
        SET_CURRENT,
        SET_VOLTAGE
};

const uint8_t cOhm[] = {
        B00000011, B11000000,
        B00000100, B00100000,
        B00001000, B00010000,
        B00001000, B00010000,
        B00001000, B00010000,
        B00000100, B00100000,
        B00000010, B01000000,
        B00001110, B01110000
};



void setup(void){
        u8g2.begin();
        delay(500);
        setupI2C();
        calibrate();

}

void loop(){

        float f_adc[6];

        for(int i=0; i<4; i++) {
                f_adc[i] = adcCalRead(i);
        }
        output_cc = (f_adc[1] < set_data[1]- 0.1 ) ? 1 : 0; //check if voltage is out for cc

        saveBuffer(f_adc);
        buttonPoll();
        systemTask(f_adc);
        serialMenu();
}

long encPoll(){
        static long encOldPosition = 0;
        long encNewPosition, diff;
        encNewPosition = enc.read()/4;
        diff = encNewPosition-encOldPosition;
        encOldPosition = encNewPosition;
        return diff;
}

void systemTask(float f_adc[]){
        static unsigned long starttime = 0;
        const unsigned int timediff = 200;//ms
        if(edit_val!=-1) {
                P(constrainVal[edit_val]);
                //P(set_data[edit_val] + encPoll() * pow(10,edit_digit));
                float f = set_data[edit_val] + encPoll() * pow(10,edit_digit);
                set_data[edit_val] = constrain(f, 0, constrainVal[edit_val]);
        }
        if((millis() - starttime)>timediff) {
                char buf[8][STRING_SIZE]; //GLOBAL VAR Reduces Error probability
                f_adc[4] = f_adc[1] * f_adc[0]; //POWER
                f_adc[5] = f_adc[1] / f_adc[0]; //RESISTANCE
                for(int i=0; i<6; i++) {
                        generateString(buf[i], i, f_adc[i]);
                }
                for(int i=6; i<8; i++) {
                        generateString(buf[i], i, set_data[i-6]); //
                }
                lcdRefresh(buf);
                starttime = millis();
        }
}

void saveBuffer(float f_adc[]){
        idx++;
        if(idx==127)
                idx=0;

        ringbuf[idx][0] = round(constrain(f_adc[0],0,imax) * int16max / imax);
        ringbuf[idx][1] = round(constrain(f_adc[1],0,umax) * int16max / umax);
        //P(idx);
        //P(ringbuf[idx][0]);
}

void buttonPoll(){
        static int lastpress = -1;
        int activepress = -1;

        ioDeviceDigitalWrite(ioExpander, 5, output_state);
        if(output_state) {
                ioDeviceDigitalWrite(ioExpander, 6, !output_cc);
                ioDeviceDigitalWrite(ioExpander, 7, output_cc);
        }
        else{
                ioDeviceDigitalWrite(ioExpander, 6, 1);
                ioDeviceDigitalWrite(ioExpander, 7, 1);
        }
        ioDeviceSync(ioExpander);
        for(int i=0; i<5; i++) {
                if(!ioDeviceDigitalRead(ioExpander, i))
                        activepress = i;
        }
        if(activepress == S2 && lastpress != S2) {
                P("S2");
                output_state = !output_state;
                lastpress = S2;
                if(output_state) {
                        setOutput(dac_u,set_data[dac_u],true);
                        setOutput(dac_i,set_data[dac_i],true);
                }
                else{
                        setOutput(dac_u,-1,true);
                        setOutput(dac_i,-1,true);
                }
        }
        else if(activepress == S3 && lastpress != S3) {

                lastpress = S3;
                if(menu_page == 0)
                        menu_page = 1;
                else
                        menu_page = 0;
                P("S3");
        }
        else if(activepress == S4 && lastpress != S4) {
                lastpress = S4;
                edit_val = (edit_val!=CURRENT) ? CURRENT :  -1;
                edit_digit = 0;
                P("S4");
        }
        else if(activepress == S5 && lastpress != S5) {
                lastpress = S5;
                edit_val = (edit_val!=VOLTAGE) ? VOLTAGE :  -1;
                edit_digit = 0;
                P("S5");
        }
        else if(activepress == S1 && lastpress != S1) {
                lastpress = S1;
                switch(edit_val) {
                case VOLTAGE:
                        edit_digit--;
                        if(edit_digit==-4)
                                edit_digit=1;
                        break;
                case CURRENT:
                        edit_digit--;
                        if(edit_digit==-5)
                                edit_digit=0;
                        break;

                }
                P("S1");

        }
        else if(activepress == -1) {
                lastpress = -1;
        }
}

void lcdRefresh(char buf[][STRING_SIZE]){
        uint16_t maxval[2] = {1, 1};
        char tmp[2][5];
        //Serial.println(buf[1]);
        //Serial.println(buf[0]);
        u8g2.firstPage();
        switch(menu_page) {
        case 1:
                for(int i=0; i<127; i++) {
                        if(ringbuf[i][0] > maxval[0])
                                maxval[0] = constrain(ringbuf[i][0],1,int16max);
                        if(ringbuf[i][1] > maxval[1])
                                maxval[1] = constrain(ringbuf[i][1],1,int16max);
                }
                floatToVariableString(tmp[0], (float)maxval[0]*imax/int16max, 4, 0, imax);
                floatToVariableString(tmp[1], (float)maxval[1]*umax/int16max, 4, 0, umax);
                do {
                        //u8g2.drawFrame(0, 0, 128, 64);
                        u8g2.setFont(u8g2_font_t0_12_tr);
                        u8g2.drawHLine(0,31,128);
                        u8g2.drawHLine(0,32,128);
                        u8g2.drawStr(3,13, "U");
                        u8g2.drawStr(100,13, tmp[1]);
                        u8g2.drawStr(3,32+13, "I");
                        u8g2.drawStr(100,32+13, tmp[0]);
                        for(int i=0; i<126; i++) {
                                if(i==idx) continue;
                                long l[4];
                                l[0] = 30-constrain(30*(long)ringbuf[i][1]/maxval[1],0,int16max);
                                l[1] = 30-constrain(30*(long)ringbuf[i+1][1]/maxval[1],0,int16max);
                                l[2] = 63-constrain(30*(long)ringbuf[i][0]/maxval[0],0,int16max);
                                l[3] = 63-constrain(30*(long)ringbuf[i+1][0]/maxval[0],0,int16max);
                                u8g2.drawLine(i, l[0], i+1, l[1]);
                                u8g2.drawLine(i, l[2], i+1, l[3]);
                                //u8g2.drawPixel(i, 30-((float)ringbuf[i][1]/(float)maxval[1]*30));
                                //u8g2.drawPixel(i, 63-((float)ringbuf[i][0]/(float)maxval[0]*30));
                        }
                } while ( u8g2.nextPage() );
                break;
        default:
                char cursor[] = {113,'\n'};
                static bool blinkState = 0;
                blinkState = !blinkState;
                do {
                        u8g2.setFont(u8g2_font_7x14_tr);
                        u8g2.drawStr(3,13, buf[VOLTAGE]);
                        u8g2.drawStr(3,13+16, buf[CURRENT]);
                        u8g2.drawStr(3,13+32, buf[POWER]);
                        u8g2.drawStr(3,13+48, buf[VOLTAGE_IN]);
                        u8g2.drawStr(3+(128-16)/2,13+48, buf[VOLTAGE_REF]);
                        //u8g2.drawStr(110,60, cursor);
                        u8g2.drawStr(3+(128-16)/2,13, buf[SET_VOLTAGE]);
                        u8g2.drawStr(3+(128-16)/2,13+16, buf[SET_CURRENT]);
                        u8g2.drawBitmap( 97, 37, 2, 8, cOhm);
                        u8g2.drawStr(3+(128-16)/2,13+32, buf[RESISTANCE]);
                        u8g2.drawStr(117,13, "U");
                        u8g2.drawStr(117,13+16, "I");
                        u8g2.drawFrame(0, 0, 128, 64);
                        u8g2.drawFrame(0, 16, 128, 16);
                        u8g2.drawFrame(0, 32, 128, 16);
                        u8g2.drawVLine(128-16, 0, 64);
                        u8g2.drawVLine((128-16)/2, 0, 64);
                        switch(edit_val) {
                        case VOLTAGE:
                                if(blinkState)
                                        u8g2.drawStr(66,13, "_");
                                u8g2.drawStr(117,13+32, "<");
                                u8g2.drawStr(117,13+48, ">");
                                break;
                        case CURRENT:
                                if(blinkState)
                                        u8g2.drawStr(59,13+16, "_");
                                u8g2.drawStr(117,13+32, "<");
                                u8g2.drawStr(117,13+48, ">");
                                break;
                        default:
                                u8g2.drawStr(117,13+32, "G");
                                u8g2.drawStr(117,13+48, "O");
                                break;
                        }
                } while ( u8g2.nextPage() );
                break;
        }
}

void serialMenu(){
        char dummy;
        static int a = 0;
        static bool listen = 0;
        if ((Serial.available() > 0) && a != 0) {
                float f;
                switch(a) {
                case 1:
                        a = Serial.parseInt();
                        Serial.println(a);
                        if(a) {
                                setOutput(dac_u,set_data[dac_u],true);
                                setOutput(dac_i,set_data[dac_i],true);
                        }
                        else{
                                setOutput(dac_u,-1,true);
                                setOutput(dac_i,-1,true);
                        }
                        break;
                case 2:
                        f = Serial.parseFloat();
                        Serial.println(f);
                        setOutput(dac_u,f,false);
                        break;
                case 3:
                        f = Serial.parseFloat();
                        Serial.println(f);
                        setOutput(dac_i,f,false);
                        break;
                }
                a=0;
                while(Serial.available()) dummy=Serial.read();
        }
        //Serial.println(Serial.available());
        if ((Serial.available() > 0) && a==0) {
                a = Serial.parseInt();
                while(Serial.available()) dummy=Serial.read();
                switch(a) {
                case 1:
                        Serial.print("State["+ (String)output_state+ "]: ");
                        break;
                case 2:
                        Serial.print("Vset["+ (String)set_data[dac_u]+ "]: ");
                        break;
                case 3:
                        Serial.print("Iset["+ (String)set_data[dac_i]+ "]: ");
                        break;
                }
        }
}

void setupI2C(){
        Serial.begin(115200);
        Wire.begin();

        ioDevicePinMode(ioExpander, 0, INPUT);
        ioDevicePinMode(ioExpander, 1, INPUT);
        ioDevicePinMode(ioExpander, 2, INPUT);
        ioDevicePinMode(ioExpander, 3, INPUT);
        ioDevicePinMode(ioExpander, 4, INPUT);
        ioDevicePinMode(ioExpander, 5, OUTPUT);
        ioDevicePinMode(ioExpander, 6, OUTPUT);
        ioDevicePinMode(ioExpander, 7, OUTPUT);

        ioDeviceDigitalWrite(ioExpander, 5, 0);
        ioDeviceDigitalWrite(ioExpander, 6, 1);
        ioDeviceDigitalWrite(ioExpander, 7, 1);

        ioDeviceSync(ioExpander);

        //                                                                ADS1015  ADS1115
        //                                                                -------  -------
        //ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
        ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
        // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
        // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
        // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
        // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
        //Serial.print(CLS);
        ads.begin();
        dacu.begin(0x62);
        daci.begin(0x63);

        setOutput(dac_u,-1,true);
        setOutput(dac_i,-1,true);

        delay(100);
}

float adcCalRead(byte channel){
        unsigned int i_adc;
        float f_adc, f_adc2;
        switch(channel) {
        default:
                i_adc = ads.readADC_SingleEnded(channel);
                f_adc = calibration_data[channel+2][0] * (float)i_adc * 0.000125 * cal_v_ref + calibration_data[channel+2][1];
                break;
        case 3:
                i_adc = ads.readADC_SingleEnded(channel);
                f_adc = ((float)i_adc) * 0.000125 * cal_v_ref;
                break;
        }
        if(f_adc<0) f_adc=0;
        return f_adc;
}

void generateString(char* buf, byte channel, float data){
        unsigned char suffix[] = {'A','V', 'V', 'V', 'W', '\0', 'A', 'V'};
        char tmp[STRING_SIZE];
        //Serial.println(buf);
        buf[0]='\0';

        switch(channel) {
        case SET_CURRENT:
                dtostrf(data,6,4,tmp);
                break;
        case SET_VOLTAGE:
                dtostrf(data,6,3,tmp);
                if(data<10)
                        tmp[0]='0';
                break;
        case POWER:
                floatToVariableString(tmp, data, 6, 0, 999);
                break;
        case RESISTANCE:
                floatToVariableString(tmp, data, 6, 0, 999999);
                break;
        default:
                floatToVariableString(tmp, data, 6, 0, 99);
                break;
        }
        strcat(buf, tmp);
        int strl = strlen(buf);
        buf[strl]=suffix[channel];
        buf[strl+1]='\0';
        //Serial.println(buf);
}

void floatToVariableString(char * str, float f, int size, int min, long max){
        f = constrain(f, min, max);
        int dec = constrain((int)log10(f),0,size);
        if(dec > (size-1))
                dtostrf(f,size,size-dec,str);
        else
                dtostrf(f,size,constrain(size-dec-2,0,size),str);
        /*
           if(f<10)
           dtostrf(f,size,4,str);
           else if(f<100)
           dtostrf(f,size,3,str);
           else if(f<1000)
           dtostrf(f,size,2,str);
           else if(f<10000)
           dtostrf(f,size,1,str);
           else
           dtostrf(f,size,0,str);
           //*/
}

void setOutput(byte prop, float value, bool override){
        unsigned int temp;
        switch(prop) {
        case dac_u:
                if(value<0) {
                        output_state = 0;
                        temp = 0;
                }
                else{
                        output_state = 1;
                        temp = calibration_data[0][0]*value + calibration_data[0][1];
                        set_data[1]=value;
                }
                if(output_state == 0 && !override) return;
                dacu.setVoltage(temp, false);
                break;

        case dac_i:
                if(value<0)
                        temp = 0;
                else{
                        temp = calibration_data[1][0]*value + calibration_data[1][1];
                        set_data[0]=value;
                }
                if(output_state == 0 && !override) return;
                daci.setVoltage(temp, false);
                break;

        }
}

void calibrate(){

        u8g2.firstPage();
        do {
                u8g2.setFont(u8g2_font_t0_12_tr);
                u8g2.drawStr(10,64/2+8/2, "Calibration...");
        } while ( u8g2.nextPage() );

        cal_v_ref = 4.096/ ((float)ads.readADC_SingleEnded(3) * 0.000125);
        Serial.println(cal_v_ref, 5);


        ioDeviceSync(ioExpander);
        unsigned int cal_buffer[2];
        //EESAVE FUSE = 0!
        if(EEPROM.read(0)== 255 || !ioDeviceDigitalRead(ioExpander, 4)) {
                dacu.setVoltage(0, true);
                daci.setVoltage(0, true);
                delay(1000);
                u8g2.firstPage();
                do {
                        u8g2.setFont(u8g2_font_t0_12_tf);
                        u8g2.drawStr(0,64/2+14/4, "Setup on Serial Port");
                } while ( u8g2.nextPage() );
                float cache[2];
                float cal_adc[2];
                Serial.println(F("CAL TABLE EMPTY, CALIBRATING!"));
                set_data[0]=cal_u_low;
                set_data[1]=cal_i_low;
                dacu.setVoltage(((cal_u_low+0.62)/13.69)/4.096*pow(2,12), false);
                daci.setVoltage(((cal_i_low+0.29)/2.48)/4.096*pow(2,12), false);
                Serial.print(F("Enter measured Voltage[V]: "));
                while (!Serial.available());
                cache[0] = Serial.parseFloat();
                Serial.println(cache[0]);
                cal_buffer[0] = round(cal_u_low/cache[0]*((cal_u_low+0.62)/13.69)/4.096*pow(2,12));
                cal_adc[0] = ads.readADC_SingleEnded(1) * 0.000125 * cal_v_ref;

                dacu.setVoltage(((cal_u_high+0.62)/13.69)/4.096*pow(2,12), false);
                Serial.print(F("Enter measured Voltage[V]: "));
                while (!Serial.available());
                cache[1] = Serial.parseFloat();
                Serial.println(cache[1]);
                cal_buffer[1] = round(cal_u_high/cache[1]*((cal_u_high+0.62)/13.69)/4.096*pow(2,12));
                cal_adc[1] = ads.readADC_SingleEnded(1) * 0.000125 * cal_v_ref;

                calibration_data[0][0] = ((float)(cal_buffer[1] - cal_buffer[0])) / (float)(cal_u_high - cal_u_low); //k
                calibration_data[0][1] = (float)cal_buffer[1] - calibration_data[0][0] * (float)cal_u_high; //d

                calibration_data[3][0] = (cache[1] - cache[0]) / (float)(cal_adc[1] - cal_adc[0]); //k
                calibration_data[3][1] = cache[1] - calibration_data[3][0] * cal_adc[1]; //d

                dacu.setVoltage(calibration_data[0][0]*cal_u_low + calibration_data[0][1], false);//for low shortcut spike currents
                daci.setVoltage(((cal_i_low+0.29)/2.48)/4.096*pow(2,12), false);
                Serial.print(F("Enter measured Current[A]: "));
                while (!Serial.available());
                cache[0] = Serial.parseFloat();
                Serial.println(cache[0]);
                cal_buffer[0] = round(((float)cal_i_low)/cache[0]*((cal_i_low+0.29)/2.48)/4.096*pow(2,12));
                cal_adc[0] = ads.readADC_SingleEnded(0) * 0.000125 * cal_v_ref;

                daci.setVoltage(((cal_i_high+0.29)/2.48)/4.096*pow(2,12), false);
                Serial.print(F("Enter measured Current[A]: "));
                while (!Serial.available());
                cache[1] = Serial.parseFloat();
                Serial.println(cache[1]);
                cal_buffer[1] = round(((float)cal_i_high)/cache[1]*((cal_i_high+0.29)/2.48)/4.096*pow(2,12));
                cal_adc[1] = ads.readADC_SingleEnded(0) * 0.000125 * cal_v_ref;

                calibration_data[1][0] = (float)(cal_buffer[1] - cal_buffer[0]) / (float)(cal_i_high - cal_i_low); //k
                calibration_data[1][1] = (float)cal_buffer[1] - calibration_data[1][0] * (float)cal_i_high; //d

                calibration_data[2][0] = (cache[1] - cache[0]) / ((float)(cal_adc[1] - cal_adc[0])); //k
                calibration_data[2][1] = cache[1] - calibration_data[2][0] * cal_adc[1]; //d

                daci.setVoltage(calibration_data[1][0]*cal_i_low + calibration_data[1][1], false);//for low shortcut spike currents

                Serial.print(F("Enter measured Input Voltage[V]: "));
                while (!Serial.available());
                cache[0] = Serial.parseFloat();
                Serial.println(cache[0]);
                cal_adc[0] = ads.readADC_SingleEnded(2) * 0.000125 * cal_v_ref;

                Serial.print(F("Enter measured Input Voltage[V]: "));
                while (!Serial.available());
                cache[1] = Serial.parseFloat();
                Serial.println(cache[1]);
                cal_adc[1] = ads.readADC_SingleEnded(2) * 0.000125 * cal_v_ref;

                calibration_data[4][0] = (cache[1] - cache[0]) / ((float)(cal_adc[1] - cal_adc[0])); //k
                calibration_data[4][1] = cache[1] - calibration_data[4][0] * cal_adc[1]; //d


                for(int i=0; i<5; i++) {
                        Serial.print("CD "+ (String)(i+1) +": "); Serial.print(calibration_data[i][0], 6); Serial.print(",  ");  Serial.println(calibration_data[i][1], 6);
                }

                EEPROM.write(0, 1);
                for(unsigned int i=0, j=1; i<10; i++, j+=sizeof(float)) {
                        EEPROM.put(j, calibration_data[i/2][(i)%2]);
                }
        }
        else{
                for(unsigned int i=0, j=1; i<10; i++, j+=sizeof(float)) {
                        EEPROM.get(j,  calibration_data[i/2][(i+2)%2]);
                }

                for(int i=0; i<5; i++) {
                        Serial.print("CD "+ (String)(i+1) +": "); Serial.print(calibration_data[i][0], 6); Serial.print(",  ");  Serial.println(calibration_data[i][1], 6);
                }
        }
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void timeMeas(bool status){
        static unsigned int sample=0;
        static unsigned int samplestarttime;
        unsigned int sampletime;
        if(status) {
                samplestarttime = micros();
        }
        else{
                sampletime = micros() - samplestarttime;
                Serial.print("Sample "); Serial.print(sampletime); Serial.print(":"); Serial.println(sampletime);
        }
}
