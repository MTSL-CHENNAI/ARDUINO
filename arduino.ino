
//#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#include <avr/interrupt.h>

#include <avr/sleep.h>

#define LEAK_IN 4
#define EN_LORA 3
#define LED1    2
#define LED2    5

#define alarm_frame "AT+DTRX=0,1,3,414C31\r"
#define ok_frame "AT+DTRX=0,1,6,4F4B30"
#define restart_frame "\0\0\r\n"
#define start_frame "AT+CLPM=0\r"
#define end_frame   "AT+CLPM=1\r"

#define INTERVAL_MN 16

uint8_t alarm_active = 1;
uint8_t statusTimer = 19;
uint8_t counter = 60;
uint8_t alarmin = 0;
uint8_t msg_count  = 0;
uint16_t Vcc_value = 1000;

void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 16384, RTC_PERIOD_CYC16384_gc resulting in 32.768kHz/16384 = 2Hz */
  | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
  
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
}

void Port_init(){
  //pinMode(LEAK_IN,INPUT_PULLUP); //PA3
  pinMode(LEAK_IN,INPUT); //PA3
  pinMode(EN_LORA,INPUT_PULLUP);//PA2
  pinMode(LED1,INPUT_PULLUP);//PA1
  pinMode(LED2,INPUT_PULLUP);//PA0
  // OK for low power
  pinMode(0,INPUT_PULLUP);//Set TXD to pull-up input
  pinMode(1,INPUT);//Set RXD to pull-up input
  delay(1000);
  Serial.begin(9600);
  delay(500);
  
  Serial.print(restart_frame);
  Serial.flush();
  delay(100);
  Serial.print(start_frame);
  Serial.flush();
  delay(100);
/*
  Serial.print("AT+IREBOOT=0\r\n");
  Serial.flush();
  delay(500);
  */
   Serial.print("AT+CSTATUS=?\r\n");
   Serial.flush();
   delay(50);
  // Serial.print("AT+CADR=0\r\n");
  // delay(2000);
  // Serial.print("AT+CDATARATE=5\r\n"); //SF12
  // delay(2000);
  // Serial.print("AT+CSAVE\r\n");
  // delay(2000);
  // Serial.print("AT+IREBOOT=0\r\n");
  // delay(1000);
  /*
  Serial.print("AT+CNWKSKEY=2E8C8650B4041672BBB9A399F2DEB427\r\n");
  delay(50);
  Serial.print("AT+CAPPSKEY=EF6D6E2503F57AE2FA151CDA87455F18\r\n");
  delay(50);
  Serial.print("AT+CSAVE");
  */
  delay(500);
  Serial.end();
  //pinMode(0,INPUT);//Set TXD to pull-up input
  //pinMode(1,INPUT);//Set RXD to pull-up input
  //PORTA.DIRCLR = PIN7_bm; 
  //PORTA.DIRCLR = PIN6_bm;    
}

/**************************************************************************************
* ADC_on - Turn the AtoD Converter ON
**************************************************************************************/
void ADC_on()
{
    VREF.CTRLA |= VREF_ADC0REFSEL_1V1_gc; // Set Vbg ref to 1.1V
    ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc; // ADC internal reference, the Vbg
    ADC0.CTRLC = ADC_PRESC_DIV64_gc // CLK_PER divided by 64 = 52 KHz
      | ADC_REFSEL_VDDREF_gc // Vdd (Vcc) be ADC reference
      | 0 << ADC_SAMPCAP_bp; // Sample Capacitance Selection: disabled
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc; // Enabled, 10-bit mode
    ADC0.COMMAND |= ADC_STCONV_bm; // Start the ADC conversions
    
    ADC0.CTRLD |= 0b00100000;  // INITDLY: delay 16 ADC clock cycles
    ADC0.SAMPCTRL |= 0b00000100; // SAMPLEN: 4 ADC clock cycles
    ADC0.CTRLC |= ADC_SAMPCAP_bm; // In ADC.CTRLC Select SAMPCAP = 5pf
}

/**************************************************************************************
* ADC_off - Turn the AtoD Converter OFF
**************************************************************************************/
inline void ADC_off()
{
    ADC0.CTRLA &= ~(ADC_ENABLE_bm);  // disable the ADC 
}

void setup()
{
  delay(100);                          // wait for clocks to stabilise (important) before...
  Port_init();
  RTC_init();
  delay(1000);                          // wait for clocks to stabilise (important) before...
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // set power saving mode as STANDBY, not POWER DOWN
  sleep_enable();                     // enable sleep mode
  delay(200);                          // wait for clocks to stabilise (important) before...
  alarmin = (PORTA.IN & PIN3_bm);
}

volatile uint8_t ledcount = 20; 

void loop() {
  if((alarm_active==0)||(statusTimer==0)||(msg_count != 0))
  { 
    if((counter&0x01)&&(counter<ledcount)){
      PORTA.OUTCLR = PIN1_bm;       
    }
    else{
      PORTA.OUTSET = PIN1_bm;       
    }
    //digitalWrite(LED1,CHANGE);
    if(counter==1){
        Serial.begin(9600);
        ADC_on();
    }
    else if(counter==2){
        Serial.print(restart_frame);
        Serial.print("\r");
        Serial.flush();
        //delay(50);
        Serial.print(start_frame);
        Serial.flush();
    }
    else if((counter==4)){
        if(alarm_active==0)
          Serial.print(alarm_frame);
        else
        {
          if (ADC0.INTFLAGS)                // if an ADC result is ready
          {
              Vcc_value = ADC0.RES;
          }
          else{
              Vcc_value = 1000;
          }
          if(Vcc_value >= 1000)
             Vcc_value = 999;
  
          uint8_t v1,v2,v3;
          v1 = (uint8_t)(Vcc_value/100);
          v2 = (uint8_t)((Vcc_value-(v1*100))/10);
          v3 = (uint8_t)(Vcc_value-(v1*100)-(v2*10));
        
          Serial.print(ok_frame);
          Serial.print(v1+30,DEC);
          Serial.print(v2+30,DEC);
          Serial.print(v3+30,DEC);
          Serial.print("\r");
        }
        Serial.flush();
    }
    else if(counter>=120){
        Serial.print(end_frame);        
        Serial.flush();
        delay(50);
        pinMode(LED1,INPUT_PULLUP);//PA1
        
        Serial.end();
        //pinMode(0,INPUT);//Set TXD to pull-up input
        //PORTA.DIRCLR = PIN6_bm; 
        //PORTA.DIRCLR = PIN7_bm; 
        msg_count--;
        if(msg_count==0)
        {
          statusTimer = 1;
          alarm_active = 1;
        }
        else
        {
          counter = 0;
        }
        ADC_off();
    }
  }
  else
  {
    if(counter>=60)
    {
      counter=0;
      statusTimer++; 
      if(statusTimer>=INTERVAL_MN)
      {
        PORTA.DIRSET = PIN1_bm; 
        statusTimer=0;
        ledcount = 3;
        msg_count = 3;
      }
    }
  }
    //if((PORTA.IN & PIN3_bm)!=0)
    //if((PORTA.IN & PIN3_bm)==0)
    if(alarmin != (PORTA.IN & PIN3_bm))   
    {
      alarmin = (PORTA.IN & PIN3_bm);   
      alarm_active = 0;
      counter = 0;
      ledcount = 20;
      PORTA.DIRSET = PIN1_bm; 
      msg_count = 3;
    }
  counter++;
  sleep_cpu();
}
