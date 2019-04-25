 /*
 * Sensor cablitration system for pulse oximeter
 * Author: Emeka Allison
 * An Arduino Uno program
 * 
 * Description:
 * Measures values from the oximeter sensor filtering circuit and
 * temperature sensor
 * Temperature sensor is LM35
 * Test and cablirates both the pulse sensor and temperature sensor
 * 
 * Components:
 * Arduino Uno
 * IR and Red LEDs
 * Two swicthing MOSFETS
 * Bandpass filters
 * Buffers
 * WiFi module
 * Temperature sensor
 * 
 */

#include<SoftwareSerial.h>
#include<WiFi.h>
#include<SPI.h>
#include<LiquidCrystal.h>

SoftwareSerial esp(A5,2);//RX,TX 


//analog inputs from sensors
const int R_input = A0; //red AC input
const int R_baseline =A1; //red baseline input
const int IR_input = A2; //infrared AC input
const int IR_baseline=A3; //infrared baseline input
const int tempinput = A4; //temperature sensor input

//Signal readings
int ir_AC;  //infrared light AC signal
int ir_DC;  //infrared light baseline
int red_AC; //red light AC signal
int red_DC; //red light baseline
int tempread; //temperature reading

//pulse signal sampling parameters
int pulsecount=0; //recongnized number of pulse
int redpulse[10];  //value of red pulse samples
int irpulse[10];  //value of infrared pulse samples
int irsignal;
int redsignal;

//pulse wave parameters
int redmax;   //peak of red ac signal 
int redmin;   //trough red ac signal
int irmax;    //peak of ir ac signal
int irmin;    //trough of ir ac signal
float redpeak, redtrough, irpeak, irtrough;

//Sampling and Driving pins
volatile int red_samp=3;  //red Led sample control
volatile int ir_samp=4;   //infrared Led sample control
volatile int redLed = 5; //red PWM
volatile int irLed = 6;  //infrared PWN

//int alpha = 0.75;
int tempoffset=0.01;
float tempcab=0;

//Vital readings
float R;  //empirical ratio
int HR; //Heartrate reading
float SpO2;  //SpO2 reading

const int Wifi_Pin = 1;
boolean last_wf = LOW;  //last state of bluetooth pin
boolean wifi_cmd=LOW;
int lastmillis=0;
int currentmillis=0;

/*
String IP="";
boolean No_IP=false;
String host =
*/
LiquidCrystal lcd(7,8,9,10,11,12);


char txt3[] = "INSERT PROBE!!!";

void setup() {

  Serial.begin(9600);
  lcd.begin(16,4);
  lcd.setCursor(2,0);
  lcd.print("EEM WIRELESS");
  lcd.setCursor(2,1);
  lcd.print("PULSE OXIMETER");
  delay(2000);
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,0);
  lcd.print("Starting...");
  delay(2000);
  //wifi_int();
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(2,0);
  lcd.print("HEALTH STATUS");
  pinMode(redLed,OUTPUT);
  pinMode(irLed,OUTPUT);
  pinMode(red_samp,OUTPUT);
  pinMode(ir_samp,OUTPUT);
  
  attachInterrupt(Wifi_Pin, startBT, RISING);
}

/*
void wifi_init(){
//  connect_wifi("AT",100);
  connect_wifi("AT+CWMODE=1",100);
  connect_wifi("AT+RST",1000);
  check4IP(5000);
  if(!No_IP)
  {
    Serial.println("Connecting Wifi...");
    connect_wifi("AT+CWJAP=\"Connectify-ali\",\"qwertyuiop\"",3000);
  }
  Serial.println("WiFi Connected");
  get_ip();
  connect_wifi("AT+CIPMUX=1",100);
  co
}
*/

//Temperature function
float calcTemp(int val, int offset, float cab){
  return(((val*4.8828)-offset)/10)+cab;
}


void startBT(){
  
  if(Wifi_Pin==HIGH && last_wf==LOW){
    wifi_cmd=HIGH;
  }
  
  else if(Wifi_Pin==HIGH && last_wf==HIGH){
    wifi_cmd=LOW;
  }
  last_wf = wifi_cmd;
}

void pulsesensor(){
  
  analogWrite(redLed,255);  //turn on red LED to maximum brightness
  digitalWrite(red_samp,HIGH); //sample red light signal
  
  for(int j=1;j<=10;j++){
      redpulse[j]=redpulse[j-1];  
      red_AC = analogRead(R_input);
      redsignal=map(redpulse[j],0,1023,0,600);
      if(redpulse[j]==min(redpulse[j],102))
      redmin= redpulse[j];  //max set  
      if(redpulse[j]==max(redpulse[j],158))
      redmax=redpulse[j]; //min set 
      redpulse[j-1] = red_AC;
  }
   //read red dc signal
  red_DC = analogRead(R_baseline); //read red baseline
  delay(20);
 
  int i = 0;
  digitalWrite(red_samp,LOW); //stop sampling red light signal
  analogWrite(redLed,i); //turn off red LED
  delay(20);
  
  analogWrite(irLed,255);  //turn on ir LED to maximum brightness
  digitalWrite(ir_samp,HIGH); //sample red light signal
  
  for(int j=0;j<=10;j++){    
      irpulse[j]=irpulse[j-1];  
      ir_AC = analogRead(IR_input); //read ir ac signal
      //check for minimum red ac signal
      irsignal=map(irpulse[j],0,1023,0,600);
      if(irpulse[j]==min(irpulse[j],102))
      irmin= irpulse[j];  //max set
      //check for maximum ir ac signal
      if(irpulse[j]==max(irpulse[j],158))
      irmax=irpulse[j]; //min set 
      irpulse[j-1] = ir_AC;
  }
  
  ir_DC = analogRead(IR_baseline);
  delay(20);

  i = 0;
  digitalWrite(ir_samp,LOW);
  analogWrite(irLed,i);
  delay(20);

  pulse();
  
}

void pulse(){
  
  float lastir; //ir signal checkpoint
  
  for(int i=1;i<=10;i++){
  redpeak = redpeak + (redmax*5.0/1024);
  redtrough = redtrough + (redmin*5.0/1024);
  irpeak = irpeak + (irmax*5.0/1024);
  irtrough = irtrough + (irmin*5.0/1024);
  }
  
  redpeak = redpeak/10;
  redtrough = redtrough/10;
  irpeak = irpeak/10;
  irtrough = irtrough/10;

  for(int i=0; i<=10;i++){
    if (irpulse[i]=irmin){
      lastir=irtrough;
    }
    else if(irpulse[i]=irmax){
      if(lastir=irtrough){
        pulsecount=+1;
      }
      lastir=irpeak;
    }
  }   
}

void loop() {
  
  tempread = analogRead(tempinput); //read temperature input
  float temperature = calcTemp(tempread, tempoffset, tempcab); //calculate temperature
  
  
  //initiate pulse sensor
  pulsesensor();

  //if(pulsecount>=4){
    
    R =calcR(redpeak, redtrough, irpeak, irtrough); //empirical ratio
    SpO2 = 10.002*(R*R*R)-52.887*(R*R)+26.871*R+98.283; //SPO2 value
    HR =(pulsecount/millis())*60000;  //Heartrate value
    
  //}

  
  //Display every one second
  if(currentmillis>=1000){
    
    lastmillis=millis();
    
    //Serial reading display
    Serial.print("Temp: ");
    Serial.print(temperature, 0);
    Serial.print("°C  ");
    Serial.print("R: ");
    Serial.print(R);
    Serial.print(" SpO2:");
    Serial.print(SpO2, 0);
    Serial.print(" HR: ");
    Serial.print(HR);
    Serial.print(" ");
    Serial.print(lastmillis);
    Serial.print(" ");
    Serial.println(irsignal);
     
    //LCD reading display
    lcd.setCursor(0,1);
    lcd.print("               ");
    lcd.print("SpO2:     %");
    lcd.setCursor(5,1);
    lcd.print(SpO2);
    lcd.setCursor(0,2);
    lcd.print("HR:  bpm");
    lcd.setCursor(3,1);
    lcd.print(HR);
    lcd.setCursor(0,3);
    lcd.print("Temp:   C");
    lcd.setCursor(5,3);
    lcd.print(temperature);
    currentmillis = 0;
    
  }
  currentmillis = millis()-lastmillis;
    
}

//Empirical calibration
float calcR(float rs, float rb, float irs, float irb){
  return((rs/rb)/(irs/irb));
}

