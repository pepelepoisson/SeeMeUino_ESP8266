#include<Wire.h>
#include "FastLED.h" // Librarie required for addressable LEDs

// Code configuration
#define DEBUG_SERIAL 0  // Set to 1 to output detailed data to serial
#define MINIMAL_SERIAL 1 // Set to 1 to output minimal status data to serial
#define USE_PHOTOCELL 0 // Set to 1 to include photocell readings in state switching

#define NUM_LEDS 300  // How many leds in your strip?
#define LOW_BRIGHTNESS 10  // Set LEDS brightness
#define HIGH_BRIGHTNESS 100  // Set LEDS brightness
#define DATA_PIN 14
#define MOSFET_GATE 16
//#define MOSFET_GATE 13
#define RED_LED 15
#define BLUE_LED 13
#define GREEN_LED 12
#define IMU_VCC  2 // Pin to power IMU with 3.3V 
#define PushB1 0
#define Button_1_On  (!digitalRead(PushB1))
#define PhotocellPin 0 // the cell and 10K pulldown are connected to a0
#define FRAMES_PER_SECOND  120
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define SET_IDLE_MILLISECONDS 5000 // how many seconds at idle before moving to sleep?
#define SET_PANIC_MILLISECONDS 5000 // how many seconds before moving away from Panic_mode?
#define SET_BRAKING_MILLISECONDS 1000 // how many seconds before moving away from Braking_mode?

// Variables used in CheckLight() routine
String light_status=String("unknown");
int photocellReading=0; // the analog reading from the analog resistor divider
int day_limit=250, night_limit=300; // analog reading levels corresponding to switching from day to night - difference used to avoid toggling between two levels when light level is borderline

// Variables used in CheckAccel() routine
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
String accel_status=String("unknown");
#define ACCELEROMETER_ORIENTATION 0     // 0, 1, 2, 3 or 4 to set the orientation of the accerometer module
int a_forward=0,a_sideway=0,a_vertical=0;
int a_forward_offset=0,a_sideway_offset=0,a_vertical_offset=0;
float a_forward_long_lag=0.0, a_sideway_long_lag=0.0, a_vertical_long_lag=0.0, a_ratio_long_lag=0.0;
float a_forward_lag=0.0, a_sideway_lag=0.0, a_vertical_lag=0.0, a_ratio_lag=0.0;
float a_forward_change=0.0, a_sideway_change=0.0, a_vertical_change=0.0, a_ratio_change=0.0;  
float long_lag_coef=0.005, lag_coef=0.1;
float a_forward_threshold=10.0, a_sideway_threshold=10.0, a_vertical_threshold=15.0;
float a_ratio_multiplier=200.0;
float idle_test_threshold=2.0,fallen_test_multiplier=0.75, strong_braking_test_threshold=50.0, long_braking_test_threshold=20.0;
//int idle_test_min_count=30, fallen_test_min_count=5, strong_braking_test_min_count=5, long_braking_test_min_count=40;
#define idle_test_min_count 30
#define fallen_test_min_count 5
#define strong_braking_test_min_count 5
//#define strong_braking_test_min_count 1
#define long_braking_test_min_count 40
int idle_test_count=0, fallen_test_count=0, strong_braking_test_count=0, long_braking_test_count=0;
unsigned long start_time=0, current_time=0, elapsed_milliseconds=0;



CRGB leds[NUM_LEDS];  // Define the array of leds

void setup() {

  Serial.begin(115200);
  //while (!Serial);

  pinMode(IMU_VCC,OUTPUT);
  digitalWrite(IMU_VCC,HIGH);

  delay(500);
  
  // Set up MPU 6050:
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  pinMode(PushB1,INPUT);
  digitalWrite(PushB1,HIGH);  // Configure built-in pullup resistor for push button 1
  
  pinMode(MOSFET_GATE,OUTPUT);
  digitalWrite(MOSFET_GATE,HIGH);
  
  pinMode(RED_LED,OUTPUT);
  digitalWrite(RED_LED,LOW);  

  pinMode(GREEN_LED,OUTPUT);
  digitalWrite(GREEN_LED,LOW);
  
  pinMode(BLUE_LED,OUTPUT);
  digitalWrite(BLUE_LED,LOW);
  
  // LEDs strip
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);  
  FastLED.setBrightness(LOW_BRIGHTNESS); 
  
  //delay(100);

  // setup starting angle
  // collect the data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

Serial.println("Starting ... ");
  CalibrateAccel();
  Serial.println("Calibration...");
  Serial.print("a_forward_offset = ");
  Serial.print(a_forward_offset);  
  Serial.print(" a_sideway_offset = ");
  Serial.print(a_sideway_offset);  
  Serial.print(" a_vertical_offset = ");
  Serial.println(a_vertical_offset); 
   
 //Initialize CheckAccel() routine
  a_forward_long_lag=a_forward_offset;
  a_sideway_long_lag=a_sideway_offset;
  a_vertical_long_lag=a_vertical_offset;
  a_forward_lag=a_forward_offset;
  a_sideway_lag=a_sideway_offset;
  a_vertical_lag=a_vertical_offset;  
  idle_test_count=idle_test_min_count+1;

  
}
// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {sinelon, juggle, bpm, rainbow, rainbowWithGlitter, confetti };
char* SimplePatternNames[]={"sinelon", "juggle", "bpm", "rainbow", "rainbowWithGlitter", "confetti" };
uint8_t gCurrentPatternNumber = 1; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void LED(String pattern){
  if (pattern=="idle"){
    digitalWrite(MOSFET_GATE,HIGH);
    
    digitalWrite(RED_LED,LOW);
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(BLUE_LED,HIGH);
    
    FastLED.setBrightness(LOW_BRIGHTNESS); 
    for (int i = NUM_LEDS; i >=0; i--){
      leds[i]=CRGB::Blue;
    }
    //sinelon();
  }
  
  if (pattern=="cruising"){
    digitalWrite(MOSFET_GATE,HIGH);

    digitalWrite(RED_LED,LOW);
    digitalWrite(GREEN_LED,HIGH);
    digitalWrite(BLUE_LED,LOW);
        
    FastLED.setBrightness(LOW_BRIGHTNESS); 
    // Call the pattern function once, updating the 'leds' array
    //juggle();
    gPatterns[gCurrentPatternNumber]();
  }
  
  if (pattern=="braking"){
    digitalWrite(MOSFET_GATE,HIGH);
    
    digitalWrite(RED_LED,HIGH);
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(BLUE_LED,LOW);
    
    FastLED.setBrightness(HIGH_BRIGHTNESS); 
      for (int i = NUM_LEDS; i >=0; i--){
      leds[i]=CRGB::Red;
    }
  }
  
  if (pattern=="panic"){      
    digitalWrite(MOSFET_GATE,HIGH);

    digitalWrite(RED_LED,HIGH);
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(BLUE_LED,LOW);   
    
    //FastLED.setBrightness(HIGH_BRIGHTNESS);
    FastLED.setBrightness(LOW_BRIGHTNESS);
    for (int i = NUM_LEDS; i >=0; i--){
      if (i%3==0){leds[i]=CRGB::Blue;}
      else {leds[i]=CRGB::Red;}
    }
  }
  
  if (pattern=="off"){
    for (int i = NUM_LEDS; i >=0; i--) {
      //leds[i]=CRGB::Black;
      leds[i].nscale8(230);
    }
    digitalWrite(MOSFET_GATE,LOW);    
    
    digitalWrite(RED_LED,LOW);
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(BLUE_LED,LOW);
  }

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);       
  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
}

void CheckBatteryVoltage(){
  // Nothing yet
}


void alloff() {
  for (int i = NUM_LEDS; i >=0; i--) {
    leds[i]=CRGB::Black;
    delay(20);
    FastLED.show();
  }
}
void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13,0,NUM_LEDS);
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16(i+7,0,NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

enum {Sleep_mode,Idle_mode,Cruising_mode,Braking_mode,Stopped_mode,Panic_mode} condition=Sleep_mode;




void loop() {
  CheckLight();
  CheckAccel();
switch (condition) {
    
    case Sleep_mode:
      Serial.print("Sleep_mode: ");
      LED("off");
      if (accel_status!="idle" & light_status=="night"){condition=Cruising_mode;}
      break;
      
    case Idle_mode:
      Serial.print("Idle_mode: ");
      LED("idle");
      current_time=millis();
      elapsed_milliseconds=current_time-start_time;
      if (elapsed_milliseconds>SET_IDLE_MILLISECONDS){condition=Sleep_mode;}
      if (accel_status!="idle" & light_status=="night"){condition=Cruising_mode;}
//      if Button_1_On {CheckBatteryVoltage();}
      break;
      
    case Cruising_mode:
      Serial.print("Cruising_mode: ");
      if (light_status=="night"){LED("cruising");}else{LED("off");}
      if (accel_status=="strong_braking"||accel_status=="long_braking"){condition=Braking_mode;}
      if (accel_status=="fallen"){start_time=millis();condition=Panic_mode;}
      if (accel_status=="idle" & light_status=="night"){start_time=millis();condition=Idle_mode;}
/*      if Button_1_On{
        delay(50);
        while(Button_1_On){}
        nextPattern();
      }
  */
      break;
      
    case Braking_mode:
      Serial.print("Braking_mode: ");
      LED("braking");
      if (accel_status!="strong_braking" && accel_status!="long_braking"){condition=Cruising_mode;}
      if (accel_status=="fallen"){condition=Panic_mode;}
      break;
      
    case Panic_mode:
      Serial.print("Panic_mode: ");
      LED("panic");
      current_time=millis();
      elapsed_milliseconds=current_time-start_time;
      if (elapsed_milliseconds>SET_PANIC_MILLISECONDS){condition=Sleep_mode;}
      if (accel_status!="fallen"){condition=Cruising_mode;}
      break;
   }  

  #if MINIMAL_SERIAL
     SerialOutput(); 
  #endif
     
  //delay(100);

}void CalibrateAccel(){
  // Reads acceleration from MPU6050 to evaluate installation offsets.
  // Tunables: 
  // Output values: a_forward_offset, a_sideway_offset, a_vertical_offset 

  for (int i=0; i<100;i++){
    // Get accelerometer readings
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
    // Convert to expected orientation - includes unit conversion to "cents of g" for MPU range set to 2g
    a_forward = (ACCELEROMETER_ORIENTATION == 0?-AcX:(ACCELEROMETER_ORIENTATION == 1?-AcX:(ACCELEROMETER_ORIENTATION == 2?-AcX:(ACCELEROMETER_ORIENTATION == 3?-AcY:AcY))))/164.0;
    a_sideway = (ACCELEROMETER_ORIENTATION == 0?AcY:(ACCELEROMETER_ORIENTATION == 1?AcZ:(ACCELEROMETER_ORIENTATION == 2?-AcZ:(ACCELEROMETER_ORIENTATION == 3?AcZ:-AcZ))))/164.0;
    a_vertical = (ACCELEROMETER_ORIENTATION == 0?AcZ:(ACCELEROMETER_ORIENTATION == 1?-AcY:(ACCELEROMETER_ORIENTATION == 2?AcY:(ACCELEROMETER_ORIENTATION == 3?AcX:AcX))))/164.0;
    
    a_forward_offset=a_forward_offset+a_forward;
    a_sideway_offset=a_sideway_offset+a_sideway;
    a_vertical_offset=a_vertical_offset+a_vertical;
  }
  a_forward_offset=a_forward_offset/100;
  a_sideway_offset=a_sideway_offset/100;
  a_vertical_offset=a_vertical_offset/100;
}




void CheckAccel(){
  // Reads acceleration from MPU6050 to evaluate current condition.
  // Tunables: 
  // Output values: still, cruising, braking, fallen, unknown

  // Get accelerometer readings
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
  // Convert to expected orientation - includes unit conversion to "cents of g" for MPU range set to 2g
  a_forward = (ACCELEROMETER_ORIENTATION == 0?-AcX:(ACCELEROMETER_ORIENTATION == 1?-AcX:(ACCELEROMETER_ORIENTATION == 2?-AcX:(ACCELEROMETER_ORIENTATION == 3?-AcY:AcY))))/164.0;
  a_sideway = (ACCELEROMETER_ORIENTATION == 0?AcY:(ACCELEROMETER_ORIENTATION == 1?AcZ:(ACCELEROMETER_ORIENTATION == 2?-AcZ:(ACCELEROMETER_ORIENTATION == 3?AcZ:-AcZ))))/164.0;
  a_vertical = (ACCELEROMETER_ORIENTATION == 0?AcZ:(ACCELEROMETER_ORIENTATION == 1?-AcY:(ACCELEROMETER_ORIENTATION == 2?AcY:(ACCELEROMETER_ORIENTATION == 3?AcX:AcX))))/164.0;

  //Serial.print("AcX: "); Serial.print(AcX);Serial.print(" AcY: "); Serial.print(AcY);Serial.print(" AcZ: "); Serial.print(AcZ);
  //Serial.print("a_forward:");Serial.print(a_forward);Serial.print(" a_sideway:");Serial.print(a_sideway);Serial.print(" a_vertical:");Serial.println(a_vertical);

  // Update long_lag references
  if (abs(a_forward-a_forward_long_lag)<a_forward_threshold){a_forward_long_lag=a_forward*long_lag_coef+(1-long_lag_coef)*a_forward_long_lag;}
  if (abs(a_sideway-a_sideway_long_lag)<a_sideway_threshold){a_sideway_long_lag=a_sideway*long_lag_coef+(1-long_lag_coef)*a_sideway_long_lag;}
  if (abs(a_vertical-a_vertical_long_lag)<a_vertical_threshold){a_vertical_long_lag=a_vertical*long_lag_coef+(1-long_lag_coef)*a_vertical_long_lag;}
  a_ratio_long_lag=a_ratio_multiplier*a_forward_long_lag/a_vertical_long_lag;
  
  // Update lag values
  a_forward_lag=a_forward*lag_coef+(1-lag_coef)*a_forward_lag;
  a_sideway_lag=a_sideway*lag_coef+(1-lag_coef)*a_sideway_lag;
  a_vertical_lag=a_vertical*lag_coef+(1-lag_coef)*a_vertical_lag;
  a_ratio_lag=a_ratio_multiplier*a_forward_lag/a_vertical_lag;

  // Update change values
  a_forward_change=a_forward_lag-a_forward_long_lag;
  a_sideway_change=a_sideway_lag-a_sideway_long_lag;
  a_vertical_change=a_vertical_lag-a_vertical_long_lag;
  a_ratio_change=a_ratio_lag-a_ratio_long_lag;

#if DEBUG_SERIAL
  Serial.print(" a_forward_change:");Serial.print(a_forward_change);
  Serial.print(" a_sideway_change:");Serial.print(a_sideway_change);
  Serial.print(" a_vertical_change:");Serial.print(a_vertical_change);
  Serial.print(" a_vertical:");Serial.print(a_vertical);
  Serial.print(" a_vertical_lag:");Serial.print(a_vertical_lag);
  Serial.print(" a_vertical_long_lag:");Serial.print(a_vertical_long_lag);
  Serial.print(" a_ratio_change:");Serial.println(a_ratio_change);
#endif

  // Evaluate current condition based on smoothed accelarations
  accel_status="cruising";

  // Test idle
  if(abs(a_vertical_change)<idle_test_threshold){idle_test_count++;}
  else{idle_test_count=0;}
  if(idle_test_count>=idle_test_min_count){
    accel_status="idle";
    }

  // Test fallen
  if(abs(a_sideway_lag)>fallen_test_multiplier*abs(a_vertical_lag)){fallen_test_count++;}
  else{fallen_test_count=0;}
  if(fallen_test_count>=fallen_test_min_count){
    accel_status="fallen";
    }

  // Test strong braking
  if(a_ratio_change>=strong_braking_test_threshold){strong_braking_test_count++;}
  else{strong_braking_test_count=0;}
  if(strong_braking_test_count>=strong_braking_test_min_count){
    accel_status="strong_braking";
    }

  // Test long braking
  if(a_ratio_change>=long_braking_test_threshold){long_braking_test_count++;}
  else{long_braking_test_count=0;}
  if(long_braking_test_count>=long_braking_test_min_count){
    accel_status="long_braking";
    }
  
}
void SerialOutput(){
  // Routine used to send status information via serial
  
  Serial.print(light_status);  
  Serial.print(" - ");
  Serial.print(accel_status);  
  Serial.print(" - ");
  if Button_1_On {Serial.print("button_on");}
  else {Serial.print("button_off");}
  Serial.print(" millis:");
  Serial.println(millis());
}


void CheckLight(){
  // Reads light level using photocell and writes status in light_status String.
  // Tunables: night_limit, day_limit
  // Output values: night, day, unknown
  photocellReading = analogRead(PhotocellPin);
  
  #if DEBUG_SERIAL
     Serial.print(" photocellReading: ");
     Serial.print(photocellReading);
  #endif
  
  if (photocellReading<night_limit){light_status="night";}
  if (photocellReading>day_limit){light_status="day";}
  // No changes in light_status for levels in between two limits to avoid constant toggling
  #if !USE_PHOTOCELL
     light_status="night";
  #endif
  
}


