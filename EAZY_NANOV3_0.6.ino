//-----------NANOv3 & TMC2130 v06 by Antony Mikkonen
//-----------TMCSTepper.h by Teemu Mäntykallio
//-----------Serial debug ON
//-------------Baudrate 250000
//-------------Stepper motor OFF   == 0
//-------------Stepper motor ON    == 1
//-------------Stepper motor HOLD  == 3
//-------------Stepper motor H/OFF == 4

#include <TMCStepper.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x08 // nano

#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000

#define STALL_VALUE      1 // [-64..63]

#define EN_PIN           7 // Enable
#define DIR_PIN          5 // Direction
#define STEP_PIN         6 // Step
#define CS_PIN           10 // Chip select
#define SW_MOSI          11 // Software Master Out Slave In (MOSI)
#define SW_MISO          12 // Software Master In Slave Out (MISO)
#define SW_SCK           13 // Software Slave Clock (SCK)
#define DIAG1             4

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075
                      

// Select your stepper driver type
TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
//TMC5160Stepper driver(CS_PIN, R_SENSE);
//TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

using namespace TMC2130_n;


int val = 0;
int paalla = false;
byte data_to_echo = 0; //I2C receive
byte data_to_pi =0;  //I2C send

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

ISR(TIMER1_COMPA_vect){
  //STEP_PORT ^= 1 << STEP_BIT_POS;
  if (paalla == true) {
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));}

  
}

void setup() {
  Wire.begin(SLAVE_ADDRESS); //I2C
  Wire.onReceive(receiveData); //I2C
  Wire.onRequest(sendData); //I2C
  SPI.begin();
  Serial.begin(250000);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  digitalWrite(EN_PIN, LOW);
  pinMode(DIAG1, INPUT_PULLUP);
 
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.en_pwm_mode(true);
  driver.THIGH(0);
  driver.semin(0);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);
  driver.diag1_stall(true);
  digitalWrite( DIR_PIN,  LOW );
  data_to_pi=0;
  // Set stepper interrupt
  {
    cli();//stop interrupts
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS11 bits for 8 prescaler
    TCCR1B |= (1 << CS11);// | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();//allow interrupts
  }
}

void loop() {
  
  static uint32_t last_time=0;
  uint32_t ms = millis();
  val = digitalRead(DIAG1);
  while(Serial.available() > 0) {
   
    int8_t read_byte = Serial.read();
    #ifdef USING_TMC2660
      if (read_byte == '0')      { TIMSK1 &= ~(1 << OCIE1A); driver.toff(0); }
      else if (read_byte == '1') { TIMSK1 |=  (1 << OCIE1A); driver.toff(driver.savedToff()); }
    #else
      if (read_byte == '0')      { TIMSK1 &= ~(1 << OCIE1A); digitalWrite( EN_PIN, HIGH ); }
      else if (read_byte == '1') { TIMSK1 |=  (1 << OCIE1A); digitalWrite( EN_PIN,  LOW ); }
    #endif
    else if (read_byte == '+') { if (OCR1A > MAX_SPEED) OCR1A -= 20; }
    else if (read_byte == '-') { if (OCR1A < MIN_SPEED) OCR1A += 20; }
 
    else if(read_byte =='3') {
      paalla=true;
     
    }
   if(read_byte=='4'){
      paalla=false;
      
    
    }

    
      


    
  }



  if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();
    if(val==0){
      paalla=false;
      
      
    }
    if (paalla==true){
      data_to_pi = 7;
      //Wire.endTransmission(true);
    }
    if (paalla==false){
      data_to_pi = 6;
      //Wire.endTransmission(true);
    }
    
    Serial.print("0 ");
    Serial.print(drv_status.sg_result, DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);
    Serial.print("\n");
    Serial.print(val);
    Serial.print("\n");
    Serial.print("\n");
    Serial.print(paalla);
    Serial.print("\n");
    
  }
  else if (data_to_echo == 4 ) { 
    
    digitalWrite( DIR_PIN,  LOW );
    paalla = true;
    data_to_echo = 0;
    //startCommand = true;  
  }
  else if (data_to_echo == 5) { 
      //sensorIn = true;
      
      digitalWrite( DIR_PIN,  HIGH) ;
      paalla = true;
      data_to_echo =0;
  }
}
void receiveData(int bytecount)
{
  for (int i = 0; i < bytecount; i++) {
    data_to_echo = Wire.read();
    Serial.print("\ndata to echo:");
    Serial.print(data_to_echo);
    Serial.print("\n");
  }
}
void sendData()
{
  Wire.write(data_to_pi);
}
