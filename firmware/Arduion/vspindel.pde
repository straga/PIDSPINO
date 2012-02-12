//MACH3

//const int pwm_pin    = 11;   // ШИМ-сигнал датчика
//int pwm_val          = 0; // значение ШИМ-сигнала
// TRIAC
  #define triac_control 5
  #define powerIndicator 12 // indicator
  #define sensorPin 0 // potentiometer 
  #define irq_Pin 2 
  #define powerOn 4
  // when using values in the main routine and IRQ routine must be volatile value
  volatile byte flag_bit1 = LOW; // declare IRQ flag
  int analogValue = 0;
  int testValue = 45;
  // HIGH = 1, LOW = 0
  
// HALL SENSOR 
  #define  hallsensor 3
  volatile byte rpmcount;
  unsigned int rpm;
  unsigned long timeold;
  
// LCD
//  #include <LiquidCrystal.h>

    // initialize the library with the numbers of the interface pins
    //LiquidCrystal lcd(13, 11, 10, 9, 8, 7);
    //LiquidCrystal(rs, rw, enable, d4, d5, d6, d7)
//  LiquidCrystal lcd(13, NULL, 11, 7, 8, 9, 10); 


#include <LCD4Bit.h> 
//create object to control an LCD.  
//number of lines in display=1
LCD4Bit lcd = LCD4Bit(2); 

// PID
  #include <PID_v1.h>

  //Define Variables we'll be connecting to
  double Setpoint, Input, Output;

  //Define the aggressive and conservative Tuning Parameters
  //double aggKp=4, aggKi=0.2, aggKd=1;
  //double consKp=0.4, consKi=0.001, consKd=1;
  //double consKp=4, consKi=0.2, consKd=1;
///  double consKp=1, consKi=0.4, consKd=0.01; 100% work
 // double aggKp=0.4, aggKi=0.001, aggKd=1;
  double consKp=1, consKi=0.5, consKd=0.05;

  //Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

  unsigned long serialTime; //this will help us know when to talk with processing

int analogmin = 40; 
int analogmax = 920;

int kDev = 25; // rpm to analog - exp: max rpm = 22000 , analogmax = 920 : rpm/analogmax = kDev

  

void setup()  {
  pinMode(triac_control, OUTPUT);  
  pinMode(powerIndicator, OUTPUT);
  digitalWrite(triac_control, 0); // LED off
  digitalWrite(powerIndicator, 0); // LED off
  pinMode(irq_Pin, INPUT);
  pinMode(powerOn, INPUT);
  digitalWrite(irq_Pin, 1); // pull up on
  digitalWrite(powerOn, 0); // pull up on
  attachInterrupt(0, flag1, FALLING);  // interrupt 0 digital pin 2 connected ZC circuit


// Hall sensor  
  pinMode(hallsensor, INPUT);
  digitalWrite(irq_Pin, 1);
  attachInterrupt(1, rpm_fun, FALLING);

  rpmcount = 0;
  rpm = 0;
  timeold = 0;  
  
// LCD  
  lcd.init();
  lcd.commandWrite(0x80+1);               //Line 1, position 5!
  lcd.printIn("vSpindel v.1.0");
  delay(1000);           
  lcd.clear();  
  

  
// Pid
  Serial.begin(19200);
   //initialize the variables we're linked to
  Input = rpm;
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(40, 922);  
 
//Mach3
//  pinMode(pwm_pin, INPUT);
 
  
}

void loop() 
{
 
  if (!digitalRead(powerOn)) 
    {
     // digitalWrite(powerIndicator, 1); 
          
    }
  else 
    {
     // digitalWrite(powerIndicator, 0);
  // pwm_val    = pulseIn(pwm_pin, HIGH);   
//   lcd.commandWrite(0xC0+1); 
//   analogValue = analogRead(sensorPin);
//   
//   if (analogValue > 999)
//   {
//     analogValue = 999;
//   }
//   
//  delay(100);   
//  lcd.printIn(itoa(analogValue , "", 10));
// 
//   //    lcd.setCursor(0, 1);
//   //    lcd.print(analogValue);
//   //    lcd.setCursor(8, 1);
//   //    lcd.print(analogValue*9);
     
    }
    
  //rpm

if (rpmcount >= 50) 
    { 
     //Update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     //rpm = 60*1000/(millis() - timeold)*rpmcount;
       unsigned long time = millis() - timeold;
       float time_in_sec  = (float)time / 1000;
       float impuls_time  = (float)rpmcount / time_in_sec;
       rpm                = (int)impuls_time*60;
     
     timeold  = millis();
     rpmcount = 0;
        
    if (rpm < 10000) 
      { 
      lcd.commandWrite(0x80); 
      lcd.printIn("RPM:");   
      lcd.commandWrite(0x80+4); 
      lcd.printIn(" ");    
      lcd.commandWrite(0x80+5);               //Line 1, position 5!     
      lcd.printIn(itoa(rpm, "", 10));
      }
    else
      {
        lcd.commandWrite(0x80); 
       lcd.printIn("RPM:"); 
       lcd.commandWrite(0x80+4);               //Line 1, position 5!
       lcd.printIn(itoa(rpm, "", 10));
       
    
      }
 
     }   
    
 //PID   
  analogValue = analogRead(sensorPin); 
  
    
      
      if (analogValue < 40)
       {
         analogValue = 40;
       }
      if (analogValue > 920)
       {
         analogValue = 920;
       }  
  
 
 if ( rpm <  kDev)
{
  Input = kDev;
}
else
{
  Input         = rpm/kDev;
}
 
 // Setpoint      = analogValue*25; 
  
  Setpoint      = analogValue;
  
//  Setpoint      = 15000;
  
 //   Setpoint      = rpm;
 //   Input    = analogValue*25;
  
 // lcd.commandWrite(0xC0);              
 // lcd.print("Set:");     
 // lcd.commandWrite(0xC0+5);               
 // lcd.printIn(itoa(analogValue = analogRead(sensorPin), "", 10));  


  
//    double gap = abs(Setpoint-Input); //distance away from setpoint
//  if(gap<1000)
//  {  //we're close to setpoint, use conservative tuning parameters
//    myPID.SetTunings(consKp, consKi, consKd);
//  }
//  else
//  {
     //we're far from setpoint, use aggressive tuning parameters
    // myPID.SetTunings(aggKp, aggKi, aggKd);
//  }
   
  myPID.Compute();
//  analogValue = Output/25;
  analogValue = Output;
  
  analogValue = analogmax-analogValue+analogmin;
   
 // lcd.commandWrite(0xC0+4); 
 // lcd.printIn(itoa(, "", 10)); 

    
 //regul   
   
  if ((flag_bit1 == 1) && (digitalRead(powerOn)== 1)) 
    {
     // analogValue = analogRead(sensorPin);
      
   //   if (analogValue < 40)
       {
   //      analogValue = 40;
       }
     // if (analogValue > 900)
       {
    //     analogValue = 900;
       }  
      
      delayMicroseconds(analogValue * 9); // set value between 4 and 14
      //delaMicroseconds(9000); 
      digitalWrite(triac_control, 1); //triac on 
      delayMicroseconds(100); 
      digitalWrite(triac_control, 0);  //triac off 
      flag_bit1 = 0; // clear flag      
      
    }
  
 // lcd.commandWrite(0xC0+5);               //Line 1, position 5!
    
// lcd.printIn(itoa(analogValue, " ", 10));
  
  
 //pid for serial
 
//  if(millis()>serialTime)
  {
//    SerialReceive();
//    SerialSend();
//    serialTime+=500;
  }
 
  
}

void flag1() // set bit
{
  flag_bit1 = 1; 
} 

void rpm_fun()
 {
   rpmcount++;
   //Each rotation, this interrupt function is run
 }


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    Setpoint=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      Output=double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    myPID.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //
    
    if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(Setpoint);   
  Serial.print(" ");
  Serial.print(Input);   
  Serial.print(" ");
  Serial.print(Output);   
  Serial.print(" ");
  Serial.print(myPID.GetKp());   
  Serial.print(" ");
  Serial.print(myPID.GetKi());   
  Serial.print(" ");
  Serial.print(myPID.GetKd());   
  Serial.print(" ");
  if(myPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}




