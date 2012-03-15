	
// TRIAC
    #define triac_control 5 // Triac control - pin
    
//RPM control    
    #define sensorPin 0 // potentiometer or MACH3 - pin
    
//Power
    #define powerOn 4 // Power switch - pin 
    #define powerIndicator 1 // indicator GREEN LED -pin
    
//Zero Detect    
    #define zero_detect 2 //Zero detect - pin
    // when using values in the main routine and IRQ routine must be volatile value
    volatile byte zero_bit = LOW; // declare IRQ flag
    // HIGH = 1, LOW = 0
  
// HALL SENSOR 
    #define  hallsensor 3 //- pin
    unsigned int rpmcount;
    unsigned int rpm;
    unsigned long timeold;
  
//LCD
    //LiquidCrystal::LiquidCrystal(rs, enable, d0, d1, d2, d3)
    #include <Wire.h>
    #include <LiquidCrystal.h>
    LiquidCrystal lcd(13, 11, 7, 8, 9, 10);

// PID
    #include <PID_v1.h>

    //Define Variables we'll be connecting to
    double Setpoint, Input, Output;

    //Define the aggressive and conservative Tuning Parameters
    //double consKp=4, consKi=0.2, consKd=1;
    //double consKp=0.4, consKi=0.001, consKd=1;
    //double consKp=4, consKi=0.2, consKd=1;
    //double consKp=1, consKi=0.4, consKd=0.01; //100% work
    //double consKp=0.4, consKi=0.001, consKd=1;
    double consKp=0.1, consKi=0.5, consKd=0.05;

    //Specify the links and initial tuning parameters
    PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

    int analogValue = 0;
    int analogmin = 139; 
    int analogmax = 920;
    float kDev = 32.5; // rpm to analog - exp: max rpm = 22000 , analogmax = 920 : rpm/analogmax = kDev = 25

  

void setup()  {
  
  
//Triac control setup  
  pinMode(triac_control, OUTPUT);  
  digitalWrite(triac_control, 0); // LED off
//Power switch  
  pinMode(powerIndicator, OUTPUT);
  digitalWrite(powerIndicator, 0); // LED off
  pinMode(powerOn, INPUT);
  digitalWrite(powerOn, 1); // pull up on
//Zero detect  
  pinMode(zero_detect, INPUT);
  digitalWrite(zero_detect, 1); // pull up on
  attachInterrupt(0, zero_fun, FALLING);  // interrupt 0 digital pin 2 connected ZC circuit
// Hall sensor  
  pinMode(hallsensor, INPUT);
  digitalWrite(hallsensor, 1); // pull up on
  attachInterrupt(1, rpm_fun, FALLING);  // interrupt 1 digital pin 3 connected hall sensor

  rpmcount = 0;
  rpm = 0;
  timeold = 0;  
  
// LCD detect
  lcd.begin(16,2);               // initialize the lcd 
  lcd.home ();                   // go home
  lcd.print("Hello, ARDUINO ");  
  lcd.setCursor ( 0, 1 );        // go to the next line
  lcd.print (" vSpindel-2.0   ");    
  delay(1000);
  lcd.clear(); 
 
//PID

  Input = rpm;
  Setpoint = analogValue;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(6000, 30000);  
 

 
  
}

void loop() 
{
 
//Power switch and indicator = ON/OFF  
  if (!digitalRead(powerOn)) 
    {
         digitalWrite(powerIndicator, 1);        
    }
  else 
    {    
         digitalWrite(powerIndicator, 0);
    }
    

//RPM counter and show on LCD
if (rpmcount >= 50) 
    { 
       //Update RPM every 20 counts, increase this for better RPM resolution,
       //decrease for faster update
       //rpm = 60*1000/(millis() - timeold)*rpmcount;
       unsigned long time = millis() - timeold;
       float time_in_sec  = (float)time / 1000;
       float impuls_time  = (float)rpmcount / time_in_sec;
       rpm                = (int)impuls_time*60;
    
     rpmcount = 0; //reset
     timeold  = millis(); //set time
        
    if (rpm < 10000) 
      { 
       lcd.setCursor ( 0, 0 );
       lcd.print ("RPM:");
       lcd.setCursor ( 4, 0 );
       lcd.print (" ");
       lcd.setCursor ( 5, 0 );
       lcd.print (rpm);
      }
    else
      {     
       lcd.setCursor ( 0, 0 );
       lcd.print ("RPM:");
       lcd.setCursor ( 4, 0 );
       lcd.print (rpm);  
      }
  
 }    
    
//PID   
  analogValue = analogRead(sensorPin); 
  
//    lcd.setCursor ( 10, 0 );
//    lcd.print ((float)analogValue);
  

  
  Input = rpm;
  
//  Input = analogmax-Input+analogmin; 
//    if (Input > analogmax) // Limit for max delay time 920*9 = 8280;
//    {
//      Input = analogmax;
//    }
 
  //139 = a * 20 + b  и 920 = a * 920 + b
  //analogValue =((781*analogValue)+109480)/900; 
  float analog1 = 781*(float)analogValue;
  float analog2 = analog1+109480;
  float analog3 = analog2/900;
  
  analogValue = (int)analog3;
  
   
  Setpoint    = analogValue*kDev;  
  
    
  myPID.Compute();
  
  
  
  analogValue = Output/kDev;
  
  analogValue = analogmax-analogValue+analogmin;  
  
  
  if (analogValue > analogmax) // Limit for max delay time 920*9 = 8280;
    {
      analogValue = analogmax;
    }
   
  
  //analogValue = analogmax-analogValue+analogmin;

// debug 1 Real potontiometr and virtual.
//  lcd.setCursor ( 10, 0 );
//  lcd.print (Setpoint);
//  
//  lcd.setCursor ( 0, 1 );
//  lcd.print (analog3);

// debug 2 
//  lcd.setCursor ( 10, 0 );
//  lcd.print ((float)analogValue);
//  
//  lcd.setCursor ( 0, 1 );
//  lcd.print ((float)analogValue*9);
//
//  lcd.setCursor ( 10, 1 );
//  lcd.print (analog3*9);




//      lcd.setCursor ( 10, 0 );
//      lcd.print (Input);                  
//
//      
//      lcd.setCursor ( 0, 1 );
//      lcd.print (Setpoint); 
//      
//      lcd.setCursor ( 10, 1 );
//      lcd.print (float(analogValue));
       
      // lcd.setCursor ( 10, 1 );
      // lcd.print (analogValue);

   
//TRIAC delay control      
  if ((zero_bit == 1) && (digitalRead(powerOn)== 0)) 
    {
      if (analogRead(sensorPin) > 0)
        {
          delayMicroseconds(analogValue * 9); // set value between 4 and 14
          //delayMicroseconds(4000); 
          digitalWrite(triac_control, 1); //triac on 
          delayMicroseconds(100); 
          digitalWrite(triac_control, 0);  //triac off 
          zero_bit = 0; // clear flag           
        }   
  
    }
       
}

void zero_fun() // set bit
  {
    zero_bit = 1;
    // if zero detect set bit == 1
  } 

void rpm_fun()
 {
   rpmcount++;
   //Each rotation, this interrupt function is run
 }
