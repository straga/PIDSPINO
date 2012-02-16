
	
// TRIAC
    #define triac_control 5
    #define powerIndicator 1 // indicator
    #define sensorPin 0 // potentiometer 
    #define irq_Pin 2 
    #define powerOn 4
    // when using values in the main routine and IRQ routine must be volatile value
    volatile byte flag_bit1 = LOW; // declare IRQ flag
    int analogValue = 0;
    int testValue = 45;
    // HIGH = 1, LOW = 0
    

    int pwm_pin    = 3;   // ШИМ-сигнал датчика
    int pwm_val          = 0;   // значение ШИМ-сигнала    
  
// HALL SENSOR 
    #define  hallsensor 3
    volatile byte rpmcount;
    unsigned int rpm;
    unsigned long timeold;
  

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
    //double consKp=1, consKi=0.4, consKd=0.01; 100% work
    //double aggKp=0.4, aggKi=0.001, aggKd=1;
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
  digitalWrite(powerOn, 1); // pull up on
  
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
  //Serial.begin(19200);
  //initialize the variables we're linked to
  Input = rpm;
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(40, 922);  
 
// Mach3
  pinMode(pwm_pin, INPUT);
 
  
}

void loop() 
{
 
  if (!digitalRead(powerOn)) 
    {
         digitalWrite(powerIndicator, 1);        
    }
  else 
    {
        // digitalWrite(powerIndicator, 0);
        pwm_val    = pulseIn(pwm_pin, HIGH);   
        lcd.commandWrite(0xC0); 
        delay(1000);   
        lcd.printIn(itoa(pwm_val , "", 10));
        
        digitalWrite(powerIndicator, 0);
 
        // lcd.setCursor(0, 1);
        // lcd.print(analogValue);
        // lcd.setCursor(8, 1);
        // lcd.print(analogValue*9);
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
       lcd.commandWrite(0x80+4);               //Line 1, position 4!
       lcd.printIn(itoa(rpm, "", 10));
      }
     }   
    
//PID   
  analogValue = analogRead(sensorPin); 
 
  if ( rpm <  kDev)
    {
       Input = kDev;
    }
  else
    {
       Input = rpm/kDev;
    }
 
  Setpoint      = analogValue;
    
  myPID.Compute();
  //analogValue = Output;
  analogValue = analogmax-analogValue+analogmin;  
  // lcd.commandWrite(0xC0+4); 
  // lcd.printIn(itoa(, "", 10)); 

    
//regul   
   
  if ((flag_bit1 == 1) && (digitalRead(powerOn)== 0)) 
    {
      if (analogRead(sensorPin) > 0h
      {
      delayMicroseconds(analogValue * 9); // set value between 4 and 14
      digitalWrite(triac_control, 1); //triac on 
      delayMicroseconds(100); 
      digitalWrite(triac_control, 0);  //triac off 
      flag_bit1 = 0; // clear flag           
      }   
  
}
    
  
  //lcd.commandWrite(0xC0+5);               //Line 1, position 5!   
  //lcd.printIn(itoa(analogValue, " ", 10));
    
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
