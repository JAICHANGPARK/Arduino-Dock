/*
  Interrupt  
   attachInterrupt(pin,Callback,state );
   detachInterrupt(pin);
   int.0 = pin1
   int.1 = pin2
   int.2 = pin3
   int.3 = pin4
*/

int led = 13; 
int intr_pin1 = 1;
int intr_pin2 = 2;
int intr_pin3 = 3;
int intr_pin4 = 4;

int myPinCallback1()
{ 
  Serial.println("Interrupt_pin1");
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
}

int myPinCallback2()
{
  Serial.println("Interrupt_pin2");  
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
}

int myPinCallback3()
{
  Serial.println("Interrupt_pin3");
   pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
}

int myPinCallback4()
{
  Serial.println("Interrupt_pin4");
   pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
   digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
}


void setup()  
{
  Serial.begin(9600);
  Serial.println("Hello!");
  
   pinMode(led, OUTPUT);
   digitalWrite(led, LOW);
  
  pinMode(intr_pin1, INPUT_PULLUP);
  attachInterrupt(intr_pin1,myPinCallback1,FALLING );
  
  pinMode(intr_pin2, INPUT_PULLUP);
  attachInterrupt(intr_pin2,myPinCallback2,RISING );
  
  pinMode(intr_pin3, INPUT_PULLUP);
  attachInterrupt(intr_pin3,myPinCallback3,CHANGE );
  
  pinMode(intr_pin4, INPUT_PULLUP);
  attachInterrupt(intr_pin4,myPinCallback4,CHANGE );
}

void loop(){
  char c;
  
  if (Serial.available()) {
      
    c = Serial.read();
    if(c=='1') {
          detachInterrupt(intr_pin1);
          Serial.println("detachInterrupt_pin1");
      } 
      else if(c=='2') {
          detachInterrupt(intr_pin2);
          Serial.println("detachInterrupt_pin2");
      } 
      else if(c=='3') {
          detachInterrupt(intr_pin3);
          Serial.println("detachInterrupt_pin3");
      } 
      else if(c=='4') {
          detachInterrupt(intr_pin4);
          Serial.println("detachInterrupt_pin4");
      }
    }
     
  }
  
