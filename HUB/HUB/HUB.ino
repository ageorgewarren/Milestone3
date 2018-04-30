// BinaryDataFromProcessing
// These defines must mirror the sending program:
const char HEADER       = 'H';
const char A_TAG    = 'M';
const char B_TAG    = 'X';
const int  TOTAL_BYTES  = 8; // the total bytes in a message
int X;
int Y;
int Redled=13;
int Greenled=12;
int Yelled=11;
 
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(Redled,OUTPUT);
  pinMode(Greenled,OUTPUT);
  pinMode(Yelled,OUTPUT);
}

void loop(){
  if ( Serial.available() >= TOTAL_BYTES)
  {
     if( Serial.read() == HEADER)
    {
      char tag = Serial.read();
      if(tag == A_TAG)
      {
        //Collect integers
        int a = Serial.read() * 256; 
        a = a + Serial.read();
        int b = Serial.read() * 256;
        b = b + Serial.read();
        int c = Serial.read() * 256;
        c = c + Serial.read();
        
        
        if(a==1){
          digitalWrite(Redled,HIGH);
          digitalWrite(Greenled,LOW);
          digitalWrite(Yelled,LOW);
        }
        if(a==2){
          digitalWrite(Redled,LOW);
          digitalWrite(Greenled,HIGH);
          digitalWrite(Yelled,LOW);
        }
        if(a==3){
          digitalWrite(Redled,LOW);
          digitalWrite(Greenled,LOW);
          digitalWrite(Yelled,HIGH);
        }
        if(a==0){
          digitalWrite(Redled,LOW);
          digitalWrite(Greenled,LOW);
          digitalWrite(Yelled,LOW);
        }

        Serial1.print('<');
        Serial1.print(a);
        Serial1.print(',');
        Serial1.print(c);
        Serial1.println('>');
      }
    }
  }
}
