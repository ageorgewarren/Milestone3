// BinaryDataFromProcessing
// These defines must mirror the sending program:
const char HEADER       = 'H';
const char A_TAG    = 'M';
const char B_TAG    = 'X';
const int  TOTAL_BYTES  = 12; // the total bytes in a message
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
        int d = Serial.read() * 256;
        d = d + Serial.read();
        int e = Serial.read() * 256;
        e = e + Serial.read();
        
        
        if(b>0){
          X=b;
        }
        if(c>0){
          X=-1*(256-c);
        }
        if(d>0){
          Y=d;
        }
        if(e>0){
          Y=-1*(256-e);
        }
        if(b==0 && c==0){
          X=0;
        }
        if(d==0 && e==0){
          Y=0;
        }
        
        
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
        Serial1.print(X);
        Serial1.print(',');
        Serial1.print(Y);
        Serial1.println('>');
     
        

      }
    }
  }
}
