#include "MegunoLink.h"
float count[]={0,0};
float reading[]={0,0,0,0};
float EST[]={0,0};
float EST2[]={0,0};
float P_I_D2[]= {0,0};
float P_I_D[]= {0,0};
unsigned long input[]={0,0};
int M1F,M1B,M2F,M2B;
float encoder_count, pgain1 ,pgain2,dgain,igain,igain2 ,dgain2,setpoint,I,I2,P,P2,D,D2 ,encoder_count2,time_counter, avg_speedy, avg_speedy2,Xkp,Xkp1,Tsright,Tsleft,PWM;

 float R = 0.45;
  float H = 1;
  float Q = 0.102;
  float Pk = 0;
  float U_hat = 0;
  float KG=0;
  int now_time,start_time;


void setup() {
  // put your setup code here, to run once:
M1F=6;
  M1B=5;

  M2F = 9;
  M2B = 10;
  // put your setup code here, to run once:
  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
   pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);
    pinMode(2, INPUT);
     pinMode(3, INPUT);
    // attachInterrupt(digitalPinToInterrupt(2), encode1, CHANGE);
 attachInterrupt(digitalPinToInterrupt(3), encode2, CHANGE);
  Serial.begin(9600);
   
   setpoint = 1 ;
 digitalWrite(M1B,LOW);
 digitalWrite(M2B,LOW);
 
  pgain2=2.53;//3.83;
dgain2=0.033;//0.6;
igain2=2.3;


  start_time = millis();


EST[0]= 0;
EST2[0]= 0;

}

void loop() {
  
now_time = millis();
 int diff_time = now_time - start_time; 
if (diff_time >= 250)
{

  start_time = now_time;
      reading[3] = (encoder_count2/20);


EST2[1] =KALMAN(reading[3]);


//PWM = 0.215*pow(P_I_D2[1],2)+13.7*(P_I_D2[1])-0.231;
//if(PWM<0){PWM = 0;}
      
 P2=(setpoint-EST2[1])*(pgain2);//3
   I=I+(setpoint-EST2[1]);
   I2 =17 + igain2*I;//0.5
   //I2=I*igain2;
  D2=((setpoint-EST2[1])-(setpoint-EST2[0]))*dgain2;
  P_I_D2[1]= P2+I2+D2;
  analogWrite(M2F,P_I_D2[1]);
 Serial.println(EST2[1]);
EST2[0]=EST2[1];
P_I_D2[0]=P_I_D2[1];
 time_counter++;
encoder_count2=0; 
encoder_count=0; 


}
    }
  

void encode2()
{
  encoder_count2++;
   //Serial.println(encoder_count);
 
} 
void encode1()
{
  encoder_count++;
   //Serial.println(encoder_count);
 
} 
float KALMAN(float U)
{
 
  KG = Pk*H/(H*Pk*H+R);
  U_hat = U_hat + KG*(U-U_hat);
  Pk = (1-KG*H)*Pk+Q;
  //Serial.println(KG);
  return U_hat;
  }
