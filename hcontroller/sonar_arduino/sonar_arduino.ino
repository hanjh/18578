
#define SR04_TRIG_PIN 4
#define SR04_ECHO_PIN 2
//#define SR04_2_TRIG_PIN 3
//#define SR04_2_ECHO_PIN 4
#define FILTER_LENGTH 9

#define SCALING_FACTOR 1 //58 for cm, 148 for inch, 1 for max acc.

int aggregateIndex;
long aggregate0;
long aggregate1;
long aggregate_array0[FILTER_LENGTH];
long aggregate_array1[FILTER_LENGTH];

long measure_SR04_distance(int trig, int echo)
{
   digitalWrite(trig,LOW);
   delayMicroseconds(2);
   digitalWrite(trig,HIGH);
   delayMicroseconds(10);
   digitalWrite(trig,LOW);
   
   long time = pulseIn(echo,HIGH);
   //Serial.println("\n");
   //Serial.println(time);
   return time / SCALING_FACTOR;
}

void setup()
{
  pinMode(SR04_ECHO_PIN,INPUT);
  pinMode(SR04_TRIG_PIN,OUTPUT);
  //pinMode(SR04_2_ECHO_PIN,INPUT);
  //pinMode(SR04_2_TRIG_PIN,OUTPUT);
 
  Serial.begin(115200);
  
  aggregate0 = 0;
  aggregate1 = 0;
  aggregateIndex = 0;
  
  digitalWrite(SR04_TRIG_PIN,LOW);
  //digitalWrite(SR04_2_TRIG_PIN,LOW);
  delayMicroseconds(2);
  for (int i = 0; i < FILTER_LENGTH; i++)
  {
    aggregate_array0[i] = measure_SR04_distance(SR04_TRIG_PIN,SR04_ECHO_PIN);
    aggregate_array1[i] = 0;//measure_SR04_distance(SR04_2_TRIG_PIN,SR04_2_ECHO_PIN);
    aggregate0 += aggregate_array0[i];
    aggregate1 += aggregate_array1[i];
    delay(60);
    //Serial.println("once\n");
    //Serial.flush();
  }
  //Serial.println("Setup done\n");
}

void loop() 
{
  long output[2];
  
  long distance0 = measure_SR04_distance(SR04_TRIG_PIN,SR04_ECHO_PIN);
  long distance1 = 0; //measure_SR04_distance(SR04_2_TRIG_PIN,SR04_2_ECHO_PIN);
    
  aggregate_array0[aggregateIndex] = distance0;
  aggregate_array1[aggregateIndex] = distance1;
  
  
  //Serial.print("printing aggregate_array0 \n");
  long sum = 0;
  for(int i = 0; i< 9; i++)
  {
    //Serial.print(aggregate_array0[i]);
    //Serial.print('\t');
    sum += aggregate_array0[i];
  }  
  //Serial.print("\n");  
  
  aggregateIndex = (aggregateIndex + 1) % FILTER_LENGTH;
  /*
  Serial.print("aggregate0 = ");
  Serial.println(aggregate0);
  Serial.print("value to subtract = ");
  Serial.println(aggregate_array0[aggregateIndex]);
  */
  aggregate0 = aggregate0 + distance0 - aggregate_array0[aggregateIndex];
  /*
  Serial.print("new aggregate0 = ");
  Serial.println(aggregate0);
  */
  aggregate1 = aggregate1 + distance1 - aggregate_array1[aggregateIndex];
  output[0] = /*aggregate0*/(sum/FILTER_LENGTH)/58;
  output[1] = (aggregate1/FILTER_LENGTH)/58;
  
  #if 1
  Serial.write((const uint8_t*)output,8);
  #else
  Serial.print("Height: ");
  Serial.print(output[0]/58);
  Serial.print(" Other: ");
  Serial.println(output[1]);
  #endif
  Serial.flush();
  delay(60);
}




