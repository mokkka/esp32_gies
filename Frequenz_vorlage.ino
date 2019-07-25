#include <Basecamp.hpp>

Basecamp iot{
  Basecamp::SetupModeWifiEncryption::secured
};

const byte        interruptPin = 23;              // Assign the interrupt pin
const byte        sensorOnPin = 4;                // Assign pin for enable sensor
volatile uint64_t StartValue;                     // First interrupt value
volatile uint64_t PeriodCount;                    // period in counts of 0.000001 of a second  
char              str[21];                        // for printing uint64_t values  
int               stateMinValue;
int               stateMaxValue;
int               threshold;         
String            pubTopic;
String            subTopic;
bool messung = false;
bool restartTimer = false;
int  i=0;
int Freg[100];                           // frequency 
int minValue, maxValue, Freq;

hw_timer_t * timer1 = NULL;                       // pointer to a variable of type hw_timer_t 
hw_timer_t * timer2 = NULL;                       // pointer to a variable of type hw_timer_t 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?
volatile int interruptCounter = 0;
int          totalInterruptCounter = 0;

// Digital Event Interrupt
// Enters on falling edge in this example
//=======================================
void IRAM_ATTR isr_timer1() 
{
  portENTER_CRITICAL_ISR(&mux);
      uint64_t TempVal= timerRead(timer1);         // value of timer at interrupt
      PeriodCount= TempVal - StartValue;          // period count between rising edges in 0.000001 of a second
      StartValue = TempVal;                       // puts latest reading as start for next calculation
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isr_timer2()
{
  portENTER_CRITICAL_ISR(&mux);
   interruptCounter++;
  portEXIT_CRITICAL_ISR(&mux);
}

// Converts unit64_t to char for printing
// Serial.println(uintToStr( num, str ));
//================================================
char * uintToStr( const uint64_t num, char *str )
{
  uint8_t i = 0;
  uint64_t n = num;
  do
    i++;
  while ( n /= 10 );
  
  str[i] = '\0';
  n = num;
 
  do
    str[--i] = ( n % 10 ) + '0';
  while ( n /= 10 );

  return str;
}

void mqttConnected(bool sessionPresent)
{
  Serial.println("MQTT verbunden!");
  subTopic="sensors/"+iot.hostname + "/#";
  pubTopic="sensors/"+iot.hostname + "/status";
  iot.mqtt.subscribe(subTopic.c_str(),2);
  iot.mqtt.publish(pubTopic.c_str(), 1, true, "online");
}


void mqttSubscribed(uint16_t packetId, uint8_t qos)
{
  Serial.println("Abonnement erfolgreich");
}


void mqttMessage(char* topic,char* payload,
AsyncMqttClientMessageProperties properties,
size_t len, size_t index, size_t total)
{
  Serial.println("Neue MQTT-Nachricht:");
  Serial.print("Topic:");
  Serial.println(topic);
  Serial.print("Payload:");
  Serial.println(payload);

  return;
  /*if (String(topic)=="sensors/esp32/minValue"){
      if (payload=="1"){
        minValue=Freq;
        iot.configuration.set(ConfigurationKey::minValue,String(minValue).c_str());
      } else {
        minValue=0;
      }
      iot.configuration.set(ConfigurationKey::minValue,String(minValue).c_str()); 
  }
  if (String(topic)="sensors/esp32/maxValue")
  {
      if (payload=="1"){
        maxValue=Freq;
        stateMaxValue=1;
      } else {
        maxValue=0;
        stateMaxValue=0;
        
      }
      iot.configuration.set(ConfigurationKey::minValue,String(minValue).c_str()); 
  }
  iot.configuration.save();  */
}


void mqttPublished(uint16_t packetId)
{
  Serial.println("MQTT-Nachricht veröffentlicht");
}


 // SetUp
//======================================
void setup() 
{
  pinMode(interruptPin, INPUT);                                            // sets pin high
  pinMode(sensorOnPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), &isr_timer1, RISING); // attaches pin to interrupt on Falling Edge
  timer1 = timerBegin(0, 8, true);                                                 // this returns a pointer to the hw_timer_t global variable
                                                                                  // 0 = first timer
                                                                                  // 80 is prescaler so 80MHZ divided by 80 = 1MHZ signal ie 0.000001 of a second
                                                                                  // true - counts up

  timer2 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer2, &isr_timer2, true);
  timerAlarmWrite(timer2, 1000000, true);
  timerAlarmEnable(timer2);
  
  
 
  iot.begin();
  iot.mqtt.onConnect(mqttConnected);
  iot.mqtt.onSubscribe(mqttSubscribed);
  iot.mqtt.onMessage(mqttMessage);
  iot.mqtt.onPublish(mqttPublished);
  
  if (iot.configuration.get(ConfigurationKey::minValue)==""){
    iot.configuration.set(ConfigurationKey::minValue,"0");
  }
  if (iot.configuration.get(ConfigurationKey::maxValue)==""){
    iot.configuration.set(ConfigurationKey::maxValue,"0");
  }
  if (iot.configuration.get(ConfigurationKey::threshold)==""){
    iot.configuration.set(ConfigurationKey::threshold,"0");
  }
  //iot.configuration.save();

  minValue = atoi(iot.configuration.get(ConfigurationKey::minValue).c_str());
  maxValue = atoi(iot.configuration.get(ConfigurationKey::maxValue).c_str());
  threshold = atoi(iot.configuration.get(ConfigurationKey::threshold).c_str());
  
  if (minValue>0) {
    stateMinValue=1;
  } else {
    stateMinValue=0;
  }
  if (maxValue>0) {
    stateMaxValue=1;
  } else {
    stateMaxValue=0;
  }
  Serial.println("minValue:"+ iot.configuration.get(ConfigurationKey::minValue));
  Serial.println("State MinValue:" + stateMinValue);
  Serial.println("maxValue:"+ iot.configuration.get(ConfigurationKey::maxValue));
  Serial.println("State MaxValue:" + stateMaxValue);
  timerStart(timer2);
}

void loop() {
  /*if (restartTimer==true){
    Serial.println("Ich bin noch da");
    restartTimer=false;
  }*/
  //Serial.println(interruptCounter);
  if (interruptCounter > 0&&messung==false){
    Serial.println("if 1");
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);

    totalInterruptCounter++;
    //Serial.print("TotalInterruptCounter: ");Serial.println(totalInterruptCounter);
  }

  if (totalInterruptCounter == 10 && messung==false){
    Serial.println("if 2");
    timerRestart(timer1);                                                              // starts the timer  
    digitalWrite(sensorOnPin, HIGH);
    i=0;
    messung=true;
  }

  if (messung==true&&i<100){
      portENTER_CRITICAL(&mux);
        Freg[i]=1000000.00/PeriodCount;
      portEXIT_CRITICAL(&mux); 
      Serial.println(i);        
      i++;
  }

  if (messung==true&&i>=100){
    Serial.println("if 4");
    timerStop(timer1);
    Freq=0;
    totalInterruptCounter = 0;
    messung = false;
    digitalWrite(sensorOnPin, LOW);
    for (i=0;i<100;i++){
      Freq = Freq + Freg[i]; 
      Freg[i]=0;
    }
    Freq = Freq / 100;
    Serial.print("Frequenz: ");
    Serial.println(Freq);
    pubTopic="sensors/"+iot.hostname+"/value";
    iot.mqtt.publish(pubTopic.c_str(), 1, true, String(Freq).c_str());
    timerRestart(timer2);
  }
  /*if (stateMinValue==0&&stateMaxValue==0){
  pubTopic="sensors/"+iot.hostname+"/status";
  iot.mqtt.publish(pubTopic.c_str(), 1, true, "Bitte Initialisieren");
  }
  Serial.println("Hallo hier bin ich");
  portENTER_CRITICAL(&mux);
  Freg = 10000000.00/PeriodCount;                       // PeriodCount in 0.000001 of a second
  portEXIT_CRITICAL(&mux);
  
  pubTopic="sensors/"+iot.hostname+"/value";
  iot.mqtt.publish(pubTopic.c_str(), 1, true, "1234");*/
}
