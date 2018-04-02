#include <Arduino.h>
#include "ndebug.h"
#include "rangefinder.h"

volatile uint32_t echo_start = 0;                         
volatile uint32_t echo_end = 0;                           
volatile int32_t sonarDistanceCM = SONAR_OUT_OF_RANGE;
volatile uint8_t echoState = START_MEASURE;
static uint32_t _sonar_timer = 0;
int16_t sonarDistanceMedian = SONAR_OUT_OF_RANGE;


bool sonarUpdate()
{
  uint32_t now = millis();

  if((echoState == START_MEASURE) && (now > _sonar_timer))
  {
    echoState = MEASURING;

    // stop sending the pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); 

    // send 10 microsecond pulse
    digitalWrite(trigPin, HIGH);
    
    // wait 10 microseconds before turning off
    delayMicroseconds(10); 
    
    // stop sending the pulse
    digitalWrite(trigPin, LOW);
  }
  else if(echoState == ECHO)
  {
    echoState = START_MEASURE;
    _sonar_timer = now + 20;
    return true;
  }
  return false;
}


void echo_interrupt()
{
  int32_t distance;
  switch (digitalRead(echoPin))                     // Test to see if the signal is high or low
  {
    case HIGH:                                      // High so must be the start of the echo pulse
      echo_end = 0;                                 // Clear the end time
      echo_start = micros();                        // Save the start time
      break;
      
    case LOW:                                       // Low so must be the end of hte echo pulse
      echo_end = micros();                          // Save the end time
      distance = (echo_end - echo_start) / 59;        // Calculate the pulse duration
      sonarDistanceCM = (distance > HCSR04_MAX_RANGE_CM) ? SONAR_OUT_OF_RANGE : distance;
      echoState = ECHO;
      break;
  }
}


void sonarInit() 
{
  pinMode(trigPin, OUTPUT);                           // Trigger pin set to output
  pinMode(echoPin, INPUT);                            // Echo pin set to input
  digitalWrite(trigPin, LOW);
  attachInterrupt(echo_int, echo_interrupt, CHANGE);  // Attach interrupt to the sensor echo input
}


#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }

int32_t quickMedianFilter5(int32_t * v)
{
    int32_t p[5];
    QMF_COPY(p, v, 5);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[2]); QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]); 
    return p[2];
}


static int32_t applySonarMedianFilter(int32_t newSonarReading)
{
    static int32_t sonarFilterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
  //  static bool medianFilterReady = false;
    int nextSampleIndex;

    //if (newSonarReading > SONAR_OUT_OF_RANGE) // only accept samples that are in range
    {
        nextSampleIndex = (currentFilterSampleIndex + 1);
        if (nextSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            nextSampleIndex = 0;
//            medianFilterReady = true;
        }

        sonarFilterSamples[currentFilterSampleIndex] = newSonarReading;
        currentFilterSampleIndex = nextSampleIndex;
    }
   // if (medianFilterReady)
        return quickMedianFilter5(sonarFilterSamples);
   // else
     //   return newSonarReading;
}


void updateSonarDistance(int32_t distance)
{
  //sonarDistanceMedian = distance;
  
    if (distance > HCSR04_MAX_RANGE_CM)
        distance = SONAR_OUT_OF_RANGE;

    sonarDistanceMedian = (int16_t)applySonarMedianFilter(distance);
}


int16_t sonarReadDistance()
{
 //   return sonarDistanceMedian;
  return sonarDistanceCM;
}



