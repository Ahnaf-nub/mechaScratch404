#include <NewPing.h>

#define MAX_DISTANCE 120
NewPing sonars[] = {
  NewPing(34, 34, MAX_DISTANCE),
  NewPing(35, 35, MAX_DISTANCE),
  NewPing(32, 32, MAX_DISTANCE),
  NewPing(33, 33, MAX_DISTANCE),
  NewPing(25, 25, MAX_DISTANCE)
};

long dists[] = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
}

void loop() {

  long sonarTimer = millis();
  
  Serial.print("Sonar Values: ");
  for(int i = 0; i < 5; i++){
    dists[i] = sonars[i].ping_cm();
    Serial.print(dists[i]);
    Serial.print(" - ");
  }
  Serial.println(" | ");
  
  while(millis() - sonarTimer < 30){ } // ensures a 30ms delay between pings per sonar
}
