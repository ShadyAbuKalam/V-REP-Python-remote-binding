#ifndef ultra 
#define ultra    

#define ultraSonicEcho  A3 //Ultrasound signal pin
#define ultraSoundSignal  A0 // Ultrasound signal pin


unsigned long ultrasonic(char i)
{
  unsigned long ultrasoundValue;
    
    digitalWrite(ultraSonicEcho + i, HIGH); // Turn on pullup resistor
  unsigned long echo = 0;                  //duration of the echo pulse
      while(echo ==0){
        taskENTER_CRITICAL();
      digitalWrite(ultraSoundSignal + i, LOW); // Send low pulse
      delayMicroseconds(2); // Wait for 2 microseconds
      digitalWrite(ultraSoundSignal + i, HIGH); // Send high pulse
      delayMicroseconds(5); // Wait for 5 microseconds
      digitalWrite(ultraSoundSignal + i, LOW); // Holdoff
      // please note that pulseIn has a 1sec timeout, which may
      // not be desirable. Depending on your sensor specs, you
      // can likely bound the time like this -- marcmerlin
      echo = pulseIn(ultraSonicEcho + i, HIGH); //Listen for echo
          taskEXIT_CRITICAL();

                   //enables the scheduler
      //gets the average of last 10 non zero readings
          //ultrasound value
        //xQueueReceive( UltraSonicQ, &( ultrasoundValue ), (TickType_t)0); //discards oldest of the the last 10 readings
        ultrasoundValue = (echo / 58.138); //convert to CM
      }
  if (ultrasoundValue < 300)
    {
        return ultrasoundValue;
    }
  else
	return 0;
}

void init_ultrasonic()
{
	for(char i = 0;i<3;i++)
	{
		pinMode(ultraSoundSignal + i, OUTPUT); //set the signal pin to OUTPUT
		pinMode(ultraSonicEcho + i, INPUT);    //sets the echo pin to INPUT
	}
}


#endif
