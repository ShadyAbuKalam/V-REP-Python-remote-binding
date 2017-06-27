#ifndef colorsensor 
#define colorsensor    

#define s2  1
#define s3  0 //12
#define colorout  4


void init_color_sensor()
{
	pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(colorout, INPUT);
  
}


unsigned long colorSensor(char color)
{
  unsigned long counters = 0; //gets input from sensor frequency generator
  unsigned long countR = 0; //accumilates reading over 8 times to get average
  unsigned long count = 0; //saves the final average reading

  //gets 8 reading for each color
  for (char i = 0; i < 8; i++) {
    if (color == 1) {
      digitalWrite(s2, LOW);
      digitalWrite(s3, LOW); 
    }
    else if (color == 0) {
      digitalWrite(s2, LOW);
      digitalWrite(s3, HIGH);
    }
    else if (color == 2) {


      digitalWrite(s2, HIGH);
      digitalWrite(s3, HIGH);
    }
    else if(color == 3)
    {
      digitalWrite(s2, HIGH);
      digitalWrite(s3, LOW);
    }

    counters = 0;  //resets counter
    //stops RTOS interrupt
    taskENTER_CRITICAL();
    counters = pulseIn(colorout, LOW);  //reads the pulse width which is inversely proportional to the frequency
    taskEXIT_CRITICAL();
    //so the less the pulse width the more the frequency and the more the intensity

    countR = countR + counters;

  }
  //Serial.print("red=");
  
  
  ///////////////////////////
 // countR = countR >> 7;
  //count = (unsigned char) countR;
//////////////////

  count = countR>>3;
  //Serial.println(countR,DEC);
  //Serial.print("inside color: ");
  //Serial.println(count);
  return count;
}

#endif
