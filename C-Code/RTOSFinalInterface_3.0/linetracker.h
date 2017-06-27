#ifndef line 
#define line    
#include "macros.h"

//#define LineDetectorPin_1 2 // left
//#define LineDetectorPin_2 3 // right


long LineDetectorCounter_1 = 0 ;
long LineDetectorCounter_2 = 0 ;


void LineDetectorFunction_1()
{
	LineDetectorCounter_1++;
}
void LineDetectorFunction_2()
{
	LineDetectorCounter_2++;
}

 void init_line_tracker()
 {
	pinMode( LineDetectorPin_1 , INPUT );
	pinMode( LineDetectorPin_2 , INPUT );
	pinMode(LineDetectorPin_1 , INPUT_PULLUP);
	pinMode(LineDetectorPin_2 , INPUT_PULLUP);
	attachInterrupt ( digitalPinToInterrupt(LineDetectorPin_1) , LineDetectorFunction_1 , FALLING);
	attachInterrupt ( digitalPinToInterrupt(LineDetectorPin_2) , LineDetectorFunction_2 , FALLING);

 }
 
 #endif
