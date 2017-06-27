#include "macros.h"

/* Header part of the frame */
struct HEADER {
  char role:8;
  unsigned int ID:16;
  byte state:8;
};

/* Payload part of the frame */
struct PAYLOAD {
  byte len:8;
  byte type:8;
  char data[26];
};

/* The full frame to be sent accross the network */
struct FRAME {
  HEADER header;
  PAYLOAD payload;
};

FRAME ownFrame, recvFrame;

#if(VERBOSE == 1)
void printFrame(struct FRAME * frame) {
  Serial.print("Role:       "); Serial.println(frame->header.role);
  Serial.print("ID:         "); Serial.println(frame->header.ID);
  Serial.print("State:      "); Serial.println(frame->header.state);
  Serial.print("Length:     "); Serial.println(frame->payload.len);
  Serial.print("Type:       "); Serial.println(frame->payload.type);
  Serial.print("Payload:    "); Serial.println(frame->payload.data);
}
#endif
/*================================= GLOBALS =================================*/
/* Pinout for the radio (CE - CSN)*/
#if (MEGA == 1)
  RF24 radio(53,48);
#else
  RF24 radio(7,8);
#endif

/* Addresses of pipes (reading and writing pipes) */
byte addresses[][6] = {"1Node","2Node"};

/* Used to control whether this node is sending or receiving (Testing only) 
 * 0 => Receiving 
 * 1 => Sending
 */
   
bool role = 1;

unsigned char RF_EVENT = 0;
/* RTOS Task handles */
TaskHandle_t xSendFrame, xRecvFrame, xMainTask;


bool sendFrame() {
   /* Stop listening to send */
   radio.stopListening();

   /* Send the frame */
   if (!radio.write(&ownFrame, sizeof(ownFrame))) {
      /* if failed to send data */
      #if(VERBOSE == 1) 
        Serial.println(F("Failed to send frame"));
      #endif
      return false;
   }
   #if(VERBOSE == 1) 
    Serial.println(F("frame sent"));
   #endif
   return true;
}

void vSendFrame(void * param) {
  #if(VERBOSE == 1)
    Serial.println("Send Task");
  #endif
  
  /* if role is receiving => suspend task */
  if(role == 0) vTaskSuspend(NULL);
  if(RF_EVENT == 0) return;

  for (;;) {
    /* Send ownFrame and delay with the specified system ticks defined in
     * RFSENDPERIOD macro.
     */
    sendFrame();
    RF_EVENT = 0;
    vTaskDelay(RFSENDPERIOD);
  }
}



void vRecvFrame(void * param) {
  #if(VERBOSE == 1)
    Serial.println("Recv Task");
  #endif
  /* if role is sending => suspend task */
  if(role == 1) vTaskSuspend(NULL);

  /* TickType for periodic delaying */
  TickType_t xLastWakeTime;

  /* Initialise the xLastWakeTime variable with the current time. */
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    /* Start listening for data */
    radio.startListening();

    /* Data is found */
    if(radio.available()) {
      // While there is data ready
      while (radio.available()) {                        
        // Get the frame
        radio.read(&recvFrame, sizeof(recvFrame));             
      }
      
      #if (VERBOSE == 1)
        Serial.println(F("Received: "));
        // Print frame details
        printFrame(&recvFrame);
      #endif
    }

    /* delay with the specified system ticks defined in RFRECVPERIOD macro */
    vTaskDelayUntil(&xLastWakeTime, RFRECVPERIOD);
  }
}



void init_rf()
{
	radio.begin();

  /* Set the PA Level low to prevent power supply related issues since 
  a seperate 3.3V supply is needed for stability */
  radio.setPALevel(RF24_PA_LOW);

  /* Required for Multicast */
  radio.enableDynamicPayloads();
  radio.enableDynamicAck();
  
  /* Default behavior for a node is receiving */
  /* Open a writing and reading pipe -> defaults for receiving */
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  
  /* Start the radio listening for data */
  radio.startListening();
  
  ownFrame.header.role = 'B';
  ownFrame.header.ID = 0;
  ownFrame.header.state = 100;
  ownFrame.payload.type = 100;
  /*RTOS creat send task*/
  xTaskCreate(vSendFrame, "RFSEND", TASKSTACKSZIE, NULL, 5, &xSendFrame);
  
  /*RTOS creat recieve task*/
  xTaskCreate(vRecvFrame, "RFRECV", TASKSTACKSZIE, NULL, 5, &xRecvFrame);

}

