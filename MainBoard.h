
class readController : public scheduler_task { //activate on the XBEE SJone only
   public:
     readController(uint8_t priority) : scheduler_task("readController", 2048, priority) {
     QueueHandle_t q = xQueueCreate(1, quesize); //create queue
     addSharedObject(controller_Q, q); //add my sensor to the queue
   };
   bool run(void *p) {
     TickType_t xLastWakeTime;
     const TickType_t xFrequency = 210;
     xLastWakeTime = xTaskGetTickCount();
     char value[quesize] = {0};
     Uart3& u3 = Uart3::getInstance();
     u3.init(9600); //baud rate
     u3.gets(&value[0],quesize,100);
     puts(value);
     xQueueSend(getSharedObject(controller_Q), &value, portMAX_DELAY); //sends the data to the queue to be processed
     vTaskDelayUntil( &xLastWakeTime, xFrequency/portTICK_PERIOD_MS );
     return true;
   }
};

class processController : public scheduler_task //activate on the XBEE controller only
{
  public:
      processController(uint8_t priority) : scheduler_task("processController", 2048, priority) { }
      bool run(void *p) {
          char valuebuffer[quesize] = {0};
          QueueHandle_t q_name = getSharedObject(controller_Q); //create the q
          if (xQueueReceive(q_name, &valuebuffer, portMAX_DELAY)) {
             printf("\n");
             printf(valuebuffer);
             printf("\n");

             }
          return true;
      }
};
