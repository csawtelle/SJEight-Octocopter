 class readIMU : public scheduler_task { //create task on AHREF board only
    public:
        readIMU(uint8_t priority) : scheduler_task("read_AHREF", 2048, priority) {
        QueueHandle_t q = xQueueCreate(1, quesize); //create queue
        addSharedObject(IMU_Q, q); //add my sensor to the queue
    };
    bool run(void *p) {
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = 20;
        xLastWakeTime = xTaskGetTickCount();
        char value[quesize] = {0};
        Uart3& u3 = Uart3::getInstance();
        u3.init(115200); //baud rate
        u3.putline("#s<12>");
        u3.gets(&value[0],quesize,100);
        xQueueSend(getSharedObject(IMU_Q), &value, portMAX_DELAY); //sends the data to the queue to be processed
        vTaskDelayUntil( &xLastWakeTime, xFrequency/portTICK_PERIOD_MS );
        return true;
    }
}
class calculateIMU : public scheduler_task //create a task on the AHREF board only
{
  public:
      calculateIMU(uint8_t priority) : scheduler_task("calculate_heading", 2048, priority) { }
      bool run(void *p) {
          char valuebuffer[quesize] = {0};
          char * tokens[3];
          uint8_t synch_counter = 0;
          uint8_t buffer_index = 0;
          float yaw=0;
          float pitch=0;
          float roll=0;
          size_t maxTokens = 0;
          const char hashtag = '#';
          QueueHandle_t q_name = getSharedObject(IMU_Q); //create the q
          if (xQueueReceive(q_name, &valuebuffer, portMAX_DELAY)) {
             printf("\n");
             
             char *tokens[4];
             char *p;
             p = strtok(valuebuffer,",");
             int j = 0;

             while (p) {
                 tokens[j++] = p;
                 p = strtok(NULL, ",");
             }
             yaw = str::toFloat(tokens[1]);
             pitch = str::toFloat(tokens[2]);
             roll = str::toFloat(tokens[3]);

             printf("\n");
             printf("\nYAW:%.3f",yaw);
             printf("     ");
             printf("PITCH:%.3f",pitch);
             printf("     ");
             printf("ROLL:%.3f",roll);
          }
          return true;
      }
   
};
