class readGPS : public scheduler_task {
    public:
        readGPS(uint8_t priority) : scheduler_task("fetchGPS", 2048, priority) {
        QueueHandle_t q = xQueueCreate(1, quesize); //create queue
        addSharedObject(GPS_Q, q); //add my sensor to the queue
    };
    bool run(void *p) {
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = 200;
        xLastWakeTime = xTaskGetTickCount();
        char value[quesize] = {0};
        Uart3& u3 = Uart3::getInstance();
        u3.init(57600); //baud rate
        u3.gets(&value[0],quesize,100);
        puts(value);
        xQueueSend(getSharedObject(GPS_Q), &value, portMAX_DELAY); //sends the data to the queue to be processed
        vTaskDelayUntil( &xLastWakeTime, xFrequency/portTICK_PERIOD_MS );
        return true;
    }
};
