  class readController : public scheduler_task {
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

 class processController : public scheduler_task
 {
     public:
         processController(uint8_t priority) : scheduler_task("processController", 2048, priority) { }
         bool run(void *p) {
             char valuebuffer[quesize] = {0};
             QueueHandle_t q_name = getSharedObject(controller_Q); //create the q
             if (xQueueReceive(q_name, &valuebuffer, portMAX_DELAY)) {
                printf("\n");
                //printf(valuebuffer);
                //printf("\n");
                char *tokens[4];
                char *p;
                p=strtok(valuebuffer,":");
                int j =0;
                
                while(p){
                  token[j++] = p;
                  p = strtok(NULL, ":");
                }
                //stick ranges from 0-1024, so divide by 10 to get roughly 0-100%
               float throttle = (str::toFloat(tokens[1]))/10; 
               float nada = (str::toFloat(tokens[2]))/10;
               float pitch = (str::toFloat(tokens[3]))/10;
               float roll = (str::toFloat(tokens[4]))/10;
               
               if(throttle< 65 | throttle >45){ //because sticks don't sit exactly at 500
                 throttle = 50;
               }
                if (throttle >= 85)
                throttle = 85;
                
                if(pitch< 65 | pitch >45){ //because sticks don't sit exactly at 500
                 pitch = 0; zero percent pitch
               }
                
                if (pitch > 65){
                  pitch = 2;
                }
                
                if (pitch < 45){
                 pitch = -2;
                }
                  if(roll< 65 | roll >45){ //because sticks don't sit exactly at 500
                 roll = 0; //zero percent roll 
               }
               
               if(roll > 65){
                 roll = 2;
               }
               
               if(roll < 45){
                 roll = -2;
               }
               
                }
             return true;
         }
 };
 
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
 class calculateGPS : public scheduler_task
 {
     public:
         calculateGPS(uint8_t priority) : scheduler_task("parseGPS", 2048, priority) { }
         bool run(void *p) {
             char valuebuffer[quesize] = {0};
             QueueHandle_t q_name = getSharedObject(GPS_Q); //create the q
             if (xQueueReceive(q_name, &valuebuffer, portMAX_DELAY)) {
                printf("\n");
                puts(valuebuffer);
                printf("\n");
                //TODO finish the parsing
                }
             return true;
         }
 };
void GPS_init(void) {
    char init_response[quesize] = {0};
    Uart3& u3 = Uart3::getInstance();
    u3.init(9600); //baud rate
    const char baud_57600[] = "$PMTK251,57600*2C";
    const char output_RMCGGA[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\n";
    const char update_1hz[]= "$PMTK220,1000*1F\n";
    const char update_5hz[]=  "$PMTK220,200*2C";
    const char antenna[] = "$PGCMD,33,1*6C\n";
    const char no_antenna[] =  "$PGCMD,33,0*6D";
    //request release and version number
    //const char PMTK_Q_RELEASE[] = "PMTK605*31\n";
 
    //Set the output for RMC and GGA
    u3.putline(output_RMCGGA);
    u3.gets(&init_response[0],quesize,1000);
    puts(init_response);
    puts("\n");
 
    //Set the output rate 1Hz
    init_response[quesize] = {0};
    u3.putline(update_5hz);
    u3.gets(&init_response[0],quesize,1000);
    puts(init_response);
    puts("\n");
 
    //Set the antenna type
    init_response[quesize] = {0};
    u3.putline(no_antenna);
    u3.gets(&init_response[0],quesize,1000);
    puts(init_response);
    puts("\n");
 
    //Set the output for RMC and GGA
    u3.putline(baud_57600);
    u3.gets(&init_response[0],quesize,1000);
    puts(init_response);
    puts("\n");
 
    //Respond with init finish
    puts("GPS Init complete");
    puts("\n");
 
}
