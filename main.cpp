#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define FIB_ITERATIONS 4

class SharedData {

    public:
        SharedData();

        bool job_done = false;
        mutex_t mutex;
        uint16_t fib_results[FIB_ITERATIONS] = {0};
};


SharedData::SharedData(){
    mutex_init(&mutex);
}


static SharedData shared_data;


uint16_t fibonacci_sequence(uint16_t iter) {

    uint16_t current, previous, sum;
    current = 0;
    previous = 1;
    sum = 0;

    for(uint16_t i = 0; i < iter; i++){
        sum = current + previous;
        previous = current;
        current = sum;
    }

    return sum;
}



void main_core1() {

    mutex_enter_blocking(&shared_data.mutex);

    for(uint16_t i = 0; i < FIB_ITERATIONS; i++){
        shared_data.fib_results[i] = fibonacci_sequence(i);
    }
    while(1)
        tight_loop_contents();

    shared_data.job_done = true;
    mutex_exit(&shared_data.mutex);

    while(1)
        tight_loop_contents();
}



void core0_loop(){
    printf("Core 0 waiting\n");
    sleep_ms(1000);
}

 
int main() {
    stdio_init_all();

    // wait for serial monitor to connect
    while(!stdio_usb_connected()){
        sleep_ms(100);
    }
    sleep_ms(200);

    multicore_launch_core1(main_core1);    

    uint32_t mutex_owner;
    bool finished = false;
    while(!finished){

        // while core1 processes, do other stuff
        if(!mutex_try_enter(&shared_data.mutex, &mutex_owner)){
            core0_loop();
            continue;
        }
 
        // handle printing fib numbers, importantly, core0 owns the mutex here
        
        if(!shared_data.job_done){
            mutex_exit(&shared_data.mutex);
            sleep_ms(1);
            continue;
        }
   
        for(uint16_t i = 0; i < FIB_ITERATIONS; i++){
            printf("%u\n", shared_data.fib_results[i]);
        }
        
        mutex_exit(&shared_data.mutex);
        finished = true;
    }

    while(1)
        tight_loop_contents();

    return 0;
}

