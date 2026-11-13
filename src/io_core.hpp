#ifndef IO_CORE_H__
#define IO_CORE_H__

#define NUM_LIDAR_SWEEPS 5
#define LIDAR_ANGLE_RANGE 180

// using alias to make changing the type easier if nessesary
typedef uint8_t lidar_data_t;

class IO_Manager {
    public:
        
    private:
        lidar_data_t data_buf[NUM_LIDAR_SWEEPS][LIDAR_ANGLE_RANGE];
};


#endif