#include "Arduino_BMI270_BMM150.h"

#define SAMPLING_RATE 100 // in Hz
#define TUNE 0.7 // for accelerometer

class SensorFusion {
    public:
        bool checkAvailability();
        void convert_accel(void);   
        void convert_gyro(void);
        float comp_filt_roll(float, float, float);
        float comp_filt_pitch(float, float, float);
        float get_roll();
        float get_pitch();

    private:
        float x, y, z;
        float Rg = 0;
        float Pg = 0;
        float Ra, Pa;
        float roll, pitch, yaw;
        float new_roll, new_pitch;
        bool roll_f = 0;
        bool pitch_f = 0;
};