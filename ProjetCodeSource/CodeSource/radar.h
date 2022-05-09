#ifndef RADAR_H_
#define RADAR_H_

#include <sensors/VL53L0X/VL53L0X.h>

enum state {Detect, Chase};

void radar_start(void);
enum state get_radar_state(void);
void radar_measure_speed(void);




#endif /* RADAR_H_ */
