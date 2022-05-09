/*
* @name		radar.h
* @author 	Bassam El Rawas, Ali Elmorsy (Groupe 15)
* @date 	Mai 2022
*/

#ifndef RADAR_H
#define RADAR_H

#include <sensors/VL53L0X/VL53L0X.h>

enum State {Detect, Chase};

void radar_start(void);
enum State get_radar_state(void);
void radar_measure_speed(void);

#endif /* RADAR_H */
