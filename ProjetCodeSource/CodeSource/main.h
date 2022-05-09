/*
* @name		main.h
* @author 	Bassam El Rawas, Ali Elmorsy (Groupe 15)
* @date 	Mai 2022
*/

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "camera/dcmi_camera.h"  // why do we need this
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

uint16_t set_ref(void);
/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
