/*@Filename	 : main.h
 *@Author	 : Om Raheja
 *@Course	 : IoT Embedded Firmware [Spring 2019]
 *@Project   : MINE SAFETY USING BLUETOOTH MESH
 *@References: Bluetooth Mesh sample example codes [Light and Switch] have been used as references.
 * */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

/* Includes */
#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include "display.h"
#include "em_core.h"
#include "gpio.h"

/* Scheduler headers */
#include "emsleep.h"
#include "delay_us.h"
#include "letimer.h"
#include "scheduler.h"

/* Bluetooth stack headers */
#include "gatt_db.h"
#include "bg_types.h"
#include "mesh_lib.h"
#include "native_gecko.h"
#include <mesh_sizes.h>
#include <gecko_configuration.h>
#include "I2C.h"
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"

// display header files
#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

// active Bluetooth connections
static uint8 num_connections = 0;

// log timestamp
uint32_t msecCount;

static uint16 _elem_index = 0x00;

#if 0
#define ONE_SECOND_TICK_COUNT			(32768)
#endif


/* Flash IDs for Flash Store and Load functions */
#define ALERT_FLASH_ID          		(0x01)
#define DISPLAY_FLASH_ID        		(0x02)
#define LIGHTS_FLASH_ID         		(0x03)

/* Flash Save Keys */
#define ALERT_FLASH_ADDR       			(0x4001)
#define DISPLAY_FLASH_ADDR  			(0x4002)
#define LIGHTS_FLASH_ADDR	    		(0x4003)

/* Persistent data lengths */
#define ALERT_DATA_LEN 			        (1)
#define DISPLAY_DATA_LEN        		(15)
#define LIGHTS_DATA_LEN        			(1)


/* Hardware soft timer handles */
#define TIMER_ID_FACTORY_RESET 0x01
#define TIMER_ID_RESTART 0x02
#define LCD_UPDATE 0x04
#define LOG_UPDATE 0x08
#define TIMER_ID_FRIEND_FIND 0x10
#define LPN2_ALERT 0x20

// external signal flags
#define PB0_FLAG 			0x01
#define PB1_FLAG 			0x02
#define NOISE_FLAG 			0x04
#define HUMIDITY_FLAG 		0x08

// alert macros
#define PB0_STOP_ALERT          (0x01)        // LEVEL model
#define VIBRATION_ALERT         (0x0A)        // LEVEL model
#define LIGHT_CONTROL_ON        (0x01)        // ON OFF model
#define LIGHT_CONTROL_OFF       (0x00)        // ON OFF model

#define GAS_ALERT               (0x0C)        // LEVEL model
#define FIRE_ALERT              (0x0D)        // LEVEL model
#define NOISE_ALERT             (0x0E)        // LEVEL model
#define HUMIDITY_ALERT          (0x0F)        // LEVEL model


/* Function Prototypes */
void set_device_name(bd_addr *pAddr);

static void init_models(void);

static void on_off_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms);

static void on_off_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags);

static void level_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags);

static void level_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms);

/* Persistent data function declarations */
uint8_t* flashLoad(uint8_t flashID);
void flashSave(uint8_t flashID, uint8_t *dataPtr);
uint8_t* stringToUint(char* str);
char* uintToString(uint8_t* buffer);

#endif
