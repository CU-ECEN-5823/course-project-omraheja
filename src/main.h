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

/* Bluetooth stack headers */
#include "gatt_db.h"
#include "bg_types.h"
#include "mesh_lib.h"
#include "native_gecko.h"
#include <mesh_sizes.h>
#include <gecko_configuration.h>
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


/* Hardware soft timer handles */
#define TIMER_ID_FACTORY_RESET 0x01
#define TIMER_ID_RESTART 0x02
#define LCD_UPDATE 0x04
#define LOG_UPDATE 0x08
#define TIMER_ID_FRIEND_FIND 0x10
#define LPN2_ALERT 0x20

// external signal flags
#define PB0_FLAG 0x01
#define PB1_FLAG 0x02
#define NOISE_FLAG 0x04

// alert macros
#define PB0_STOP_ALERT          (0x01)        // LEVEL model
#define VIBRATION_ALERT         (0x0A)        // LEVEL model

#if 0
#define LIGHT_CONTROL_ON        (0x1B)        // LEVEL model
#define LIGHT_CONTROL_OFF       (0x2B)        // LEVEL model
#else
#define LIGHT_CONTROL_ON        (0x01)        // LEVEL model
#define LIGHT_CONTROL_OFF       (0x00)        // LEVEL model
#endif

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

#endif
