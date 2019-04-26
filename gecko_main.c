/*@Filename	 : main.c
 *@Author	 : Om Raheja (Code developed with group members)
 *@Course	 : IoT Embedded Firmware [Spring 2019]
 *@Project   : MINE SAFETY USING BLUETOOTH MESH
 *@brief	 : This function handles gecko events and contains the main portion of the code
 *@References: Bluetooth Mesh sample example codes [Light and Switch] have been used as references.
 * */

/* Includes */
#include "init_mcu.h"
#include "init_board.h"
#include "board_features.h"
#include "init_app.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "ble-configuration.h"
#include "bg_types.h"
#include <mesh_sizes.h>
#include <gecko_configuration.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

/* Header files required for the main code */
#include "src/main.h"
#include "src/gpio.h"
#include "src/display.h"
#include "src/log.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#include "src/ble_mesh_device_type.h"

/* Bluetooth stack heap */
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .max_timers = 16,
};

// for alerts
uint8_t toggleCount = 0;

/* Function Prototypes */
void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
void mesh_native_bgapi_init(void);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

// set device name
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address
  sprintf(name, "5823LPN2 %02x:%02x", pAddr->addr[1], pAddr->addr[0]);
  displayPrintf(DISPLAY_ROW_NAME, "%s", name);
  displayPrintf(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x", pAddr-> addr[0], pAddr-> addr[1], pAddr-> addr[2], pAddr-> addr[3], pAddr-> addr[4], pAddr-> addr[5]);

  // write device name to the GATT database
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
}

static void init_models(void)
{
  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                           0,
                                           on_off_request,
                                           on_off_change);
  mesh_lib_generic_server_register_handler(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
                                           0,
                                           on_off_request,
                                           on_off_change);
}

// Level model request from friend
static void level_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	LOG_INFO("IN LEVEL REQUEST");

	if(request->level == FIRE_ALERT) {
		displayPrintf(DISPLAY_ROW_SENSOR, "FIRE ALERT");
		toggleCount = 0;
		gecko_cmd_hardware_set_soft_timer(3277, LPN2_ALERT, 0);
	}
	if(request->level == GAS_ALERT) {
		displayPrintf(DISPLAY_ROW_SENSOR, "GAS ALERT");
		toggleCount = 0;
		gecko_cmd_hardware_set_soft_timer(3277, LPN2_ALERT, 0);
	}
	if(request->level == VIBRATION_ALERT) {
		displayPrintf(DISPLAY_ROW_SENSOR, "EARTHQUAKE");
		toggleCount = 0;
		gecko_cmd_hardware_set_soft_timer(3277, LPN2_ALERT, 0);
	}
	if(request->level == PB0_STOP_ALERT) {
		toggleCount = 101;
	}
}

static void level_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
	LOG_INFO("IN LEVEL CHANGED");
}

// On off request from friend
static void on_off_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	LOG_INFO("Got onoff request, data = %d", request->on_off);

	if(request->on_off == LIGHT_CONTROL_ON) {
		gpioLed0SetOn();
	}

	if(request->on_off == LIGHT_CONTROL_OFF) {
		gpioLed0SetOff();
	}
}

static void on_off_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{

}

/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 ******************************************************************************/
void lpn_init(void)
{
	uint16 res;
	// Initialize LPN functionality.
	res = gecko_cmd_mesh_lpn_init()->result;
	if (res) {
		LOG_INFO("LPN init failed (0x%x)", res);
		return;
	}

	// Configure the lpn with following parameters:
	// - Minimum friend queue length = 2
	// - Poll timeout = 1 seconds
	res = gecko_cmd_mesh_lpn_configure(2, 1 * 1000)->result;
	if (res) {
		LOG_INFO("LPN conf failed (0x%x)", res);
		return;
	}

	LOG_INFO("trying to find friend...");
	res = gecko_cmd_mesh_lpn_establish_friendship(0)->result;

	if (res != 0) {
		LOG_INFO("ret.code %x", res);
	}
}

/**
 * See light switch app.c file definition
 */
void gecko_bgapi_classes_init_server_lpn(void)
{
	gecko_bgapi_class_dfu_init();
	gecko_bgapi_class_system_init();
	gecko_bgapi_class_le_gap_init();
	gecko_bgapi_class_le_connection_init();
	gecko_bgapi_class_gatt_server_init();
	gecko_bgapi_class_hardware_init();
	gecko_bgapi_class_flash_init();
	gecko_bgapi_class_test_init();
	gecko_bgapi_class_mesh_node_init();
	gecko_bgapi_class_mesh_proxy_init();
	gecko_bgapi_class_mesh_proxy_server_init();
	gecko_bgapi_class_mesh_generic_server_init();
	gecko_bgapi_class_mesh_lpn_init();
}

void gecko_main_init()
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Minimize advertisement latency by allowing the advertiser to always
  // interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_stack_init(&config);

  // Server initialization which is LPN
  gecko_bgapi_classes_init_server_lpn();

  // Initialize coexistence interface. Parameters are taken from HAL config.
  gecko_initCoexHAL();
}

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  switch (evt_id) {
    case gecko_evt_system_boot_id:
    	// for factory reset
		if (GPIO_PinInGet(__PB0_BUTTON_PORT, __PB0_BUTTON_PIN) == 0 || GPIO_PinInGet(__PB1_BUTTON_PORT, __PB1_BUTTON_PIN) == 0) {
			LOG_INFO("factory reset");
			displayPrintf(DISPLAY_ROW_ACTION, ">>>FACTORY RESET<<<");

			/* perform a factory reset by erasing PS storage. This removes all the keys and other settings
			that have been configured for this node */
			gecko_cmd_flash_ps_erase_all();
			// reboot after a small delay
			gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
		} else {
			struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
			set_device_name(&pAddr->address);

			// Initialize Mesh stack in Node operation mode, wait for initialized event
			gecko_cmd_mesh_node_init();
			LOG_INFO("BOOT DONE");
		}
      break;

    case gecko_evt_mesh_node_initialized_id:
    	;
		struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);
		if (pData->provisioned) {
			LOG_INFO("node is provisioned. address:%x, ivi:%ld", pData->address, pData->ivi);

			mesh_lib_init(malloc,free,9);
			init_models();
			gecko_cmd_mesh_generic_server_init();
			lpn_init();
			gpioIntEnable();
			displayPrintf(DISPLAY_ROW_ACTION, "PROVISIONED");
		}
		else {

			LOG_INFO("node is unprovisioned");
			LOG_INFO("starting unprovisioned beaconing...");

			// start beaconing
			gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
			displayPrintf(DISPLAY_ROW_ACTION, "UNPROVISIONED");
		}
		break;

    case gecko_evt_mesh_node_provisioning_started_id:
    	displayPrintf(DISPLAY_ROW_ACTION, "PROVISIONING");
    	LOG_INFO("Provisiong Started");
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		displayPrintf(DISPLAY_ROW_ACTION, "Provision Failed");
		LOG_INFO("provisioning failed, code %x", evt->data.evt_mesh_node_provisioning_failed.result);
		// reset
		gecko_cmd_hardware_set_soft_timer(2*32768, TIMER_ID_RESTART, 1);
		break;

	case gecko_evt_mesh_node_provisioned_id:
		displayPrintf(DISPLAY_ROW_ACTION, "PROVISIONED");
		LOG_INFO("node is provisioned. address:%x, ivi:%ld", pData->address, pData->ivi);

		mesh_lib_init(malloc,free,9);
		init_models();
		gecko_cmd_mesh_generic_server_init();
		lpn_init();
		gpioIntEnable();
		break;

    case gecko_evt_hardware_soft_timer_id:
    	switch (evt->data.evt_hardware_soft_timer.handle) {
			case TIMER_ID_FRIEND_FIND:
			{
				LOG_INFO("trying to find friend...");
				uint16_t result;
				result = gecko_cmd_mesh_lpn_establish_friendship(0)->result;
				if (result != 0) {
					LOG_INFO("ret.code %x", result);
				}
			}
			break;
    		case TIMER_ID_RESTART:
				gecko_cmd_system_reset(0);
				break;
			case TIMER_ID_FACTORY_RESET:
				gecko_cmd_system_reset(0);
				break;
			case LOG_UPDATE:
				msecCount += 10;
				break;
			case LCD_UPDATE:
				displayUpdate();
				break;
			case LPN2_ALERT:
				if(toggleCount % 2) {
					toggleCount++;
					GPIO_PinOutSet(BUZZ_PORT, BUZZ_PIN);
					gpioLed1SetOn();
				}
				else {
					toggleCount++;
					GPIO_PinOutClear(BUZZ_PORT, BUZZ_PIN);
					gpioLed1SetOff();

					// stop alerts after 10 seconds
					if(toggleCount > 100) {
						toggleCount = 0;
						gecko_cmd_hardware_set_soft_timer(0, LPN2_ALERT, 0);
						displayPrintf(DISPLAY_ROW_SENSOR, " ");
						displayPrintf(DISPLAY_ROW_ACTUATOR, " ");
					}
				}
				break;
    	}
    	break;

	case gecko_evt_system_external_signal_id:
		;
		struct mesh_generic_state current;
		struct mesh_generic_state target;
		uint16_t resp;

		current.kind = mesh_generic_state_level;
		target.kind = mesh_generic_state_level;

		// PB0 button press
		if (((evt->data.evt_system_external_signal.extsignals) & PB0_FLAG) != 0) {
			LOG_INFO("PB0 EXTERNAL SIGNAL");

			// stop alert
			toggleCount = 101;

			// server publish alert stop data
			current.level.level = PB0_STOP_ALERT;
			target.level.level = PB0_STOP_ALERT;

			// server update
			resp = mesh_lib_generic_server_update(MESH_GENERIC_LEVEL_SERVER_MODEL_ID, 0, &current, &target, 0);
			if (resp) {
				LOG_INFO("gecko_cmd_mesh_generic_server_update failed, code %x", resp);
			} else {
				LOG_INFO("update done");
			}

			// publish update
			resp = mesh_lib_generic_server_publish(MESH_GENERIC_LEVEL_SERVER_MODEL_ID, 0, current.kind);
			if (resp) {
				LOG_INFO("gecko_cmd_mesh_generic_server_publish failed, code %x", resp);
			} else {
				LOG_INFO("update done");
			}
		}

		// Noise sensor event
		if (((evt->data.evt_system_external_signal.extsignals) & NOISE_FLAG) != 0) {
			LOG_INFO("NOISE EXTERNAL SIGNAL");

			displayPrintf(DISPLAY_ROW_SENSOR, "NOISE ALERT");

			// start alerts
			toggleCount = 0;
			gecko_cmd_hardware_set_soft_timer(3277, LPN2_ALERT, 0);

			// server publish noise alert
			current.level.level = NOISE_ALERT;
			target.level.level = NOISE_ALERT;

			// server update
			resp = mesh_lib_generic_server_update(MESH_GENERIC_LEVEL_SERVER_MODEL_ID, 0, &current, &target, 0);
			if (resp) {
				LOG_INFO("gecko_cmd_mesh_generic_server_update failed, code %x", resp);
			} else {
				LOG_INFO("update done");
			}

			// publish update
			resp = mesh_lib_generic_server_publish(MESH_GENERIC_LEVEL_SERVER_MODEL_ID, 0, current.kind);
			if (resp) {
				LOG_INFO("gecko_cmd_mesh_generic_server_publish failed, code %x", resp);
			} else {
				LOG_INFO("update done");
			}
		}
		break;

	case gecko_evt_le_connection_opened_id:
		LOG_INFO("In connection opened id");
		num_connections++;
		displayPrintf(DISPLAY_ROW_CONNECTION, "Connected");
		gecko_cmd_mesh_lpn_deinit();
		displayPrintf(DISPLAY_ROW_LPN, "LPN off");
		break;

    case gecko_evt_le_connection_closed_id:
    	LOG_INFO("In connection closed id");

		/* Check if need to boot to dfu mode */
		if (boot_to_dfu) {
			/* Enter to DFU OTA mode */
			gecko_cmd_system_reset(2);
		}
		if (num_connections > 0) {
			if (--num_connections == 0) {
				displayPrintf(DISPLAY_ROW_CONNECTION, " ");
				lpn_init();
			}
		}
		break;

    case gecko_evt_mesh_lpn_friendship_established_id:
    	LOG_INFO("friendship established");
    	displayPrintf(DISPLAY_ROW_LPN, "LPN");
    	break;

    case gecko_evt_mesh_lpn_friendship_failed_id:
    	LOG_INFO("friendship failed");
    	displayPrintf(DISPLAY_ROW_LPN, "no friend");
    	// try again in 2 seconds
    	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FRIEND_FIND, 1);
    	break;

    case gecko_evt_mesh_lpn_friendship_terminated_id:
    	LOG_INFO("friendship terminated");
    	displayPrintf(DISPLAY_ROW_LPN, "friend lost");
    	if (num_connections == 0) {
    		// try again in 2 seconds
    		gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FRIEND_FIND, 1);
    	}
    	break;

    case gecko_evt_gatt_server_user_write_request_id:
    	if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
    		/* Set flag to enter to OTA mode */
        boot_to_dfu = 1;
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(
          evt->data.evt_gatt_server_user_write_request.connection,
          gattdb_ota_control,
          bg_err_success);

        /* Close connection to enter to DFU OTA mode */
        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
      }
      break;

    case gecko_evt_mesh_node_reset_id:
    	gecko_cmd_flash_ps_erase_all();
    	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
    	break;

    case gecko_evt_mesh_generic_server_client_request_id:
    	mesh_lib_generic_server_event_handler(evt);
    	LOG_INFO("In server client request id");
    	break;

    case gecko_evt_mesh_generic_server_state_changed_id:
    	mesh_lib_generic_server_event_handler(evt);
    	LOG_INFO("In server state changed id");
    	break;

    default:
      break;
  }
}
