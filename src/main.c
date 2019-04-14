/*@Filename	 : main.c
 *@Date		 : 3rd April 2019
 *@Author	 : Om Raheja
 *@Course	 : IoT Embedded Firmware [Spring 2019]
 *@References: Bluetooth Mesh sample example codes [Light and Switch] have been used as references.
 * */

/*************************************************************
 * STANDARD LIBRARY HEADER FILES							 *
 *************************************************************/
#include <stdbool.h>
#include "native_gecko.h"
#include "log.h"
#include "display.h"
#include "gpio.h"
#include "gatt_db.h"
#include "mesh_generic_model_capi_types.h"
#include "ble_mesh_device_type.h"
#include <gpiointerrupt.h>
#include "mesh_lib.h"
#include "em_cmu.h"
#include "em_core.h"

/*************************************************************
 * FUNCTION PROTOTYPES										 *
 *************************************************************/
void gpioint(uint8_t pin);
static void init_models(void);
extern void gecko_main_init();
void initiate_factory_reset(void);
void set_device_name(bd_addr *pAddr);
void enable_push_button_interrupts();
void handle_button_press(int button);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
//static errorcode_t onoff_update(uint16_t element_index);
//static errorcode_t onoff_update_and_publish(uint16_t element_index);
void gecko_event_handler(uint32_t evt_id, struct gecko_cmd_packet *evt);
void gecko_event_handler_pub(uint32_t evt_id, struct gecko_cmd_packet *evt);
void gecko_event_handler_sub(uint32_t evt_id, struct gecko_cmd_packet *evt);
extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
//static void onoff_change(uint16_t model_id, uint16_t element_index, const struct mesh_generic_state *current, const struct mesh_generic_state *target,\
//		                 uint32_t remaining_ms);
//static void onoff_request(uint16_t model_id, uint16_t element_index, uint16_t client_addr, uint16_t server_addr, uint16_t appkey_index,\
//						  const struct mesh_generic_request *request, uint32_t transition_ms, uint16_t delay_ms, uint8_t request_flags);

static void pb0_pressrelease_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags);

static void pb0_pressrelease_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms);


/*************************************************************
 * GLOBAL VARIABLES										     *
 *************************************************************/

/* transaction identifier */
static uint8 trid = 0;

/* current position of the switch */
volatile uint8 switch_pos = 0;

/* number of on/off requests to be sent */
static uint8 request_count;

/* For indexing elements of the node (this example has only one element) */
static uint16 _elem_index = 0xffff;

static uint16 _primary_elem_index = 0xffff;


/*************************************************************
 * MACROS													 *
 *************************************************************/
#define TIMER_ID_RESTART          	78
#define TIMER_ID_FACTORY_RESET    	77
#define TIMER_ID_PROVISIONING     	66
#define TIMER_ID_RETRANS          	10
#define TIMER_ID_FRIEND_FIND      	20
#define TIMER_ID_NODE_CONFIGURED  	30
#define PB0_PRESSED					0x00
#define PB0_RELEASED				0x01
#define MESH_GENERIC_ON_OFF_SERVER_MODEL_ID		0x1000	/** Generic on/off server */
#define MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID		0x1001	/** Generic on/off client */
#define MESH_GENERIC_ON_OFF_STATE_OFF			0x00	/** Generic on/off state value off */
#define MESH_GENERIC_ON_OFF_STATE_ON			0x01	/** Generic on/off state value on */

#define IR_SENSOR_PORT			gpioPortD
#define IR_SENSOR_PIN			(11)			//Pin P9
/* gpioPortD, Pin 10 -> Pin P7 */

int IR_FLAG;

/********************************************************************************************
 * @function_name : INITIATE FACTORY RESET					 								*
 * @brief		  : This function is called to initiate factory reset. Factory reset may be	*
 * 					initiated by keeping one of the WSTK pushbuttons pressed during reboot.	*
 * 					Factory reset is also performed if it is requested by the provisioner	*
 * 					(event gecko_evt_mesh_node_reset_id).									*
 * @Reference	  : Silicon Lab's Bluetooth Mesh Switch Example								*
 ********************************************************************************************/
void initiate_factory_reset(void)
{
  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();

  /* reboot after a small delay */
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);

  LOG_INFO("[INFO] :\tFactory Reset\n\r");
  displayPrintf(DISPLAY_ROW_ACTION,"Factory Reset!");

}


/***************************************************************************//**
 * Initialize LPN functionality with configuration and friendship establishment.
 * Copied from silicon labs switch example code
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
	// - Poll timeout = 5 seconds
	res = gecko_cmd_mesh_lpn_configure(2, 5 * 1000)->result;
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

/************************************************************************************************
 * @function_name : SET DEVICE NAME							 									*
 * @brief		  : Set device name in the GATT database. A unique name is generated using		*
 * 					the two last bytes from the Bluetooth address of this device. Name is also	*
 * 					displayed on the LCD.														*
 * @param[in] pAddr  Pointer to Bluetooth address.												*
 * @Reference	  : Silicon Lab's Bluetooth Mesh Switch Example									*
 ********************************************************************************************/
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  /* create unique device name using the last two bytes of the Bluetooth address */
  sprintf(name, "5823PUB %02x:%02x", pAddr->addr[1], pAddr->addr[0]);

  LOG_INFO("[INFO] :\t%s\n\r",name);

  displayPrintf(DISPLAY_ROW_NAME,"%s",name);

  displayPrintf(DISPLAY_ROW_BTADDR,"%x:%x:%x:%x:%x:%x",pAddr->addr[0], pAddr->addr[1],pAddr->addr[2], pAddr->addr[3],pAddr->addr[4], pAddr->addr[5]);

  /* write device name to the GATT database */
  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res)
  {
    LOG_INFO("[ERROR]:\tgecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }
}




/*********************************************************************************
 * @Function_name: GPIOINT														 *
 * @brief: This is a callback function that is invoked each time a GPIO interrupt*
 * in one of the pushbutton inputs occurs. Pin number is passed as parameter.	 *
 *																				 *
 * @param[in] pin  Pin number where interrupt occurs							 *
 *																				 *
 * @note This function is called from ISR context and therefore it is			 *
 *       not possible to call any BGAPI functions directly. The button state	 *
 *       change is signaled to the application using gecko_external_signal()	 *
 *       that will generate an event gecko_evt_system_external_signal_id		 *
 *       which is then handled in the main loop.								 *
 *********************************************************************************/
void gpioint(uint8_t pin)
{
	if(pin == __PB0_BUTTON_PIN)
	{
		ext_sig_handler |= PUSH_BUTTON_STATUS;
		gecko_external_signal(ext_sig_handler);
	}
}


/*************************************************************************************************
 * @function_name : ENABLE PUSH BUTTON INTERRUPTS			 									 *
 * @brief		  : Enable button interrupts for PB0. This GPIO is configured to trigger an		 *
 * 					an interrupt on rising edge(button released) and falling edge(button pressed)*
 * @param[in] pAddr  Pointer to Bluetooth address.												 *
 * @Reference	  : Silicon Lab's Bluetooth Mesh Switch Example									 *
 *************************************************************************************************/
void enable_push_button_interrupts()
{
	/* Clear pending IRQs */
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);

	/* Enable interrupt for PB0 */
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

	/* configure interrupt for PB0, both falling and rising edges */
	GPIO_ExtIntConfig(__PB0_BUTTON_PORT, __PB0_BUTTON_PIN, __PB0_BUTTON_PIN, true, true, true);

	/* register the callback function that is invoked when interrupt occurs */
	GPIOINT_CallbackRegister(__PB0_BUTTON_PIN, gpioint);
}




/********************************************************************************
 *@Function_name: HANDLE BUTTON PRESS											*
 *																				*
 *@brief		: Handling of button presses.									*
 * 				  This function is called from the main loop when application	*
 * 				  receives event gecko_evt_system_external_signal_id.			*
 *																				*
 * @param[in] button  Defines if button was pressed or released					*
 *																				*
 * @note This function is called from application context (not ISR)				*
 *       so it is safe to call BGAPI functions									*
 *																				*
 *@References	: Silicon Lab's Light/ Switch Example							*
 ********************************************************************************/
void handle_button_press(int button)
{
  /* short press adjusts light brightness, using Light Lightness model */
  if (button == PB0_PRESSED)
  {
	  LOG_INFO("[INFO] :\tButton PRESSED\n\r");
	  switch_pos = PB0_PRESSED;
	  LOG_INFO("[DEBUG]:\tSwitch Position : %d\n\r",switch_pos);
	  displayPrintf(DISPLAY_ROW_ACTION,"Button Pressed");
  }
  else if ( button == PB0_RELEASED)
  {
	  LOG_INFO("[INFO] :\tButton RELEASED\n\r");
	  switch_pos = PB0_RELEASED;
	  LOG_INFO("[DEBUG]:\tSwitch Position : %d\n\r",switch_pos);
	  displayPrintf(DISPLAY_ROW_ACTION,"Button Released");
  }
  LOG_INFO("[DEBUG]:\tSending ON OFF Request\n\r");
  send_onoff_request(0);
}



/************************************************************************************
 * @Function_name :SEND ONOFF REQUEST												*
 *																					*
 * @brief: This function publishes one generic on/off request to display the push 	*
 *         button state in the subscriber.											*
 *																					*
 * param[in] retrans  Indicates if this is the first request or a retransmission,	*
 *                    possible values are 0 = first request, 1 = retransmission.	*
 *																					*
 * @note This application sends multiple generic on/off requests for each			*
 *       very long button press to improve reliability.								*
 *       The transaction ID is not incremented in case of a retransmission.			*
 ************************************************************************************/
void send_onoff_request(int retrans)
{
  uint16 resp;
  uint16 delay;
  struct mesh_generic_request req;
  const uint32 transtime = 0; /* using zero transition time by default */

  req.kind = mesh_generic_request_pb0_press_release;
  if(switch_pos == PB0_PRESSED)
  {
	  req.pb0_press_release = MESH_GENERIC_PB0_PRESS_RELEASE_STATE_PRESS;
  }
  else if(switch_pos == PB0_RELEASED)
  {
	  req.pb0_press_release = MESH_GENERIC_PB0_PRESS_RELEASE_STATE_RELEASE;
  }

  LOG_INFO("[DEBUG]:\treq.pb0_press_release-> %d",req.pb0_press_release);

  /* increment transaction ID for each request, unless it's a retransmission */
  if (retrans == 0)
  {
    trid++;
  }

  /* delay for the request is calculated so that the last request will have a zero delay and each
   * of the previous request have delay that increases in 50 ms steps. For example, when using three
   * on/off requests per button press the delays are set as 100, 50, 0 ms
   */
  //delay = (request_count - 1) * 50;

  LOG_INFO("[INFO] :\tPublishing.......\n\r");
  resp = mesh_lib_generic_client_publish(
		  MESH_GENERIC_PB0_PRESS_RELEASE_CLIENT_MODEL_ID,
		  _elem_index,
		  trid,
		  &req,
		  transtime,	// transition time in ms
		  delay,
		  0			// flags
  );

  if (resp) {
    LOG_INFO("[ERROR]:\tgecko_cmd_mesh_generic_client_publish failed,code %x\r\n", resp);
    LOG_INFO("[INFO] :\tPublish FAILED\n\r");
  } else {
    LOG_INFO("[INFO] :\trequest sent, trid = %u, delay = %d\r\n", trid, delay);
    LOG_INFO("[INFO] :\tPublish SUCCESS\n\r");
  }

  /* keep track of how many requests has been sent */
  if (request_count > 0) {
    request_count--;
  }
}



/******************************************************************************
 * @Function_Name	: ONOFF_UPDATE AND PUBLISH
 * @brief			: Update generic on/off state and publish model state
 * 					  to the network.
 *
 * @param[in] element_index  Server model element index.
 *
 * @return Status of the update and publish operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
//static errorcode_t onoff_update_and_publish(uint16_t element_index)
//{
//  errorcode_t e;
//
//  e = onoff_update(element_index);
//
//  if (e == bg_err_success) {
//    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
//                                        element_index,
//                                        mesh_generic_state_on_off);
//    LOG_INFO("[INFO] :\tonoff_update return status = %d\n\r",e);
//
//  }
//
//  return e;
//}



/*******************************************************************************
 * @Function_Name	: ONOFF_UPDATE AND PUBLISH
 * Update generic on/off state.
 *
 * @param[in] element_index  Server model element index.
 *
 * @return Status of the update operation.
 *         Returns bg_err_success (0) if succeed, non-zero otherwise.
 ******************************************************************************/
//static errorcode_t onoff_update(uint16_t element_index)
//{
// struct mesh_generic_state current, target;
//
//  current.kind = mesh_generic_state_on_off;
//  current.on_off.on = PB0_PRESSED;
//  //current.on_off.on = 0;
//
//  target.kind = mesh_generic_state_on_off;
//  target.on_off.on = PB0_RELEASED;
//  //target.on_off.on = 1;
//
//  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
//                                        element_index,
//                                        &current,
//                                        &target,
//                                        0);
//}



/************************************************************************************
 * @Function_name : INIT MODELS														*
 *																					*
 * @References : Silicon Lab's Bluetooth Mesh Switch Example						*
 ************************************************************************************/
static void init_models(void)
{
	LOG_INFO("[DEBUG]:\tINITIALIZE MODELS\n\r");
	mesh_lib_generic_server_register_handler(MESH_GENERIC_PB0_PRESS_RELEASE_SERVER_MODEL_ID,
                                           0,
										   pb0_pressrelease_request,
										   pb0_pressrelease_change);
}



/********************************************************************************
 * This function is a handler for generic on/off change event.					*
 *																				*
 * @param[in] model_id       Server model ID.									*
 * @param[in] element_index  Server model element index.						*
 * @param[in] current        Pointer to current state structure.				*
 * @param[in] target         Pointer to target state structure.					*
 * @param[in] remaining_ms   Time (in milliseconds) remaining before transition	*
 *                           from current state to target state is complete.	*
 ********************************************************************************/
static void pb0_pressrelease_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
  //NOP
}



/****************************************************************************************
 * This function process the requests for the generic on/off model.						*
 *																						*
 * @param[in] model_id       Server model ID.											*
 * @param[in] element_index  Server model element index.								*
 * @param[in] client_addr    Address of the client model which sent the message.		*
 * @param[in] server_addr    Address the message was sent to.							*
 * @param[in] appkey_index   The application key index used in encrypting the request.	*
 * @param[in] request        Pointer to the request structure.							*
 * @param[in] transition_ms  Requested transition time (in milliseconds).				*
 * @param[in] delay_ms       Delay time (in milliseconds).								*
 * @param[in] request_flags  Message flags. Bitmask of the following:					*
 *                           - Bit 0: Nonrelayed. If nonzero indicates					*
 *                                    a response to a nonrelayed request.				*
 *                           - Bit 1: Response required. If nonzero client				*
 *                                    expects a response from the server.				*
 ****************************************************************************************/
static void pb0_pressrelease_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	LOG_INFO("[DEBUG]:\t Request->on_off = %d",request->pb0_press_release );
	if((request->pb0_press_release ) == 0)
	{
		LOG_INFO("[INFO] :\tBUTTON PRESSED\n\r");
		displayPrintf(DISPLAY_ROW_ACTION,"Button Pressed");
	}

	if((request->pb0_press_release ) == 1)
	{
		LOG_INFO("[INFO] :\tBUTTON RELEASED\n\r");
		displayPrintf(DISPLAY_ROW_ACTION,"Button Released");
	}

//	onoff_update_and_publish(element_index);
}



//void GPIO_EVEN_IRQHandler(void)
//{
//	LOG_INFO("IN EVEN IRQ HANDLER\n\r");
//
//	uint32_t flag;
//	flag = GPIO_IntGet();
//	GPIO_IntClear(flag);
//
//	if(flag & 0x800)
//	{
//		LOG_INFO("INTERRUPT DUE TO IR SENSOR\n\r");
//	}
//
//}
//
//void GPIO_ODD_IRQHandler(void)
//{
//	LOG_INFO("IN ODD IRQ HANDLER\n\r");
//
//	uint32_t flag;
//	flag = GPIO_IntGet();
//	GPIO_IntClear(flag);
//
//	if(flag & 0x800)
//	{
//		bool data = GPIO_PinInGet(IR_SENSOR_PORT,IR_SENSOR_PIN);
//		LOG_INFO("FLAG VALUE = %d\n\r",data);
//		LOG_INFO("INTERRUPT DUE TO IR SENSOR\n\r");
//	}
//
//
//}


/********************************************************************************************
 * @function_name : MAIN FUNCTION					 										*
 ********************************************************************************************/
int main(void)
{

  /* Initialize stack */
  gecko_main_init();

  /* Initialize Logging */
  logInit();

  /* Initialize GPIO  */
  gpioInit();

  /* Initialize Display */
  displayInit();

//  /* This part of the code will be moved to gpio.c. */
//  /******************************************************/
//  CMU_ClockEnable(cmuClock_LFA,true);
//  CMU_ClockEnable(cmuClock_GPIO,true);
//  GPIO_PinModeSet(IR_SENSOR_PORT,IR_SENSOR_PIN,gpioModeInputPull,true);
//  GPIO->IFC = 0x00000000;	//Clear all Gpio Interrupt flags
//  GPIO_IntConfig(IR_SENSOR_PORT,IR_SENSOR_PIN,0,1,1);
//  NVIC_EnableIRQ(GPIO_ODD_IRQn);
//  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
//  /*****************************************************/


  /* Infinite loop */
  while (1) {
	struct gecko_cmd_packet *evt = gecko_wait_event();
	bool pass = mesh_bgapi_listener(evt);
	if (pass) {
#if DEVICE_IS_ONOFF_PUBLISHER
//		handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		gecko_event_handler_pub(BGLIB_MSG_ID(evt->header), evt);
#else
		gecko_event_handler_sub(BGLIB_MSG_ID(evt->header), evt);
#endif

	}
  };
}

#if DEVICE_IS_ONOFF_PUBLISHER
/*************************************************************
 * GECKO EVENT HANDLER PUBLISHER							 *
 *************************************************************/
void gecko_event_handler_pub(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	uint16 result;

	if(evt == NULL)
	{
		return;
	}


	switch(evt_id)
	{
	/* SYSTEM BOOT ID */
	case gecko_evt_system_boot_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_SYSTEM_BOOT_ID:\n\r");
		/* If either PB0 or PB1 is pressed on system start up, then Factory reset */
		if((GPIO_PinInGet(__PB0_BUTTON_PORT,__PB0_BUTTON_PIN) == 0) || (GPIO_PinInGet(__PB1_BUTTON_PORT,__PB1_BUTTON_PIN) == 0))
		{
			initiate_factory_reset();
		}
		else
		{
			LOG_INFO("[DEBUG]:\tSET DEVICE NAME\n\r[DEBUG]:\tINITIALIZE MESH STACK\n\r");
			struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
			set_device_name(&pAddr->address);
			 displayPrintf(DISPLAY_ROW_CLIENTADDR,"Publisher");
			/* Initialize Mesh stack in Node operation mode, it will generate initialized event */
			result = gecko_cmd_mesh_node_init()->result;
			if (result)
			{
				LOG_INFO("[ERROR]:\tInitialization Failed! [0x%x]\n\r",result);
			}
		}
		break;

	case gecko_evt_mesh_node_initialized_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_MESH_NODE_INITIALIZED_ID:\n\r");
		LOG_INFO("[INFO] :\tNode Initialized!\n\r");

	      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

	      if(pData->provisioned)
	      {

	          /* Initialize generic client models */
	          gecko_cmd_mesh_generic_client_init();
	          lpn_init();
	          LOG_INFO("[INFO] :\tClient Initialization\n\r");
	          displayPrintf(DISPLAY_ROW_CONNECTION,"Node Initialized!");

	          /* Enable Push Button Interrupts  */
	          enable_push_button_interrupts();

	          _elem_index = 0;

	          /* Initialize mesh lib, upto 8 models */
	          mesh_lib_init(malloc,free,8);
	      }
	      else
	      {
	    	  displayPrintf(DISPLAY_ROW_ACTION, "UNPROVISIONED");
	    	  gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
	      }
	      break;


	case gecko_evt_mesh_node_provisioning_started_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_MESH_NODE_PROVISIONING_STARTED_ID:\n\r")
		LOG_INFO("[INFO] :\tProvisioning...\n\r");
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning...");
		break;

	case gecko_evt_mesh_node_provisioned_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_MESH_NODE_PROVISIONED_ID:\n\r")
		LOG_INFO("[INFO] :\tProvisioned!\n\r");
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioned");
		/* Initialize generic client models */
		gecko_cmd_mesh_generic_client_init();
		lpn_init();
		LOG_INFO("[INFO] :\tClient Initialization\n\r");
		displayPrintf(DISPLAY_ROW_CONNECTION,"Node Initialized!");

		/* Enable Push Button Interrupts  */
		enable_push_button_interrupts();

		_elem_index = 0;

		/* Initialize mesh lib, upto 8 models */
		mesh_lib_init(malloc,free,8);
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_MESH_NODE_PROVISIONING_FAILED_ID:\n\r")
		LOG_INFO("[INFO] :\tProvisioning Failed\n\r");
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning Failed");
		break;


	case gecko_evt_hardware_soft_timer_id:
		//LOG_INFO("[DEBUG]:\tGECKO_EVT_HARDWARE_SOFT_TIMER_ID:\n\r");
		switch(evt->data.evt_hardware_soft_timer.handle)
		{
		case TIMER_ID_FACTORY_RESET:
			/* reset the device to finish factory reset */
			gecko_cmd_system_reset(0);
			break;

		case LCD_UPDATE:
			displayUpdate();
			break;

		case LOG_UPDATE:
			timeCount = timeCount + 10;
			break;

		// find friend case
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
//		if (num_connections == 0) {
//			// try again in 2 seconds
//			gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FRIEND_FIND, 1);
//		}
		break;


	case gecko_evt_system_external_signal_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_SYSTEM_EXTERNAL_ID:\n\r");


		if((evt->data.evt_system_external_signal.extsignals) & PUSH_BUTTON_STATUS)
		{
			ext_sig_handler	&= ~(PUSH_BUTTON_STATUS);
			LOG_INFO("[DEBUG]:\tBUTTON PRESS DETECTED\n\r");
			if(GPIO_PinInGet(gpioPortF,6) == 0)
			{
				displayPrintf(DISPLAY_ROW_ACTION,"Button Pressed");
				handle_button_press(PB0_PRESSED);
				break;
			}
			if(GPIO_PinInGet(gpioPortF,6) == 1)
			{
				displayPrintf(DISPLAY_ROW_ACTION,"Button Released");
				handle_button_press(PB0_RELEASED);
				break;
			}
		}
	}
}

#else
/*************************************************************
 * GECKO EVENT HANDLER SUBSCRIBER							 *
 *************************************************************/
void gecko_event_handler_sub(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	uint16 result;

	if(evt == NULL)
	{
		return;
	}

	switch(evt_id)
	{
	/* SYSTEM BOOT ID */
	case gecko_evt_system_boot_id:
		LOG_INFO("[DEBUG SUB]:\tGECKO_EVT_SYSTEM_BOOT_ID:\n\r");
		  displayPrintf(DISPLAY_ROW_CLIENTADDR,"Subscriber");
		/* If either PB0 or PB1 is pressed on system start up, then Factory reset */
		if((GPIO_PinInGet(__PB0_BUTTON_PORT,__PB0_BUTTON_PIN) == 0) || (GPIO_PinInGet(__PB1_BUTTON_PORT,__PB1_BUTTON_PIN) == 0))
		{
			initiate_factory_reset();
		}
		else
		{
			LOG_INFO("[DEBUG SUB]:\tSET DEVICE NAME\n\r[DEBUG]:\tINITIALIZE MESH STACK\n\r");
			struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();
			set_device_name(&pAddr->address);
			/* Initialize Mesh stack in Node operation mode, it will generate initialized event */
			result = gecko_cmd_mesh_node_init()->result;
			if (result)
			{
				LOG_INFO("[ERROR SUB]:\tInitialization Failed! [0x%x]\n\r",result);
			}
		}
		break;

	case gecko_evt_mesh_node_initialized_id:
		LOG_INFO("[DEBUG SUB]:\tGECKO_EVT_MESH_NODE_INITIALIZED_ID:\n\r");
		LOG_INFO("[INFO SUB] :\tNode Initialized!\n\r");

	      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

	      if(pData->provisioned)
	      {
	          _primary_elem_index = 0;

	          /* Initialize generic server models */
	          gecko_cmd_mesh_generic_server_init();
	          //Initialize Friend functionality
	          LOG_INFO("Friend mode initialization");
	          uint16 res;
	          res = gecko_cmd_mesh_friend_init()->result;
	          if (res) {
	        	  LOG_INFO("Friend init failed 0x%x", res);
	          }

	          LOG_INFO("[INFO SUB] :\tServer Initialization\n\r");
	          displayPrintf(DISPLAY_ROW_CONNECTION,"Node Initialized!");

	          /* Initialize mesh lib, upto 8 models */
	          mesh_lib_init(malloc,free,9);
	          init_models();
//	          onoff_update_and_publish(_primary_elem_index);
	      }
	      else
	      {
	    	  gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
	      }
	      break;


	case gecko_evt_mesh_node_provisioning_started_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_MESH_NODE_PROVISIONING_STARTED_ID:\n\r")
		LOG_INFO("[INFO] :\tProvisioning...\n\r");
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning...");
		break;

	case gecko_evt_mesh_node_provisioned_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_MESH_NODE_PROVISIONED_ID:\n\r")
		LOG_INFO("[INFO] :\tProvisioned!\n\r");
		displayPrintf(DISPLAY_ROW_ACTION,"Sub Provisioned");
		_primary_elem_index = 0;

		/* Initialize generic server models */
		gecko_cmd_mesh_generic_server_init();
		//Initialize Friend functionality
		LOG_INFO("Friend mode initialization");
		uint16 res;
		res = gecko_cmd_mesh_friend_init()->result;
		if (res) {
			LOG_INFO("Friend init failed 0x%x", res);
		}

		LOG_INFO("[INFO SUB] :\tServer Initialization\n\r");
		displayPrintf(DISPLAY_ROW_CONNECTION,"Node Initialized!");

		/* Initialize mesh lib, upto 8 models */
		mesh_lib_init(malloc,free,9);
		init_models();
		//	          onoff_update_and_publish(_primary_elem_index);
		break;

	case gecko_evt_mesh_node_provisioning_failed_id:
		LOG_INFO("[DEBUG]:\tGECKO_EVT_MESH_NODE_PROVISIONING_FAILED_ID:\n\r")
		LOG_INFO("[INFO] :\tProvisioning Failed\n\r");
		displayPrintf(DISPLAY_ROW_ACTION,"Provisioning Failed");
		break;


	case gecko_evt_hardware_soft_timer_id:
		//LOG_INFO("[DEBUG]:\tGECKO_EVT_HARDWARE_SOFT_TIMER_ID:\n\r");
		switch(evt->data.evt_hardware_soft_timer.handle)
		{
		case TIMER_ID_FACTORY_RESET:
			/* reset the device to finish factory reset */
			gecko_cmd_system_reset(0);
			break;

		case LCD_UPDATE:
			displayUpdate();
			break;

		case LOG_UPDATE:
			timeCount = timeCount + 10;
			break;

		// find friend case
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
		}
		break;

	case gecko_evt_mesh_generic_server_client_request_id:
		LOG_INFO("[DEBUG SUB]:\tGECKO_EVT_MESH_GENERIC_SERVER_CLIENT_REQUEST_ID:\n\r");
	      // pass the server client request event to mesh lib handler that will invoke
	      // the callback functions registered by application
	      mesh_lib_generic_server_event_handler(evt);
	      break;
	}
}
#endif
