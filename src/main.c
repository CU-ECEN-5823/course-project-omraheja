/*@Filename	 : main.c
 *@Author	 : Om Raheja
 *@Course	 : IoT Embedded Firmware [Spring 2019]
 *@Project   : MINE SAFETY USING BLUETOOTH MESH
 *@References: Bluetooth Mesh sample example codes [Light and Switch] have been used as references.
 * */

/* Includes */
#include "main.h"

/* Function Prototypes */
extern void gecko_main_init();
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);
extern void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

/* Main Function */
int main(void)
{
  /* Initialize stack */
  gecko_main_init();

  /*Initialize logging for Blue Gecko */
  logInit();

  /* Initialize the display */
  displayInit();

  /* Initialize GPIO pins  */
  gpioInit();

  /* Set default event as true */
  timer_events.def_event = true;

  // Initialize Low Energy Timer
  letimer_init();

  // Enabling LETIMER
  LETIMER_Enable(LETIMER0,true);

  // Configuring Sleep function
//  om_config();

  // Initialize I2C
  i2c_configure();

  /* Set GPIO Pin mode */
  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, true);

  /* SLeep Block begin according to the energy state */
  if((ENERGY_STATE == 0) | (ENERGY_STATE == 1) | (ENERGY_STATE == 2))
  {
	  SLEEP_SleepBlockBegin(ENERGY_STATE + 1);
  }

  /* Infinite loop */
  while (1) {
	struct gecko_cmd_packet *evt = gecko_wait_event();
	bool pass = mesh_bgapi_listener(evt);
	if (pass) {
		handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
	}

  };
}
