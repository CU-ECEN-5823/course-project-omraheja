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

  /* Infinite loop */
  while (1) {
	struct gecko_cmd_packet *evt = gecko_wait_event();
	bool pass = mesh_bgapi_listener(evt);
	if (pass) {
		handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
	}
  };
}
