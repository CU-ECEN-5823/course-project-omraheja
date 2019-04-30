# ECEN 5823 Bluetooth Mesh Project

## Project Report Links
Group Report Link: https://drive.google.com/drive/folders/1i9VWkrgTB9wjng7UBufxlCxIRutydFNX

Om Raheja Report Link: https://drive.google.com/drive/folders/1WHJSQL1TFp-1mP_ql-ged_eQFDn26k7e

### Project Status
1. Configured low power node [Completed & Working]
2. Established friendship with friend node [Completed & Working]
3. Interfaced noise/sound sensor [Completed & Working]
4. Interfaced buzzer, LCD and LED [Completed & Working]
5. Showing noise alerts locally and communicating alerts to other nodes in the mesh netwoek [Completed & Working]
6. Subscribe to friend messages - vibration sensor and push button functionality [Completed & Working]
7. Integrate humidity sensor with main appication code [Completed & Working]
8. Publish noise sensor and humidity sensor data [Completed & Working]
9. Encorporated interrupt driven routines instead of polling to conserve energy [Completed & Working]

### Models Used
LEVEL MODEL : Level model was used to publish noise and humidity sensor data. This model was also used to receive Flame sensor alert, Gas sensor alert and Vibration sensor alert. 

ON OFF MODEL : On-Off model was used to receive the LED0 state from the friend node. If the people count detected at friend node was 0, then an "Off" message was received. If the people count at friend node was detected to be more than 0, then an "On" message was received. This On-Off messages received were used to switch the on-board LED0 on and off.
