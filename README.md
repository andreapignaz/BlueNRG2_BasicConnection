# BlueNRG2_BasicConnection
 Simple code to configure connection and GATT on a BlueNRG 2 microcontroller from ST. With a lot of nice comments :)

 ## What does it do (SPOILER: nothing useful 😎)
 1. Initialize the platform.
 2. Play with LEDs.
 3. Start Bluetooth, GAP and GATT framework.
 4. Make the device visible and connectable.
 5. Publish as GATT characteristics some random values. 
 6. *What can I do with GATT characteristics?* Read them using smartphone apps (nRF connect for Android, as example) or play with them using Pyhton (GATT library https://github.com/getsenic/gatt-python). 

## Future work
1. Make modular code 
2. Make it read values from the sensors embedded in the ST BlueTile and publish them via Bluetooth. 

## How to compile, build and so on.
1. Get KEIL uVision for ST Microelectronics from https://www2.keil.com/stmicroelectronics-stm32/mdk
2. Install on KEIL the Software Pack for the BlueNRG-2.
3. Get the Software Development Kit for the ST BlueTile from https://www.st.com/en/evaluation-tools/steval-bcn002v1b.html#tools-software
4. Unpack the SDK just downloaded, go to Project -> BLE_Examples -> BLE_SensorDemo and replace the existing folders with the ones from here. EWARM and TrueStudio folders are not necessary, as we're using uVision. 
5. Go inside the MDK-ARM folder, and open the uVision project. It will load everything, including the libraries from ST that are included in the SDK. 
6. Compile everything: you will get an .hex file in the MDK-ARM folder (or a subfolder). Use the ST RF-Flasher Utility from https://www.st.com/en/embedded-software/stsw-bnrgflasher.html to flash the HEX file (not the BIN!) to the board. 
7. The blue LED will turn on on the board and the device will be visible to any Bluetooth device. Use the nRF Android App or an application you wrote to take a look at its services and characteristics :)
8. If something went wrong, flash another .hex file from the SDK, for example the ResetManager (Firmware -> BLE_Examples -> BLE_OTA_ResetManager) and then flash your .hex again. This has something to do with the OTA system from ST. https://community.st.com/s/question/0D50X0000BB0RlUSQV/unable-to-run-precompiled-firmware-on-bluetile-stevalbcn002v1
