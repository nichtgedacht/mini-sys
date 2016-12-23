# mini-sys
Playground to start with minimum STM32F103C8 boards and ST7735 driven TFT displays with SD card slot from ebay 

* Tested and more elaborated code for steering a LCD 128x160 ST7735 driven display
* Works with maple mini clones or stm32 minimum system ( STM32F103C8 or STM32F103CB on a small PCB )
* Uses SD or SDHC class 4 cards with FATFS to read from (and to write to)
* Code for IMU MPU-9250 with examples
* Code for Futaba S-Bus with examples
* Shows examples of ADC and USB CDC usage
* Flies as quadrocopter now
* Based on ST Firmware from F1 and F4 archives (see each file for STM COPYRIGHT)
* Initial created by CubeMX (currently 4.15.1)

Installation ( Linux ):

* Install Eclipse (Mars)
* Install OpenSTM32 SystemWorkbench http://www.openstm32.org/Installing+System+Workbench+for+STM32+from+Eclipse
* Restart Eclipse with empty folder as workspace and click on "Workbench"
* Import: In "Workbench" goto File -> Import -> Git -> from Uri and select the current workspace folder as destination for mini-sys

Hints for usage:

* Compile and link first. Project -> "build all"
* Go to Project Explorer right click on Debug/mini-sys.elf
* Select "Debug As"-> "Debug Configurations ..."
* Change Name to "mini-sys.elf-Debug", click Apply
* In "Debugger" tab "Use local script" browse/select STM32F103C8T6-mini.cfg", click Apply
* Copy the BMPs to a SD card and insert.
* Find pin connections in mini-sys.pdf and mini-sys/Drivers/STM32F1xx_MiniSys/stm32f1xx_minisys.h 
* Click on the run symbol.
* Semantic error 'SysTick_IRQn' could not be resolved (cosmetics):
 In Project Explorer right click on top (mini-sys) -> Properties -> C/C++ General (Tree) -> Indexer -> click on link "Configure Workspace Settings" (new window) -> uncheck "index unused headers" -> Apply
* Optional: Install CubeMX (4.15.1) and load project to be able to modify the configuration the easy way.
* If all code is used, now it fits only to the STM32F103CBT6 (maple mini clone) with 128k flash. You have to edit STM32F103C8Ty_FLASH.ld (64K -> 128K) after having recreated the project with CubeMX

Bootloader:

If using my bootloader https://github.com/nichtgedacht/bootloader one have to edit 2 files:
* edit linker script STM32F103C8Tx_FLASH.ld. Change flash origin and length to: FLASH (rx) : ORIGIN = 0x8004000, LENGTH = 112K
* edit Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c. Change VECT_TAB_OFFSET to VECT_TAB_OFFSET 0x4000

This way the application will be linked properly for an offset of 16k in flash and the addresses of the interrupt vectors are correct.  

Configurator101:

Configuration of the Quadrocopter can be done with my configuaration tool https://github.com/nichtgedacht/configurator101

Screenshots:

![alt tag](https://cloud.githubusercontent.com/assets/18667858/21460292/ecc843fa-c946-11e6-9419-7a5d06d951ac.png)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/21460306/095ccb76-c947-11e6-98f6-e21c37afe99f.png)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/21460313/16fb87f4-c947-11e6-87fa-bab87ab2c2de.png)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/21460321/2a26ba24-c947-11e6-881b-8e0336a35119.png)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/21460331/3d4ccd1e-c947-11e6-8d17-af2b464c69ec.png)
 

Pictures:

![alt tag](https://cloud.githubusercontent.com/assets/18667858/17494738/589deeba-5db6-11e6-820a-5a0b6cb27699.JPG)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/17494984/6db2da6c-5db7-11e6-929d-5e37cae038cb.JPG)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/17495337/05e0f994-5db9-11e6-92c0-ccbcadc19b39.JPG)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/17495219/85d2defc-5db8-11e6-826f-5d35f28e533e.JPG)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/17495246/a946d104-5db8-11e6-9fe2-7bae8c5966d3.JPG)

![alt tag](https://cloud.githubusercontent.com/assets/18667858/17495270/ca519820-5db8-11e6-9c69-6378f580d438.JPG)



