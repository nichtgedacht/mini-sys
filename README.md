# mini-sys
Playground to start with minimum STM32F103C8 boards and ST7735 driven TFT displays with SD card slot from ebay 

* Tested and more elaborated code for steering a LCD 128x160 ST7735 driven display
* Works with maple mini clones or stm32 minimum system ( STM32F103C8 or STM32F103CB on a small PCB )
* Uses SD or SDHC class 4 cards with FATFS to read from (and to write to)
* Code for IMU MPU-9250 with examples
* Code for Futaba S-Bus with examples
* Shows examples of ADC and USB CDC usage
* Based on ST Firmware from F1 and F4 archives (see each file for STM COPYRIGHT)
* Initial created by CubeMX (currently 4.15)

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
* Optional: Install CubeMX (4.15) and load project to be able to modify the configuration the easy way.
* If all code is used, now it fits only to the STM32F103CBT6 (maple mini clone) with 128k flash. You have to edit STM32F103C8Ty_FLASH.ld (64K -> 128K) after having recreated the project with CubeMX