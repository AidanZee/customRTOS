To find main.c file to grade RTOS go to:

Core > Source > Main.c

Getting the code onto Your STM32 Board:
Download the Project:

Click the green code button and select download ZIP.

Unzip the files to a simple folder on your computer with preffered project name, do not use the espcode.c file for this step.

Open in STM32CubeIDE:

Open STM32CubeIDE.

Go to file > import

Browse to the folder where you unzipped the files, select the project, and click finish.

Build and Run:

Right click the project name and choose build project.

Click the run button.

Getting the code onto your esp32:

Copy and paste the espcode.c file into an arduino ide project.

Compile and upload the project to your esp32.

Hardware: 

Ensure a common ground among esp32 and stm32.

Connect the stm32 tx pin to the esp32 rx pin.

connec the scl and sda pins the to scl and sda pins on the ina219 current sensors.

power all components with 5V.
