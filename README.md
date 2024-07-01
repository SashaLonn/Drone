## 3D Visualization MPU6050 WebServese using ESP32-s3 Feather
Sasha Soloviova ss226sh

3Dcubb visualization of accelerometer and gyrometer values from MPU-6050 using ESP32-S3 MCU. The results are displayed on a local web server using Wi-Fi.Data is read directly from registers using the C language

I had a problem with the installation of Espressif IDF. Besides that, it will take approximately 1 day.


It is my dream to build a drone from scratch. This is the first step in my drone project. These values will help define the drone's position and assist in self-adjusting its orientation. By building this project, I will gain a good understanding of how the ESP-IDF works, how to read and visualize sensor values, and much more.
## List of material

MC -Esp32-s3 is a microcontroller made by Espressif Systems. It has dual-core processors running up to 240 MHz, with Wi-Fi and Bluetooth for wireless connections.The ESP32-S3 includes UART, SPI, I2C, and ADC interfaces for connecting sensors and devices . Low power consumption. (See datastit https://cdn-learn.adafruit.com/downloads/pdf/adafruit-esp32-s3-feather.pdf)

Sensor MPU 6050 combines a 3-axis accelerometer and a 3-axis gyroscope. This allows it to measure both acceleration and rotation rate. Communicates with microcontrollers through I2C (Inter-Integrated Circuit) protocol, (https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf)

usb C Cabel for connection to MC and computer
 
6 cabels to connect sensor with mc

## Computer setup 

First, read the instructions (https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html).

You can install ESP-IDF in different ways; I use it as a VS Code extension (https://dl.espressif.com/dl/esp-idf/).

Choose the correct version; the version I use is 5.1.2. There may be differences depending on which version you choose.

"Before you start coding, you need to make sure everything is working fine. Use this command in the terminal inside VS Code: idf.py. If everything is working, you will see ![alt text](image.png).  These are all terminal commands you can use.

I recommend cloning my repository to work with the code. ESP-IDF has many configurations, which can be tricky to set up from scratch.

Next, we need to configure the project. Press F1 or Ctrl+Shift+P and type Device target. Choose your type of ESP chip, in my case esp32s3, and press Enter.

In your Windows search, type Device Manager and check which port your MCU is connected to. In my case, it's COM8 (your MCU must be connected to the computer).

Return to VS Code, press F1 again, and select Select port. Choose the port to which your MCU is connected

Now, go to Kconfig.projbuild and change your Wi-Fi credentials. I use my phone's hotspot, but you can use any network. Change the name and password accordingly.

If you encounter a CMake error that keeps popping up, just ignore it, as long as everything else is working correctly. I experience the same issue.

In ESP-IDF, you can build, upload, and monitor all together or separately. Press the fire icon at the bottom of the window to do all at the same time. Use the thunderbolt icon for only uploading and the screwdriver icon for building, or use the terminal. The first time, it will take some time to compile, but it will be faster the next time. It's good practice to occasionally clean the cache with idf.py clean or by pressing the garbage can icon.

If you encounter errors during uploads, do the following:

Press and hold the BOOT button.
While holding the BOOT button, press and release the Reset button.
Continue holding the BOOT button for a few more seconds, then release it

***chech if you have compiler for C,if don't  (https://code.visualstudio.com/docs/cpp/config-mingw)


## Platform 

If I knew how difficult it would be, I would have chosen another platform, like PlatformIO. I don't recommend choosing ESP-IDF if you are a beginner. However, I have learned more during these weeks than I did in the past year. I chose ESP-IDF because I will go to practice, and they use it there. I want to work with embedded programming, and ESP-IDF gives developers more control over the hardware.

ESP-IDF is better for advanced embedded development because it gives you direct access to hardware features and detailed settings. It's the official tool from Espressif, so it has the latest support for all ESP32 features. This is important for projects where you need precise control and optimization.

PlatformIO, on the other hand, is easier to use, especially for beginners. It supports many different microcontrollers and works with various IDEs, making it more user-friendly. PlatformIO is a good choice if you are new to embedded programming or working with different types of microcontrollers.

## The code


## Transmitting the data 


## Presenting the data









