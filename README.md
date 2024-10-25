# W5500 UDP Communication with STM32

This repository demonstrates how to establish and manage UDP communication on an STM32 microcontroller using the W5500 Ethernet module. The program configures the STM32 to send and receive UDP packets, displaying messages through UART for debugging and status updates.

## Features
- **W5500 Ethernet Communication:** Configures the STM32 to communicate over UDP with the W5500 module using SPI.
- **UDP Socket Handling:** Establishes and manages a UDP socket connection, allowing data transmission and reception.
- **UART Debugging:** Outputs status and debugging messages over UART for easy monitoring.

## Requirements
- **Hardware:**
  - STM32 microcontroller
  - W5500 Ethernet module ([W5500 Module Product Page](https://robu.in/product/w5500-tcp-ip-spi-to-lan-ethernet-interface-spi-to-lan-ethernet-converter/?gad_source=1&gclid=CjwKCAjwg-24BhB_EiwA1ZOx8gmG97EPR4Eeb_cDfobfCvqw8VmkLKJ5izCyKA48QGf5VNaT3iyzrRoCoS0QAvD_BwE))
  - UART-to-USB converter (optional for debugging)

- **Software:**
  - STM32CubeIDE
  - W5500 library ([Library Link](https://github.com/Wiznet/ioLibrary_Driver.git))

## Setup
1. **Hardware Connections:**
   - Connect the W5500 module to the STM32 using the SPI interface.
   - Configure GPIO for the SPI pins, CS pin, and UART for debugging.

2. **Code Configuration:**
   - Update network settings (MAC, IP, Subnet) in `main.c` as required.
   - Configure UART and SPI settings in STM32CubeIDE according to your hardware setup.

3. **Build and Flash:**
   - Build the project in STM32CubeIDE.
   - Flash the firmware onto the STM32 microcontroller.

## Usage
- On power-up, the STM32 initializes the W5500 module and configures the UDP socket.
- The STM32 then listens for incoming UDP messages and sends status updates at regular intervals.
- Debugging messages are output through UART for easy monitoring.

## License
This project is licensed under the MIT License
