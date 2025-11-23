# Car gauge using Arduino UNO R3 and MCP2515

**Author:** Tomas Baublys  
**Course:** Introduction to robotics  

---

## Project Overview
This project uses UNO R3 with MCP2515 to communicate with the cars OBD-II port and display the wanted stats to the display. Keep in mind, your vehicle might not support all the PID's this program sends. You can check what is supported using any OBD-II scanner or by sending a PID with 0x00 for 0x01 - 0x20 PID's and so on... This was tested on Skoda Fabia RS mk2. Since it uses the car CAN bus, it can only be used with vehicles after 2008.

You can use the button to switch between different display modes.

---

## Hardware Setup

- You can view or download the component list here: [partslist.csv](partslist.csv)

## Circuit Diagram
![Circuit photo](photo.jpg)

## Wirring Diagram
![Wirring diagram](wirring.pdf)

---

## Software

- Written in **Arduino C++**.
- Uses MCP2515 library

## Usage
You will have to create the provided wiring diagram.

1. Connect the hardware according to the pinout table.
   For more details, refer to the [Wiring Diagram](wirring.pdf)  
2. Upload the code to your UNO R3.
3. Have fun!.

--- 

# Notes
- Feel free to experiment with more CAN request (you can find them in pids.h file or ![wikipedia OBD II PIDS](https://en.wikipedia.org/wiki/OBD-II_PIDs)), for now I have included the more important ones for my car, but I plan on making the gauge more universal.


