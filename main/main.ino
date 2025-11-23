#include <SPI.h>
#include <mcp2515.h>
#include "pids.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// MCP2515
MCP2515 mcp2515(10);
struct can_frame canMsg;

// TFT pins
#define TFT_CS 7
#define TFT_DC 6
#define TFT_RST 8
#define TFT_UPDATE_INT 50 // ms

// Mode switching button
#define MODE_BTN 3
#define MODE_BTN_DEBOUNCE_TIME_MS 200

#define MODES_SIZE 6

enum Modes : byte {
    DISP_BOOST = 0x00,
    DISP_COOLANT = 0x01,
    DISP_RPM = 0x02,
    DISP_ELOAD = 0x03,
    DISP_LAMBDA = 0x04,
    DISP_FUEL_RAIL_PRESS = 0x05
};


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Values
int rpm_val = 0;
int coolant_temp_val = 0;
int map_val = -1;
int bar_val = -1;
float boost_bar = 0;
float fuel_rail_pressure = 0;
bool request_map = true;

float engine_load_val = 0;
float fuel_air_eqr_val = 0;

// for pressing the button
volatile bool button_pressed = false;
unsigned long last_pressed_time = 0;

// current mode of what to send and what to display
byte curr_mode = DISP_BOOST;

// old values for smooth display
int old_rpm_val = -9999;
int old_coolant_val = -9999;
float old_boost_bar = -9999;
float old_lambda_val = -9999;
float old_eload_val = -9999;
float old_fuel_rail_pressure = -9999;

void send_PID(byte mode, byte pid) {
    canMsg.can_id  = 0x7DF;
    canMsg.can_dlc = 8;
    canMsg.data[0] = 0x02;
    canMsg.data[1] = mode;
    canMsg.data[2] = pid;

    for (int i = 3; i < 8; i++)
        canMsg.data[i] = 0;

    mcp2515.sendMessage(&canMsg);
}

void draw_unit(const char* unit) {
    tft.setTextSize(3);
    tft.setCursor(170, 10); 
    tft.print(unit);
    tft.print(" ");
}

void draw_mode_screen() {
    tft.fillScreen(ST77XX_BLACK); 
    
    tft.setCursor(10, 10);
    tft.setTextSize(3);

    // reset old values to force a redraw
    old_rpm_val = -9999; 
    old_coolant_val = -9999;
    old_boost_bar = -9999;
    old_lambda_val = -9999;
    old_eload_val = -9999;
    old_fuel_rail_pressure = -9999;

    switch (curr_mode) {
        case DISP_RPM:
            tft.println("RPM");
            break;
        case DISP_COOLANT:
            tft.println("Coolant");
            draw_unit("C");
            break;
        case DISP_BOOST:
            tft.println("Boost");
            draw_unit("bar");
            break;
        case DISP_LAMBDA:
            tft.println("Lambda");
            draw_unit("EQ");
            break; 
        case DISP_ELOAD:
            tft.println("E. Load");
            draw_unit("%");
            break;
        case DISP_FUEL_RAIL_PRESS:
            tft.println("Fuel P.");
            draw_unit("bar");
            break;
        default:
            tft.println("ERROR");
            break;
    }
}

void setup() {
    Serial.begin(115200);
    SPI.begin();

    // TFT
    tft.init(240, 240);
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);
    
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    
    tft.setTextWrap(false);

    tft.setTextSize(5);
    tft.setCursor(20, 40);
    tft.println("Booting...");

    // MCP2515
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    
    // Mode button
    pinMode(MODE_BTN, INPUT_PULLUP);

    // interrupt for the button
    attachInterrupt(digitalPinToInterrupt(MODE_BTN), on_button_press, FALLING);

    delay(200);
    draw_mode_screen();
}

void loop() {
    // get the current time
    unsigned long time_now = millis();

    static unsigned long last_request = 0;

    if(button_pressed) {
        handle_pressed_button();
    }

    // send the request based which mode we are in
    if(time_now - last_request > SAFE_PID_TIMEOUT) {
        switch(curr_mode) {
            case Modes::DISP_RPM:
                send_PID(CURRENT_DATA_MODE, ENGINE_RPM);
                break;
            case Modes::DISP_LAMBDA:
                send_PID(CURRENT_DATA_MODE, FUEL_AIR_EQUIV_RATIO);
                break;
            case Modes::DISP_ELOAD:
                send_PID(CURRENT_DATA_MODE, CALCULATED_ENGINE_LOAD);
                break;
            case Modes::DISP_COOLANT:
                send_PID(CURRENT_DATA_MODE, COOLANT_TEMP);
                break;
            case Modes::DISP_BOOST:
                if(request_map) {
                    send_PID(CURRENT_DATA_MODE, INTAKE_MANIFOLD_PRESSURE);
                }
                else {
                    send_PID(CURRENT_DATA_MODE, BAROMETRIC_PRESSURE);
                }
                request_map = !request_map;
                break;
            case Modes::DISP_FUEL_RAIL_PRESS:
                send_PID(CURRENT_DATA_MODE, FUEL_RAIL_PRESSURE);
                break;
            default:
                break;
        }
        last_request = time_now;
    }


    // Read incoming CAN frames
    while (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if (canMsg.can_id >= 0x7E8 && canMsg.can_id <= 0x7EF &&
            canMsg.data[1] == CURRENT_DATA_RESP) {

            byte pid = canMsg.data[2];
            byte a = canMsg.data[3];
            byte b = canMsg.data[4];

            switch (pid) {
                case ENGINE_RPM:
                    rpm_val = pid_calc::rpm(a, b);
                    break;
                case COOLANT_TEMP:
                    coolant_temp_val = pid_calc::coolant_temp(a);
                    break;
                case INTAKE_MANIFOLD_PRESSURE:
                    map_val = pid_calc::map_kpa(a);
                    boost_bar = pid_calc::boost_bar(map_val, bar_val);
                    break;
                case BAROMETRIC_PRESSURE:
                    bar_val = pid_calc::bap_kpa(a);
                    boost_bar = pid_calc::boost_bar(map_val, bar_val);
                    break;
                case CALCULATED_ENGINE_LOAD:
                    engine_load_val = pid_calc::engine_load(a);
                    break;
                case FUEL_AIR_EQUIV_RATIO:
                    fuel_air_eqr_val = pid_calc::fuel_air_eq_ration(a, b);
                    break;
                case FUEL_RAIL_PRESSURE:
                    fuel_rail_pressure = pid_calc::fuel_rail_pressure_bar(a, b);
                    break;
                default: 
                    break;
            }
        }
    }
    
    static unsigned long last_screen_update = 0;
    if (millis() - last_screen_update > TFT_UPDATE_INT) { 
        switch (curr_mode) {
            case DISP_RPM:
                if (rpm_val != old_rpm_val) {
                    tft.setCursor(10, 100);
                    tft.setTextSize(6);
                    tft.print(rpm_val);
                    tft.print("   "); // Padding
                    old_rpm_val = rpm_val;
                }
                break;
            case DISP_COOLANT:
                if (coolant_temp_val != old_coolant_val) {
                    tft.setCursor(10, 100);
                    tft.setTextSize(6);
                    tft.print(coolant_temp_val);
                    tft.print("   "); // Padding
                    old_coolant_val = coolant_temp_val;
                }
                break;
            case DISP_BOOST:
                if (abs(boost_bar - old_boost_bar) > 0.01) {
                    tft.setCursor(10, 100);
                    tft.setTextSize(6);
                    tft.print(boost_bar, 1);
                    tft.print("   "); // Padding
                    old_boost_bar = boost_bar;
                }
                break;
            case DISP_LAMBDA:
                if (abs(fuel_air_eqr_val - old_lambda_val) > 0.01) {
                    tft.setCursor(10, 100);
                    tft.setTextSize(6);
                    tft.print(fuel_air_eqr_val, 2);
                    tft.print("   "); // Padding
                    old_lambda_val = fuel_air_eqr_val;
                }
                break;
            case DISP_ELOAD:
                if (abs(engine_load_val - old_eload_val) > 0.1) {
                    tft.setCursor(10, 100);
                    tft.setTextSize(6);
                    tft.print(engine_load_val, 1);
                    tft.print("   "); // Padding
                    old_eload_val = engine_load_val;
                }
                break;
            case DISP_FUEL_RAIL_PRESS:
                if(old_fuel_rail_pressure != fuel_rail_pressure) {
                    tft.setCursor(10, 100);
                    tft.setTextSize(6);
                    tft.print(fuel_rail_pressure, 1);
                    tft.print("   "); // Padding
                    old_fuel_rail_pressure = fuel_rail_pressure;
                }
                break;
            default:
                break;
        }

        last_screen_update = millis();
    }
}

void on_button_press() {
    unsigned long time_now = millis();
    if(time_now - last_pressed_time > MODE_BTN_DEBOUNCE_TIME_MS) {
        button_pressed = true;
        last_pressed_time = time_now;
    }
}

void handle_pressed_button() {
    curr_mode = (curr_mode + 1) % MODES_SIZE;
    button_pressed = false;
    draw_mode_screen();
}