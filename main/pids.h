#ifndef PIDS_H_INCLUDED
#define PIDS_H_INCLUDED

// OBD2 PIDS
// MODE 01 – STANDARD OBD-II PIDs (SAE J1979)
#define SAFE_PID_TIMEOUT 100

#define CURRENT_DATA_MODE         0x01
#define CURRENT_DATA_RESP         0x41

// 0x00–0x1F
#define SUPPORTED_PIDS_00_20      0x00
#define MONITOR_STATUS_SINCE_DTC  0x01
#define FREEZE_DTC                0x02
#define FUEL_SYSTEM_STATUS        0x03
#define CALCULATED_ENGINE_LOAD    0x04
#define COOLANT_TEMP              0x05
#define STFT_BANK1                0x06
#define LTFT_BANK1                0x07
#define STFT_BANK2                0x08
#define LTFT_BANK2                0x09
#define FUEL_PRESSURE             0x0A
#define INTAKE_MANIFOLD_PRESSURE  0x0B
#define ENGINE_RPM                0x0C
#define VEHICLE_SPEED             0x0D
#define TIMING_ADVANCE            0x0E
#define INTAKE_AIR_TEMP           0x0F
#define MAF_AIRFLOW_RATE          0x10
#define THROTTLE_POSITION         0x11
#define SECONDARY_AIR_STATUS      0x12
#define O2_SENSORS_PRESENT        0x13
#define O2_SENSOR1_VOLTAGE_STFT   0x14
#define O2_SENSOR2_VOLTAGE_STFT   0x15
#define O2_SENSOR3_VOLTAGE_STFT   0x16
#define O2_SENSOR4_VOLTAGE_STFT   0x17
#define O2_SENSOR5_VOLTAGE_STFT   0x18
#define O2_SENSOR6_VOLTAGE_STFT   0x19
#define O2_SENSOR7_VOLTAGE_STFT   0x1A
#define O2_SENSOR8_VOLTAGE_STFT   0x1B
#define OBD_STANDARDS             0x1C
#define O2_SENSORS_PRESENT_ALT    0x1D
#define AUX_INPUT_STATUS          0x1E
#define RUN_TIME_WITH_ENGINE_ON   0x1F

// 0x20–0x3F
#define SUPPORTED_PIDS_20_40      0x20
#define DIST_SINCE_DTC_CLEAR      0x21
#define EVAPORATIVE_PURGE         0x22
#define FUEL_RAIL_PRESSURE        0x23
#define FUEL_RAIL_GAUGE_PRESSURE  0x24
#define O2_S1_WR_CURRENT          0x24
#define O2_S2_WR_CURRENT          0x25
#define O2_S3_WR_CURRENT          0x26
#define O2_S4_WR_CURRENT          0x27
#define O2_S5_WR_CURRENT          0x28
#define O2_S6_WR_CURRENT          0x29
#define O2_S7_WR_CURRENT          0x2A
#define O2_S8_WR_CURRENT          0x2B
#define COMMANDED_EGR             0x2C
#define EGR_ERROR                 0x2D
#define COMMANDED_EVAP_PURGE      0x2E
#define FUEL_TANK_LEVEL_INPUT     0x2F
#define WARM_UPS_SINCE_DTC_CLEAR  0x30
#define DISTANCE_SINCE_DTC_CLEAR  0x31
#define EVAP_SYSTEM_VAPOR_PRESS   0x32
#define BAROMETRIC_PRESSURE       0x33

// 0x3C–0x5F
#define SUPPORTED_PIDS_40_60      0x40
#define CONTROL_MODULE_VOLTAGE    0x42
#define ENGINE_LOAD_ABSOLUTE      0x43
#define FUEL_AIR_EQUIV_RATIO      0x44
#define RELATIVE_THROTTLE         0x45
#define AMBIENT_AIR_TEMP          0x46
#define ABSOLUTE_THROTTLE_POSB    0x47
#define ABSOLUTE_THROTTLE_POSC    0x48
#define ACCELERATOR_PEDAL_POSD    0x49
#define ACCELERATOR_PEDAL_POSE    0x4A
#define ACCELERATOR_PEDAL_POSF    0x4B
#define THROTTLE_ACTUATOR         0x4C
#define RUN_TIME_MIL_ON           0x4D
#define TIME_SINCE_DTC_CLEARED    0x4E

#define ENGINE_OIL_TEMP           0x5C

namespace pid_calc {
  inline int rpm(int a, int b) {
    return (256 * a + b) / 4;
  }
  
  inline int coolant_temp(int a) {
    return a - 40;
  }

  inline int map_kpa(int a) {
    return a;
  }

  inline int bap_kpa(int a) {
    return a;
  }

  inline float boost_bar(int map_kpa, int bap_kpa) {
    return (map_kpa - bap_kpa) / 100.0f;
  }

  inline float boost_psi(int map_kpa, int bap_kpa) {
    return (map_kpa - bap_kpa) * 0.1450377f;
  }

  inline float engine_load(int a) {
    return (a * 100.0f) / 255.0f;  
  }

  inline int speed(int a) {
    return a;
  }

  inline float fuel_air_eq_ration(int a, int b) {
    return (2.0f / 65536.0f) * (256 * a + b);
  }

  inline int fuel_pressure(int a) {
    return 3 * a;
  }

  inline int fuel_rail_pressure_kpa(int a, int b) {
    return 10 * (256 * a + b);
  }

  inline float fuel_rail_pressure_bar(int a, int b) {
    return (256.0f * a + b) / 10.0f;
  }
}

#endif // PIDS_H_INCLUDED