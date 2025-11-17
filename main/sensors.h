#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

typedef struct {
    float temperature_c;
} temperature_t;

typedef struct {
    vector3f_t linear_accel_g;
    vector3f_t gravity_g;
    vector3f_t gyro_dps;
    temperature_t temperature;
} imu_data_t;

typedef enum {
    GNSS_CONSTELLATION_GPS,
    GNSS_CONSTELLATION_GLONASS,
    GNSS_CONSTELLATION_GALILEO,
    GNSS_CONSTELLATION_BEIDOU,
    GNSS_CONSTELLATION_UNKNOWN
} gnss_constellation_t;

typedef enum {
    GNSS_SAT_STATUS_SEARCHING,
    GNSS_SAT_STATUS_TRACKING,
    GNSS_SAT_STATUS_USED
} gnss_sat_status_t;

typedef struct {
    uint8_t id;
    gnss_constellation_t constellation;
    float cn0_dbhz;
    float elevation_deg;
    float azimuth_deg;
    gnss_sat_status_t status;
} gnss_satellite_t;

#define GNSS_MAX_SATELLITES 32

typedef struct {
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float speed_kmh;
    float hdop;
    float vdop;
    float pdop;
    uint8_t sats_in_view;
    uint8_t sats_in_use;
    bool fix_valid;
    time_t timestamp_utc;
    gnss_satellite_t satellites[GNSS_MAX_SATELLITES];
} gnss_state_t;

typedef struct {
    float pressure_hpa;
    float altitude_m;
    temperature_t temperature;
} barometer_state_t;

typedef struct {
    vector3f_t magnetic_ut;
    temperature_t temperature;
} magnetometer_state_t;

typedef struct {
    float battery_voltage_v;
    uint8_t battery_percent;
    bool charging;
} power_state_t;

typedef struct {
    imu_data_t imu;
    magnetometer_state_t mag;
    barometer_state_t baro;
    gnss_state_t gnss;
    power_state_t power;
} sensors_state_t;

typedef enum {
    SENSORS_CALIBRATION_NONE = 0,
    SENSORS_CALIBRATION_IMU,
    SENSORS_CALIBRATION_MAG,
} sensors_calibration_type_t;

typedef struct {
    sensors_calibration_type_t active_type;
    float progress;
    bool running;
    bool success;
    char message[32];
} sensors_calibration_status_t;

void sensors_init(void);
void sensors_update(void);
void sensors_get_state(sensors_state_t *state_out);
bool sensors_start_calibration(sensors_calibration_type_t type);
void sensors_get_calibration_status(sensors_calibration_status_t *status_out);

