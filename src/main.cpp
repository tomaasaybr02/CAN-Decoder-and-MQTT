#include <Arduino.h>
#include "driver/twai.h"  // ESP32 TWAI driver (for CAN)
#include "batteries_gps_motor_2.h"  // FF DTU Boat CAN Libraty
#include <WiFi.h>
#include <PubSubClient.h>


//--------------------------------------------------
//  CONFIGURATION
//--------------------------------------------------

// Wi-Fi Confiuration
const char* ssid = "DTU_FF_Boat_Team";
const char* password = "DTUBoatFF2425#";

// MQTT configuration
const char* mqtt_server = "192.168.50.10"; // Raspberry Pi IP
const uint16_t mqttPort = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// CAN PINS:
#define CAN_TX GPIO_NUM_5
#define CAN_RX GPIO_NUM_4

//--------------------------------------------------
//  FUNCTIONS
//--------------------------------------------------

void sendCanMessage() {
    twai_message_t txMessage;
    txMessage.identifier = 0x10;
    txMessage.extd = 0;
    txMessage.data_length_code = 4;
    txMessage.data[0] = 0x11;
    txMessage.data[1] = 0x22;
    txMessage.data[2] = 0x33;
    txMessage.data[3] = 0x44;

    int retries = 3;
    while (retries--) {
        if (twai_transmit(&txMessage, pdMS_TO_TICKS(1000)) == ESP_OK) {
            Serial.println("✅ CAN message sent successfully!");
            return;
        } else {
            Serial.println("⚠️ Failed to send CAN message. Retrying...");
            delay(100);
        }
    }
    Serial.println("❌ Final failure: CAN message could not be sent.");
}

// Helper – reconnect WiFi 
static void connectWiFi()
{
    Serial.printf("\n📶 Connecting to %s ", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\n✅ WiFi connected – IP: %s\n", WiFi.localIP().toString().c_str());
}

// Helper – reconnect MQTT 
static void connectMQTT()
{
    while (!mqttClient.connected()) {
        Serial.println("Connecting to MQTT broker...");
        if (mqttClient.connect("ESP32Telemetry")) {
            Serial.println("Connected to MQTT broker!");
        } else {
            Serial.print("Error on connection. Status: ");
            Serial.println(mqttClient.state());
            delay(2000);
        }
    }
}


// Publish helper (float with two decimals) 
static void publishFloat(const String& topic, float value, uint8_t decimals = 2)
{
    char payload[32];
    dtostrf(value, /*width*/0, decimals, payload);
    mqttClient.publish(topic.c_str(), payload, true); // retain
    Serial.printf("📤 MQTT  %s  ->  %s\n", topic.c_str(), payload);
}

// Publish helper (integer / boolean) 
static void publishInt(const String& topic, long value)
{
    char payload[16];
    ltoa(value, payload, 10);
    mqttClient.publish(topic.c_str(), payload, true);
    Serial.printf("📤 MQTT  %s  ->  %s\n", topic.c_str(), payload);
}

//--------------------------------------------------
//  SET UP
//--------------------------------------------------

void setup() {
    Serial.begin(115200);

    // Wi-Fi connection
    connectWiFi();

    // MQTT Broker connection
    mqttClient.setServer(mqtt_server, mqttPort);
    connectMQTT();

    // CAN TWAI Initialization:
    while (!Serial);

    Serial.println("🚀 Initializing TWAI (CAN) in NORMAL mode...");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("✅ TWAI driver installed successfully.");
        if (twai_start() == ESP_OK) {
            Serial.println("✅ TWAI driver started in NORMAL mode.");
        } else {
            Serial.println("❌ Failed to start TWAI driver.");
        }
    } else {
        Serial.println("❌ Failed to install TWAI driver.");
    }
}

//--------------------------------------------------
//  LOOP
//--------------------------------------------------

void loop() {

    // Keep MQTT connection alive
    if (!mqttClient.connected()) connectMQTT();
    mqttClient.loop();


    // Read CAN messages
    static uint32_t lastSendTime = 0;
    uint32_t currentTime = millis();

    twai_message_t rxMessage;
    while (twai_receive(&rxMessage, pdMS_TO_TICKS(10)) == ESP_OK) {
        Serial.printf("📩 Received CAN Frame - ID: 0x%03X, Length: %d, Data: ", 
                      rxMessage.identifier, rxMessage.data_length_code);
        for (int i = 0; i < rxMessage.data_length_code; i++) {
            Serial.printf("%02X ", rxMessage.data[i]);
        }
        Serial.println();

        // Decoding and showing messages filtered by ID
        switch (rxMessage.identifier) {
            case 0x359: {  // Battery_VoltCurrTemp_1
                batteries_gps_motor_2_battery_volt_curr_temp_1_t msg;
                batteries_gps_motor_2_battery_volt_curr_temp_1_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🔋 Decoded Battery_VoltCurrTemp_1:");
                Serial.printf("   Voltage: %.2f V\n", batteries_gps_motor_2_battery_volt_curr_temp_1_battery_voltage_1_decode(msg.battery_voltage_1));
                Serial.printf("   Current: %.2f A\n", batteries_gps_motor_2_battery_volt_curr_temp_1_battery_current_1_decode(msg.battery_current_1));
                Serial.printf("   Temperature: %.2f °C\n", batteries_gps_motor_2_battery_volt_curr_temp_1_battery_temp_1_decode(msg.battery_temp_1));

                float v = batteries_gps_motor_2_battery_volt_curr_temp_1_battery_voltage_1_decode(msg.battery_voltage_1);
                float c = batteries_gps_motor_2_battery_volt_curr_temp_1_battery_current_1_decode(msg.battery_current_1);
                float t = batteries_gps_motor_2_battery_volt_curr_temp_1_battery_temp_1_decode(msg.battery_temp_1);

                publishFloat("boat/telemetry/battery/1/voltage", v);
                publishFloat("boat/telemetry/battery/1/current", c);
                publishFloat("boat/telemetry/battery/1/temperature", t);
                break;
            }
            
            case 0x360: {  // Battery_VoltCurrTemp_2
                batteries_gps_motor_2_battery_volt_curr_temp_2_t msg;
                batteries_gps_motor_2_battery_volt_curr_temp_2_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🔋 Decoded Battery_VoltCurrTemp_2:");
                Serial.printf("   Voltage: %.2f V\n", batteries_gps_motor_2_battery_volt_curr_temp_2_battery_voltage_2_decode(msg.battery_voltage_2));
                Serial.printf("   Current: %.2f A\n", batteries_gps_motor_2_battery_volt_curr_temp_2_battery_current_2_decode(msg.battery_current_2));
                Serial.printf("   Temperature: %.2f °C\n", batteries_gps_motor_2_battery_volt_curr_temp_2_battery_temp_2_decode(msg.battery_temp_2));

                float v = batteries_gps_motor_2_battery_volt_curr_temp_2_battery_voltage_2_decode(msg.battery_voltage_2);
                float c = batteries_gps_motor_2_battery_volt_curr_temp_2_battery_current_2_decode(msg.battery_current_2);
                float t = batteries_gps_motor_2_battery_volt_curr_temp_2_battery_temp_2_decode(msg.battery_temp_2);

                publishFloat("boat/telemetry/battery/2/voltage",      v);
                publishFloat("boat/telemetry/battery/2/current",      c);
                publishFloat("boat/telemetry/battery/2/temperature",  t);
                break;
            }

            case 0x361: {  // Battery_VoltCurrTemp_3
                batteries_gps_motor_2_battery_volt_curr_temp_3_t msg;
                batteries_gps_motor_2_battery_volt_curr_temp_3_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🔋 Decoded Battery_VoltCurrTemp_3:");
                Serial.printf("   Voltage: %.2f V\n", batteries_gps_motor_2_battery_volt_curr_temp_3_battery_voltage_3_decode(msg.battery_voltage_3));
                Serial.printf("   Current: %.2f A\n", batteries_gps_motor_2_battery_volt_curr_temp_3_battery_current_3_decode(msg.battery_current_3));
                Serial.printf("   Temperature: %.2f °C\n", batteries_gps_motor_2_battery_volt_curr_temp_3_battery_temp_3_decode(msg.battery_temp_3));

                float v = batteries_gps_motor_2_battery_volt_curr_temp_3_battery_voltage_3_decode(msg.battery_voltage_3);
                float c = batteries_gps_motor_2_battery_volt_curr_temp_3_battery_current_3_decode(msg.battery_current_3);
                float t = batteries_gps_motor_2_battery_volt_curr_temp_3_battery_temp_3_decode(msg.battery_temp_3);

                publishFloat("boat/telemetry/battery/3/voltage",      v);
                publishFloat("boat/telemetry/battery/3/current",      c);
                publishFloat("boat/telemetry/battery/3/temperature",  t);
                break;
            }

            case 0x362: {  // Battery_VoltCurrTemp_4
                batteries_gps_motor_2_battery_volt_curr_temp_4_t msg;
                batteries_gps_motor_2_battery_volt_curr_temp_4_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🔋 Decoded Battery_VoltCurrTemp_4:");
                Serial.printf("   Voltage: %.2f V\n", batteries_gps_motor_2_battery_volt_curr_temp_4_battery_voltage_4_decode(msg.battery_voltage_4));
                Serial.printf("   Current: %.2f A\n", batteries_gps_motor_2_battery_volt_curr_temp_4_battery_current_4_decode(msg.battery_current_4));
                Serial.printf("   Temperature: %.2f °C\n", batteries_gps_motor_2_battery_volt_curr_temp_4_battery_temp_4_decode(msg.battery_temp_4));
                
                float v = batteries_gps_motor_2_battery_volt_curr_temp_4_battery_voltage_4_decode(msg.battery_voltage_4);
                float c = batteries_gps_motor_2_battery_volt_curr_temp_4_battery_current_4_decode(msg.battery_current_4);
                float t = batteries_gps_motor_2_battery_volt_curr_temp_4_battery_temp_4_decode(msg.battery_temp_4);

                publishFloat("boat/telemetry/battery/4/voltage",      v);
                publishFloat("boat/telemetry/battery/4/current",      c);
                publishFloat("boat/telemetry/battery/4/temperature",  t);
                break;
            }

            case 0x35A: {  // extra_battery_info_1
                batteries_gps_motor_2_extra_battery_info_1_t msg;
                batteries_gps_motor_2_extra_battery_info_1_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚠️ Decoded Extra Battery Info 1 (Alarm/Warning Bits):");
                Serial.printf("   General Alarm: %d\n", msg.general_alarm_1);
                Serial.printf("   High Voltage Alarm: %d\n", msg.battery_high_voltage_alarm_1);
                Serial.printf("   Low Voltage Alarm: %d\n", msg.battery_low_voltage_alarm_1);
                Serial.printf("   High Temp Alarm: %d\n", msg.battery_high_temperature_alarm_1);
                Serial.printf("   Low Temp Alarm: %d\n", msg.battery_low_temperature_alarm_1);
                Serial.printf("   High Temp Charge Alarm: %d\n", msg.battery_high_temp_charge_alarm_1);
                Serial.printf("   Low Temp Charge Alarm: %d\n", msg.battery_low_temp_charge_alarm_1);
                Serial.printf("   High Current Alarm: %d\n", msg.battery_high_current_alarm_1);
                Serial.printf("   High Chaege Current Alarm: %d\n", msg.battery_high_charge_curr_alarm_1);
                Serial.printf("   Contactor Alarm: %d\n", msg.contactor_alarm_1);
                Serial.printf("   Short Circuit Alarm: %d\n", msg.short_circuit_alarm_1);
                Serial.printf("   BMS Internal Alarm: %d\n", msg.bms_internal_alarm_1);
                Serial.printf("   Cell Imbalance Alarm: %d\n", msg.cell_imbalance_alarm_1);
                Serial.printf("   General Warning: %d\n", msg.general_warning_1);
                Serial.printf("   High Voltage Warning: %d\n", msg.battery_high_voltage_warning_1);
                Serial.printf("   Low Voltage Warning: %d\n", msg.battery_low_voltage_warning_1);
                Serial.printf("   High Temp Warning: %d\n", msg.battery_high_temp_warning_1);
                Serial.printf("   Low Temp Warning: %d\n", msg.battery_low_temp_warning_1);
                Serial.printf("   High Temp Charge Warning: %d\n", msg.battery_high_temp_charge_warn_1);
                Serial.printf("   Low Temp Charge Warning: %d\n", msg.battery_low_temp_charge_warn_1);
                Serial.printf("   High Current Warning: %d\n", msg.battery_high_current_warning_1);
                Serial.printf("   Contactor Warning: %d\n", msg.contactor_warning_1);
                Serial.printf("   Short Circuit Warning: %d\n", msg.short_circuit_warning_1);
                Serial.printf("   BMS Intenal Warning: %d\n", msg.bms_internal_warning_1);
                Serial.printf("   Cell Imbalance Warning: %d\n", msg.cell_imbalance_warning_1);
                Serial.printf("   System Status: %d\n", msg.system_status_1);

                publishInt("boat/telemetry/battery/1/status", msg.system_status_1);
                publishInt("boat/telemetry/battery/1/alarm", msg.general_alarm_1);
                break;
            }

            case 0x35B: {  // extra_battery_info_2
                batteries_gps_motor_2_extra_battery_info_2_t msg;
                batteries_gps_motor_2_extra_battery_info_2_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚠️ Decoded Extra Battery Info 2 (Alarm/Warning Bits):");
                Serial.printf("   General Alarm: %d\n", msg.general_alarm_2);
                Serial.printf("   High Voltage Alarm: %d\n", msg.battery_high_voltage_alarm_2);
                Serial.printf("   Low Voltage Alarm: %d\n", msg.battery_low_voltage_alarm_2);
                Serial.printf("   High Temp Alarm: %d\n", msg.battery_high_temperature_alarm_2);
                Serial.printf("   Low Temp Alarm: %d\n", msg.battery_low_temperature_alarm_2);
                Serial.printf("   High Temp Charge Alarm: %d\n", msg.battery_high_temp_charge_alarm_2);
                Serial.printf("   Low Temp Charge Alarm: %d\n", msg.battery_low_temp_charge_alarm_2);
                Serial.printf("   High Current Alarm: %d\n", msg.battery_high_current_alarm_2);
                Serial.printf("   High Chaege Current Alarm: %d\n", msg.battery_high_charge_curr_alarm_2);
                Serial.printf("   Contactor Alarm: %d\n", msg.contactor_alarm_2);
                Serial.printf("   Short Circuit Alarm: %d\n", msg.short_circuit_alarm_2);
                Serial.printf("   BMS Internal Alarm: %d\n", msg.bms_internal_alarm_2);
                Serial.printf("   Cell Imbalance Alarm: %d\n", msg.cell_imbalance_alarm_2);
                Serial.printf("   General Warning: %d\n", msg.general_warning_2);
                Serial.printf("   High Voltage Warning: %d\n", msg.battery_high_voltage_warning_2);
                Serial.printf("   Low Voltage Warning: %d\n", msg.battery_low_voltage_warning_2);
                Serial.printf("   High Temp Warning: %d\n", msg.battery_high_temp_warning_2);
                Serial.printf("   Low Temp Warning: %d\n", msg.battery_low_temp_warning_2);
                Serial.printf("   High Temp Charge Warning: %d\n", msg.battery_high_temp_charge_warn_2);
                Serial.printf("   Low Temp Charge Warning: %d\n", msg.battery_low_temp_charge_warn_2);
                Serial.printf("   High Current Warning: %d\n", msg.battery_high_current_warning_2);
                Serial.printf("   Contactor Warning: %d\n", msg.contactor_warning_2);
                Serial.printf("   Short Circuit Warning: %d\n", msg.short_circuit_warning_2);
                Serial.printf("   BMS Intenal Warning: %d\n", msg.bms_internal_warning_2);
                Serial.printf("   Cell Imbalance Warning: %d\n", msg.cell_imbalance_warning_2);
                Serial.printf("   System Status: %d\n", msg.system_status_2);

                publishInt("boat/telemetry/battery/2/status", msg.system_status_2);
                publishInt("boat/telemetry/battery/2/alarm", msg.general_alarm_2);
                break;
            }

            case 0x35C: {  // extra_battery_info_3
                batteries_gps_motor_2_extra_battery_info_3_t msg;
                batteries_gps_motor_2_extra_battery_info_3_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚠️ Decoded Extra Battery Info 3 (Alarm/Warning Bits):");
                Serial.printf("   General Alarm: %d\n", msg.general_alarm_3);
                Serial.printf("   High Voltage Alarm: %d\n", msg.battery_high_voltage_alarm_3);
                Serial.printf("   Low Voltage Alarm: %d\n", msg.battery_low_voltage_alarm_3);
                Serial.printf("   High Temp Alarm: %d\n", msg.battery_high_temperature_alarm_3);
                Serial.printf("   Low Temp Alarm: %d\n", msg.battery_low_temperature_alarm_3);
                Serial.printf("   High Temp Charge Alarm: %d\n", msg.battery_high_temp_charge_alarm_3);
                Serial.printf("   Low Temp Charge Alarm: %d\n", msg.battery_low_temp_charge_alarm_3);
                Serial.printf("   High Current Alarm: %d\n", msg.battery_high_current_alarm_3);
                Serial.printf("   High Chaege Current Alarm: %d\n", msg.battery_high_charge_curr_alarm_3);
                Serial.printf("   Contactor Alarm: %d\n", msg.contactor_alarm_3);
                Serial.printf("   Short Circuit Alarm: %d\n", msg.short_circuit_alarm_3);
                Serial.printf("   BMS Internal Alarm: %d\n", msg.bms_internal_alarm_3);
                Serial.printf("   Cell Imbalance Alarm: %d\n", msg.cell_imbalance_alarm_3);
                Serial.printf("   General Warning: %d\n", msg.general_warning_3);
                Serial.printf("   High Voltage Warning: %d\n", msg.battery_high_voltage_warning_3);
                Serial.printf("   Low Voltage Warning: %d\n", msg.battery_low_voltage_warning_3);
                Serial.printf("   High Temp Warning: %d\n", msg.battery_high_temp_warning_3);
                Serial.printf("   Low Temp Warning: %d\n", msg.battery_low_temp_warning_3);
                Serial.printf("   High Temp Charge Warning: %d\n", msg.battery_high_temp_charge_warn_3);
                Serial.printf("   Low Temp Charge Warning: %d\n", msg.battery_low_temp_charge_warn_3);
                Serial.printf("   High Current Warning: %d\n", msg.battery_high_current_warning_3);
                Serial.printf("   Contactor Warning: %d\n", msg.contactor_warning_3);
                Serial.printf("   Short Circuit Warning: %d\n", msg.short_circuit_warning_3);
                Serial.printf("   BMS Intenal Warning: %d\n", msg.bms_internal_warning_3);
                Serial.printf("   Cell Imbalance Warning: %d\n", msg.cell_imbalance_warning_3);
                Serial.printf("   System Status: %d\n", msg.system_status_3);

                publishInt("boat/telemetry/battery/3/status", msg.system_status_3);
                publishInt("boat/telemetry/battery/3/alarm", msg.general_alarm_3);
                break;
            }

            case 0x35D: {  // extra_battery_info_4
                batteries_gps_motor_2_extra_battery_info_4_t msg;
                batteries_gps_motor_2_extra_battery_info_4_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚠️ Decoded Extra Battery Info 4 (Alarm/Warning Bits):");
                Serial.printf("   General Alarm: %d\n", msg.general_alarm_4);
                Serial.printf("   High Voltage Alarm: %d\n", msg.battery_high_voltage_alarm_4);
                Serial.printf("   Low Voltage Alarm: %d\n", msg.battery_low_voltage_alarm_4);
                Serial.printf("   High Temp Alarm: %d\n", msg.battery_high_temperature_alarm_4);
                Serial.printf("   Low Temp Alarm: %d\n", msg.battery_low_temperature_alarm_4);
                Serial.printf("   High Temp Charge Alarm: %d\n", msg.battery_high_temp_charge_alarm_4);
                Serial.printf("   Low Temp Charge Alarm: %d\n", msg.battery_low_temp_charge_alarm_4);
                Serial.printf("   High Current Alarm: %d\n", msg.battery_high_current_alarm_4);
                Serial.printf("   High Chaege Current Alarm: %d\n", msg.battery_high_charge_curr_alarm_4);
                Serial.printf("   Contactor Alarm: %d\n", msg.contactor_alarm_4);
                Serial.printf("   Short Circuit Alarm: %d\n", msg.short_circuit_alarm_4);
                Serial.printf("   BMS Internal Alarm: %d\n", msg.bms_internal_alarm_4);
                Serial.printf("   Cell Imbalance Alarm: %d\n", msg.cell_imbalance_alarm_4);
                Serial.printf("   General Warning: %d\n", msg.general_warning_4);
                Serial.printf("   High Voltage Warning: %d\n", msg.battery_high_voltage_warning_4);
                Serial.printf("   Low Voltage Warning: %d\n", msg.battery_low_voltage_warning_4);
                Serial.printf("   High Temp Warning: %d\n", msg.battery_high_temp_warning_4);
                Serial.printf("   Low Temp Warning: %d\n", msg.battery_low_temp_warning_4);
                Serial.printf("   High Temp Charge Warning: %d\n", msg.battery_high_temp_charge_warn_4);
                Serial.printf("   Low Temp Charge Warning: %d\n", msg.battery_low_temp_charge_warn_4);
                Serial.printf("   High Current Warning: %d\n", msg.battery_high_current_warning_4);
                Serial.printf("   Contactor Warning: %d\n", msg.contactor_warning_4);
                Serial.printf("   Short Circuit Warning: %d\n", msg.short_circuit_warning_4);
                Serial.printf("   BMS Intenal Warning: %d\n", msg.bms_internal_warning_4);
                Serial.printf("   Cell Imbalance Warning: %d\n", msg.cell_imbalance_warning_4);
                Serial.printf("   System Status: %d\n", msg.system_status_4);

                publishInt("boat/telemetry/battery/4/status", msg.system_status_4);
                publishInt("boat/telemetry/battery/4/alarm", msg.general_alarm_4);
                break;
            }

            case 0x355: {  // SOC_SOH_1
                batteries_gps_motor_2_soc_soh_1_t msg;
                batteries_gps_motor_2_soc_soh_1_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🪫 Decoded SOC_SOH_1:");
                Serial.printf("   SOC Value: %d\n", msg.soc_value_1);
                Serial.printf("   SOH Value: %d\n", msg.soh_value_1);
                Serial.printf("   High Res SOC: %.2f%%\n", msg.high_res_soc_1*0.01);

                publishFloat("boat/telemetry/battery/1/soc", msg.high_res_soc_1*0.01, 2);
                break;
            }

            case 0x356: {  // SOC_SOH_2
                batteries_gps_motor_2_soc_soh_2_t msg;
                batteries_gps_motor_2_soc_soh_2_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🪫 Decoded SOC_SOH_2:");
                Serial.printf("   SOC Value: %d\n", msg.soc_value_2);
                Serial.printf("   SOH Value: %d\n", msg.soh_value_2);
                Serial.printf("   High Res SOC: %.2f%%\n", msg.high_res_soc_2*0.01);

                publishFloat("boat/telemetry/battery/2/soc", msg.high_res_soc_2*0.01, 2);
                break;
            }

            case 0x357: {  // SOC_SOH_3
                batteries_gps_motor_2_soc_soh_3_t msg;
                batteries_gps_motor_2_soc_soh_3_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🪫 Decoded SOC_SOH_3:");
                Serial.printf("   SOC Value: %d\n", msg.soc_value_3);
                Serial.printf("   SOH Value: %d\n", msg.soh_value_3);
                Serial.printf("   High Res SOC: %.2f%%\n", msg.high_res_soc_3*0.01);

                publishFloat("boat/telemetry/battery/3/soc", msg.high_res_soc_3*0.01, 2);
                break;
            }

            case 0x358: {  // SOC_SOH_4
                batteries_gps_motor_2_soc_soh_4_t msg;
                batteries_gps_motor_2_soc_soh_4_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🪫 Decoded SOC_SOH_4:");
                Serial.printf("   SOC Value: %d\n", msg.soc_value_4);
                Serial.printf("   SOH Value: %d\n", msg.soh_value_4);
                Serial.printf("   High Res SOC: %.2f%%\n", msg.high_res_soc_4*0.01);

                publishFloat("boat/telemetry/battery/4/soc", msg.high_res_soc_4*0.01, 2);
                break;
            }
        
            case 0x351: {  // charge_status_1
                batteries_gps_motor_2_charge_status_1_t msg;
                batteries_gps_motor_2_charge_status_1_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                 Serial.println("🔌 Decoded Charge Status 1:");
                // Serial.printf("   Charge Voltage Limit: %d\n", msg.charge_voltage_limit_1);
                // Serial.printf("   Max Charge Current: %d\n", msg.max_charge_current_1);
                // Serial.printf("   Max Discharge Current: %d\n", msg.max_discharge_current_1);
                // Serial.printf("   Discharge Voltage: %d\n", msg.discharge_voltage_1);
                Serial.printf("   Charge Voltage Limit: %.1f V\n", batteries_gps_motor_2_charge_status_1_charge_voltage_limit_1_decode(msg.charge_voltage_limit_1));
                Serial.printf("   Max Charge Current: %.1f A\n", batteries_gps_motor_2_charge_status_1_max_charge_current_1_decode(msg.max_charge_current_1));
                Serial.printf("   Max Discharge Current: %.1f A\n", batteries_gps_motor_2_charge_status_1_max_discharge_current_1_decode(msg.max_discharge_current_1));
                Serial.printf("   Discharge Voltage: %.1f V\n", batteries_gps_motor_2_charge_status_1_discharge_voltage_1_decode(msg.discharge_voltage_1));
                break;
            }

            case 0x352: {  // charge_status_2
                batteries_gps_motor_2_charge_status_2_t msg;
                batteries_gps_motor_2_charge_status_2_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🔌 Decoded Charge Status 2:");
                Serial.printf("   Charge Voltage Limit: %.1f V\n", batteries_gps_motor_2_charge_status_2_charge_voltage_limit_2_decode(msg.charge_voltage_limit_2));
                Serial.printf("   Max Charge Current: %.1f A\n", batteries_gps_motor_2_charge_status_2_max_charge_current_2_decode(msg.max_charge_current_2));
                Serial.printf("   Max Discharge Current: %.1f A\n", batteries_gps_motor_2_charge_status_2_max_discharge_current_2_decode(msg.max_discharge_current_2));
                Serial.printf("   Discharge Voltage: %.1f V\n", batteries_gps_motor_2_charge_status_2_discharge_voltage_2_decode(msg.discharge_voltage_2));
                break;
            }

            case 0x353: {  // charge_status_3
                batteries_gps_motor_2_charge_status_3_t msg;
                batteries_gps_motor_2_charge_status_3_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🔌 Decoded Charge Status 3:");
                Serial.printf("   Charge Voltage Limit: %.1f V\n", batteries_gps_motor_2_charge_status_3_charge_voltage_limit_3_decode(msg.charge_voltage_limit_3));
                Serial.printf("   Max Charge Current: %.1f A\n", batteries_gps_motor_2_charge_status_3_max_charge_current_3_decode(msg.max_charge_current_3));
                Serial.printf("   Max Discharge Current: %.1f A\n", batteries_gps_motor_2_charge_status_3_max_discharge_current_3_decode(msg.max_discharge_current_3));
                Serial.printf("   Discharge Voltage: %.1f V\n", batteries_gps_motor_2_charge_status_3_discharge_voltage_3_decode(msg.discharge_voltage_3));
                break;
            }

            case 0x354: {  // charge_status_4
                batteries_gps_motor_2_charge_status_4_t msg;
                batteries_gps_motor_2_charge_status_4_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🔌 Decoded Charge Status 4:");
                Serial.printf("   Charge Voltage Limit: %.1f V\n", batteries_gps_motor_2_charge_status_4_charge_voltage_limit_4_decode(msg.charge_voltage_limit_4));
                Serial.printf("   Max Charge Current: %.1f A\n", batteries_gps_motor_2_charge_status_4_max_charge_current_4_decode(msg.max_charge_current_4));
                Serial.printf("   Max Discharge Current: %.1f A\n", batteries_gps_motor_2_charge_status_4_max_discharge_current_4_decode(msg.max_discharge_current_4));
                Serial.printf("   Discharge Voltage: %.1f V\n", batteries_gps_motor_2_charge_status_4_discharge_voltage_4_decode(msg.discharge_voltage_4));
                break;
            }

            case 0x1: {  // GnssStatus
                batteries_gps_motor_2_gnss_status_t msg;
                batteries_gps_motor_2_gnss_status_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssStatus:");
                Serial.printf("   Fix Type: %d\n", msg.fix_type);
                //Serial.printf("   Fix types: %d\n", batteries_gps_motor_2_gnss_status_fix_type_decode(msg.fix_type));
                Serial.printf("   Satellites: %d\n", msg.satellites);
                //Serial.printf("   Satellites: %d\n", batteries_gps_motor_2_gnss_status_satellites_decode(msg.satellites));
                break;
            }

            case 0x2: {  // GnssTime
                batteries_gps_motor_2_gnss_time_t msg;
                batteries_gps_motor_2_gnss_time_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssTime:");
                Serial.printf("   Time Valid: %d\n", msg.time_valid);
                Serial.printf("   Time Confirmed: %d\n", msg.time_confirmed);
                //Serial.printf("   Epoch: %d\n", msg.epoch);
                Serial.printf("   Epoch: %.3f s\n", batteries_gps_motor_2_gnss_time_epoch_decode(msg.epoch));
                break;
            }

            case 0x3: {  // GnssPosition
                batteries_gps_motor_2_gnss_position_t msg;
                batteries_gps_motor_2_gnss_position_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssPosition:");
                Serial.printf("   Position Valid: %d\n", msg.position_valid);
                Serial.printf("   Latitude: %.6f°\n", batteries_gps_motor_2_gnss_position_latitude_decode(msg.latitude));
                Serial.printf("   Longitude: %.6f°\n", batteries_gps_motor_2_gnss_position_longitude_decode(msg.longitude));
                //Serial.printf("   Longitude (manual decode): %.6f°\n", ((double)msg.longitude * 1e-6) - 180.0);
                Serial.printf("   Position Accuracy: %.1f m\n", batteries_gps_motor_2_gnss_position_position_accuracy_decode(msg.position_accuracy));

                float lat = batteries_gps_motor_2_gnss_position_latitude_decode(msg.latitude);
                float lon = batteries_gps_motor_2_gnss_position_longitude_decode(msg.longitude);

                publishFloat("boat/telemetry/gps/latitude",  lat, 6);
                publishFloat("boat/telemetry/gps/longitude", lon, 6);                
                break;
            }

            case 0x4: {  // GnssAltitude
                batteries_gps_motor_2_gnss_altitude_t msg;
                batteries_gps_motor_2_gnss_altitude_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssAltitude:");
                Serial.printf("   Altitude Valid: %d\n", msg.altitude_valid);
                Serial.printf("   Altitude: %.1f m\n", batteries_gps_motor_2_gnss_altitude_altitude_decode(msg.altitude));
                Serial.printf("   Altitude Accuracy: %.1f m\n", batteries_gps_motor_2_gnss_altitude_altitude_accuracy_decode(msg.altitude_accuracy));

                float alt = batteries_gps_motor_2_gnss_altitude_altitude_decode(msg.altitude);

                publishFloat("boat/telemetry/gps/altitude", alt, 1);
                break;
            }

            case 0x5: {  // GnssAttitude
                batteries_gps_motor_2_gnss_attitude_t msg;
                batteries_gps_motor_2_gnss_attitude_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssAttitude:");
                Serial.printf("   Attitude Valid: %d\n", msg.attitude_valid);
                Serial.printf("   Roll: %.1f°\n", batteries_gps_motor_2_gnss_attitude_roll_decode(msg.roll));
                Serial.printf("   Roll Accuracy: %.1f°\n", batteries_gps_motor_2_gnss_attitude_roll_accuracy_decode(msg.roll_accuracy));
                Serial.printf("   Pitch: %.1f°\n", batteries_gps_motor_2_gnss_attitude_pitch_decode(msg.pitch));
                Serial.printf("   Pitch Accuracy: %.1f°\n", batteries_gps_motor_2_gnss_attitude_pitch_accuracy_decode(msg.pitch_accuracy));
                Serial.printf("   Heading: %.1f°\n", batteries_gps_motor_2_gnss_attitude_heading_decode(msg.heading));
                Serial.printf("   Heading Accuracy: %.1f°\n", batteries_gps_motor_2_gnss_attitude_heading_accuracy_decode(msg.heading_accuracy));
                break;
            }

            case 0x6: {  // GnssOdo
                batteries_gps_motor_2_gnss_odo_t msg;
                batteries_gps_motor_2_gnss_odo_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssOdo:");
                Serial.printf("   Distance Valid: %d\n", msg.distance_valid);
                Serial.printf("   Distance Trip: %d m\n", msg.distance_trip);
                Serial.printf("   Distance Accuracy: %d m\n", msg.distance_accuracy);
                Serial.printf("   Distance Total: %d km\n", msg.distance_total);
                break;
            }

            case 0x7: {  // GnssSpeed
                batteries_gps_motor_2_gnss_speed_t msg;
                batteries_gps_motor_2_gnss_speed_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssSpeed:");
                Serial.printf("   Speed Valid: %d\n", msg.speed_valid);
                Serial.printf("   Speed: %.3f m/s\n", batteries_gps_motor_2_gnss_speed_speed_decode(msg.speed));
                Serial.printf("   Speed Accuracy: %.3f m/s\n", batteries_gps_motor_2_gnss_speed_speed_accuracy_decode(msg.speed_accuracy));
                break;
            }

            case 0x8: {  // GnssGeofence
                batteries_gps_motor_2_gnss_geofence_t msg;
                batteries_gps_motor_2_gnss_geofence_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssGeofence:");
                Serial.printf("   Fence Valid: %d\n", msg.fence_valid);
                Serial.printf("   Fence Combined: %d\n", msg.fence_combined);
                Serial.printf("   Fence 1: %d\n", msg.fence1);
                Serial.printf("   Fence 2: %d\n", msg.fence2);
                Serial.printf("   Fence 3: %d\n", msg.fence3);
                Serial.printf("   Fence 4: %d\n", msg.fence4);
                break;
            }

            case 0x9: {  // GnssImu
                batteries_gps_motor_2_gnss_imu_t msg;
                batteries_gps_motor_2_gnss_imu_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("📡 Decoded GnssImu:");
                Serial.printf("   Imu Valid: %d\n", msg.imu_valid);
                Serial.printf("   Acceleration X: %.3f m/s²\n", batteries_gps_motor_2_gnss_imu_acceleration_x_decode(msg.acceleration_x));
                Serial.printf("   Acceleration Y: %.3f m/s²\n", batteries_gps_motor_2_gnss_imu_acceleration_y_decode(msg.acceleration_y));
                Serial.printf("   Acceleration Z: %.3f m/s²\n", batteries_gps_motor_2_gnss_imu_acceleration_z_decode(msg.acceleration_z));
                Serial.printf("   Angular Rate X: %.2f °/s\n", batteries_gps_motor_2_gnss_imu_angular_rate_x_decode(msg.angular_rate_x));
                Serial.printf("   Angular Rate Y: %.2f °/s\n", batteries_gps_motor_2_gnss_imu_angular_rate_y_decode(msg.angular_rate_y));
                Serial.printf("   Angular Rate Z: %.2f °/s\n", batteries_gps_motor_2_gnss_imu_angular_rate_z_decode(msg.angular_rate_z));
                break;
            }

            case 0x1f200: {  // Engine_Param_Rapid_Update
                batteries_gps_motor_2_engine_param_rapid_update_t msg;
                batteries_gps_motor_2_engine_param_rapid_update_unpack(&msg, rxMessage.data, rxMessage.data_length_code);
                
                Serial.println("🚤 Decoded Engine_Param_Rapid_Update:");
                Serial.printf("   Motor Speed: %.2f RPM\n", batteries_gps_motor_2_engine_param_rapid_update_motor_speed_decode(msg.motor_speed));

                float speed = batteries_gps_motor_2_engine_param_rapid_update_motor_speed_decode(msg.motor_speed);

                publishFloat("boat/telemetry/motor/speed", speed);
                break;
            }

            case 0x1f201: {  // Engine_Param_Dynamic
                batteries_gps_motor_2_engine_param_dynamic_t msg;
                batteries_gps_motor_2_engine_param_dynamic_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("🚤 Decoded Engine_Param_Dynamic:");
                Serial.printf("   Engine Temp: %d\n", msg.engine_temp);
                Serial.printf("   Engine Temp: %.2f K\n", batteries_gps_motor_2_engine_param_dynamic_engine_temp_decode(msg.engine_temp));

                float temp = batteries_gps_motor_2_engine_param_dynamic_engine_temp_decode(msg.engine_temp);

                publishFloat("boat/telemetry/motor/temperature", temp);
                break;
            }

            case 0x1f202: {  // Electric_Drive_Status
                batteries_gps_motor_2_electric_drive_status_t msg;
                batteries_gps_motor_2_electric_drive_status_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚡️ Decoded Electric_Drive_Status:");
                Serial.printf("   Motor Temperature: %d\n", msg.motor_temperature);
                Serial.printf("   Motor Temperature: %.2f K\n", batteries_gps_motor_2_electric_drive_status_motor_temperature_decode(msg.motor_temperature));
                break;
            }

            case 0x1f203: {  // Electric_Energy_Storage_Statuse
                batteries_gps_motor_2_electric_energy_storage_statuse_t msg;
                batteries_gps_motor_2_electric_energy_storage_statuse_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚡️ Decoded Electric_Energy_Storage_Statuse:");
                Serial.printf("   State of Charge: %d\n", msg.state_of_charge);
                Serial.printf("   Time Remaining: %d\n", msg.time_remaining);
                Serial.printf("   Time Remaining: %.0f s\n", batteries_gps_motor_2_electric_energy_storage_statuse_time_remaining_decode(msg.time_remaining));
                break;
            }

            case 0x1f206: {  // Electric_Drive_Info
                batteries_gps_motor_2_electric_drive_info_t msg;
                batteries_gps_motor_2_electric_drive_info_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚡️ Decoded Electric_Drive_Info:");
                Serial.printf("   Motor ID: %d\n", msg.motor_id);
                Serial.printf("   Motor Type: %d\n", msg.motor_type);
                Serial.printf("   Motor Voltage Rating: %.1f V\n", batteries_gps_motor_2_electric_drive_info_motor_voltage_rating_decode(msg.motor_voltage_rating));
                Serial.printf("   Max Cont Motor Power: %d W\n", msg.max_cont_motor_power);
                Serial.printf("   Max Boost Motor Power: %d W\n", msg.max_boost_motor_power);
                Serial.printf("   Max Motor Temp Rating: %.2f K\n", batteries_gps_motor_2_electric_drive_info_max_motor_temp_rating_decode(msg.max_motor_temp_rating));
                Serial.printf("   Rated Motor Speed: %.2f RPM\n", batteries_gps_motor_2_electric_drive_info_rated_motor_speed_decode(msg.rated_motor_speed));
                Serial.printf("   Max Controller Temp Rating: %.2f K\n", batteries_gps_motor_2_electric_drive_info_max_controller_temp_rating_decode(msg.max_controller_temp_rating));
                Serial.printf("   Motor Shaft Torque Rating: %.1f\n", batteries_gps_motor_2_electric_drive_info_motor_shaft_torque_rating_decode(msg.motor_shaft_torque_rating));
                Serial.printf("   Motor DC Volt Derating Threshold: %.1f V\n", batteries_gps_motor_2_electric_drive_info_motor_dc_volt_derating_threshold_decode(msg.motor_dc_volt_derating_threshold));
                Serial.printf("   Motor DC Volt Cut Off Threshold: %.1f V\n", batteries_gps_motor_2_electric_drive_info_motor_dc_volt_cut_off_threshold_decode(msg.motor_dc_volt_cut_off_threshold));
                Serial.printf("   Motor Hours: %d s\n", msg.motor_hours);

                //float power = ??
                break;
            }

            case 0x1f402: {  // Electric_Drive_Status_Rapid_Up
                batteries_gps_motor_2_electric_drive_status_rapid_up_t msg;
                batteries_gps_motor_2_electric_drive_status_rapid_up_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚡️ Decoded Electric_Drive_Status_Rapid_Up:");
                Serial.printf("   Motor DC Voltage: %.1f V\n", batteries_gps_motor_2_electric_drive_status_rapid_up_motor_dc_voltage_decode(msg.motor_dc_voltage));
                Serial.printf("   Motor DC Current: %.1f A\n", batteries_gps_motor_2_electric_drive_status_rapid_up_motor_dc_current_decode(msg.motor_dc_current));

                float voltage = batteries_gps_motor_2_electric_drive_status_rapid_up_motor_dc_voltage_decode(msg.motor_dc_voltage);
                float current = batteries_gps_motor_2_electric_drive_status_rapid_up_motor_dc_current_decode(msg.motor_dc_current);

                publishFloat("boat/telemetry/motor/voltage", voltage);
                publishFloat("boat/telemetry/motor/current", current);
                break;
            }

            case 0x1f802: {  // COG_and_SOG
                batteries_gps_motor_2_cog_and_sog_t msg;
                batteries_gps_motor_2_cog_and_sog_unpack(&msg, rxMessage.data, rxMessage.data_length_code);

                Serial.println("⚡️ Decoded COG_and_SOG:");
                Serial.printf("   COG: %.4f rad\n", batteries_gps_motor_2_cog_and_sog_cog_decode(msg.cog));
                Serial.printf("   SOG: %.2f m/s\n", batteries_gps_motor_2_cog_and_sog_sog_decode(msg.sog));
                break;
            }


            default:
                Serial.printf("🟡 Unhandled CAN ID: 0x%03X\n", rxMessage.identifier);
                break;
        }
    }

    // if (currentTime - lastSendTime > 5000) {
    //     lastSendTime = currentTime;
    //     sendCanMessage();
    // }
}