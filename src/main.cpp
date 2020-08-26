#include "config.h"
#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <Sds011.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

unsigned int counter = 0;

void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

//payload struct
typedef struct __attribute__((packed))
{
    int32_t temperature;
    int32_t humidity;
    int32_t pressure;
    int32_t pm10;
    int32_t pm25;
} data_t;

extern data_t
    data; // Make struct for storing data globally available

static osjob_t sendjob;

HardwareSerial &serialSDS(Serial2);
Sds011Async<HardwareSerial> sds011(serialSDS);

Adafruit_BME280 bme; // I2C

data_t data = {0};

constexpr int pm_tablesize = 20;
static int pm10, pm25;

void start_SDS()
{
    ESP_LOGI(TAG, "wakeup SDS011");
    sds011.set_sleep(false);
}

void stop_SDS()
{
    ESP_LOGI(TAG, "sleep SDS011");
    sds011.set_sleep(true);
}

void do_send(osjob_t *j)
{

    start_SDS();

    ESP_LOGI(TAG, "Warmup %i sec", SDS_WARMUP);
    delay(SDS_WARMUP * 1000);
    ESP_LOGI(TAG, "Measurement");

    sds011.query_data(pm25, pm10);
    data.pm10 = pm10;
    data.pm25 = pm25;

    stop_SDS();

    ESP_LOGI(TAG, "Read BME280");
    data.temperature = bme.readTemperature() * 100;
    data.pressure = bme.readPressure();
    data.humidity = bme.readHumidity() * 100;

    ESP_LOGI(TAG, "Humidity    : %d.%02d %%", data.humidity / 100, data.humidity % 100);
    ESP_LOGI(TAG, "Temperature : %d.%02d °C ", data.temperature / 100, data.temperature % 100);
    ESP_LOGI(TAG, "Pressure    : %i hPa", data.pressure / 100);
    ESP_LOGI(TAG, "PM10        : %d.%02d µg/m³", data.pm10 / 10, data.pm10 % 10);
    ESP_LOGI(TAG, "PM2.5       : %d.%02d µg/m³", data.pm25 / 10, data.pm25 % 10);

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        ESP_LOGI(TAG, "OP_TXRXPEND, not sending");
    }
    else
    {
        LMIC_setTxData2(1, (unsigned char *)&data, sizeof(data), 0);
        ESP_LOGI(TAG, "msg=\"sending packet\" port=%d loraseq=%d bytes=%d",
                 1, LMIC.seqnoUp, sizeof(data));
        digitalWrite(LEDPIN, HIGH);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent(ev_t ev)
{

    ESP_LOGI(TAG, "%i", os_getTime());
    ESP_LOGI(TAG, ": ");
    switch (ev)
    {
    case EV_TXCOMPLETE:
        ESP_LOGI(TAG, "EV_TXCOMPLETE (includes waiting for RX windows)");

        if (LMIC.txrxFlags & TXRX_ACK)
        {
            ESP_LOGI(TAG, "Received ack");
        }

        if (LMIC.dataLen)
        {
            // data received in rx slot after tx
            ESP_LOGI(TAG, "Data Received: ");
            ESP_LOGI(TAG, "%s,%i", LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            ESP_LOGI(TAG, "\n");
            ESP_LOGI(TAG, "%i", LMIC.rssi);
        }

        // Schedule next transmission
        ESP_LOGI(TAG, "schedule next job");
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        digitalWrite(LEDPIN, LOW);
        break;
    case EV_JOINING:
        ESP_LOGI(TAG, "EV_JOINING: -> Joining...");
        break;
    case EV_JOINED:
    {
        ESP_LOGI(TAG, "EV_JOINED");
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
    }
    break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        ESP_LOGI(TAG, "EV_RXCOMPLETE");
        break;
    case EV_LINK_DEAD:
        ESP_LOGI(TAG, "EV_LINK_DEAD");
        break;
    case EV_LINK_ALIVE:
        ESP_LOGI(TAG, "EV_LINK_ALIVE");
        break;
    default:
        ESP_LOGI(TAG, "Unknown event %i", ev);
        break;
    }
}

void setup()
{

    // disable brownout detection
    (*((volatile uint32_t *)ETS_UNCACHED_ADDR((DR_REG_RTCCNTL_BASE + 0xd4)))) = 0;

    Serial.begin(115200);
    delay(100);

    ESP_LOGI(TAG, "Starting...");
    ESP_LOGI(TAG, "Sending Interval: %i sec", TX_INTERVAL);

    serialSDS.begin(9600, SERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX);
    delay(100);

    ESP_LOGI(TAG, "SDS011 start/stop and reporting device infos");
    start_SDS();

    String firmware_version;
    uint16_t device_id;
    if (!sds011.device_info(firmware_version, device_id))
    {
        ESP_LOGI(TAG, "Sds011::firmware_version() failed");
    }
    else
    {
        ESP_LOGI(TAG, "Sds011 firmware version: %s", firmware_version);
        ESP_LOGI(TAG, "Sds011 device id: %04x", device_id);
    }

    Sds011::Report_mode report_mode;
    if (!sds011.get_data_reporting_mode(report_mode))
    {
        ESP_LOGI(TAG, "Sds011::get_data_reporting_mode() failed");
    }
    if (Sds011::REPORT_ACTIVE != report_mode)
    {
        ESP_LOGI(TAG, "Turning on Sds011::REPORT_ACTIVE reporting mode");
        if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE))
        {
            ESP_LOGI(TAG, "Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed");
        }
    }

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    if (!bme.begin(BME_ADDRESS))
    {
        ESP_LOGI(TAG, "Could not find a valid BME280 sensor, check wiring!");
    }

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN, OUTPUT);

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    LMIC_setDrTxpow(DR_SF9, 14);

    // Start job
    do_send(&sendjob); // Will fire up also the join
    //LMIC_startJoining();
}

void loop()
{
    os_runloop_once();
}
