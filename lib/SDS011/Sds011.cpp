/*
Sds011.cpp

ESP8266/ESP32 Arduino library for the SDS011 particulation matter sensor.

The MIT License (MIT)

Copyright (c) 2016 Krzysztof A. Adamski
Copyright (c) 2018-2019 Dirk O. Kaar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "Sds011.h"

bool Sds011::device_info(String& firmware_version, uint16_t& device_id) {
    _send_cmd(CMD_FIRMWARE_VERSION, NULL, 0);
    bool ok = _read_response(CMD_FIRMWARE_VERSION);
    if (!(ok && crc_ok())) { return false; }
    firmware_version = String(_buf[3]) + "-" + String(_buf[4]) + "-" + String(_buf[5]);
    device_id = (static_cast<uint16_t>(_buf[6]) << 8) + _buf[7];
    return true;
}

bool Sds011::set_data_reporting_mode(Report_mode mode) {
    uint8_t data[] = { 0x1, mode };
    _send_cmd(CMD_DATA_REPORTING_MODE, data, 2);
    return _read_response(CMD_DATA_REPORTING_MODE) && crc_ok() && _buf[3] == 0x1 && _buf[4] == mode;
}

bool Sds011::get_data_reporting_mode(Report_mode& mode) {
    _send_cmd(CMD_DATA_REPORTING_MODE, NULL, 0);
    bool ok = _read_response(CMD_DATA_REPORTING_MODE);
    if (!(ok && crc_ok() && _buf[3] == 0x0)) { return false; }
    mode = static_cast<Report_mode>(_buf[4]);
    return true;
}

bool Sds011::set_device_id(uint16_t device_id) {
    uint8_t data[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, static_cast<uint8_t>(device_id >> 8), static_cast<uint8_t>(device_id & 0xff) };
    _send_cmd(CMD_SET_DEVICE_ID, data, 12);
    return _read_response(CMD_SET_DEVICE_ID) && crc_ok() && _buf[6] == device_id >> 8 && _buf[7] == (device_id & 0xff);
}

bool Sds011::set_sleep(bool sleep) {
    uint8_t data[] = { 0x1, !sleep };
    _send_cmd(CMD_SLEEP_AND_WORK, data, 2);
    return _read_response(CMD_SLEEP_AND_WORK) && crc_ok() && _buf[3] == 0x1 && !_buf[4] == sleep;
}

bool Sds011::get_sleep(bool& sleep) {
    _send_cmd(CMD_SLEEP_AND_WORK, NULL, 0);
    bool ok = _read_response(CMD_SLEEP_AND_WORK);
    if (!(ok && crc_ok() && _buf[3] == 0x0)) { return false; }
    sleep = !_buf[4];
    return true;
}

bool Sds011::set_working_period(uint8_t minutes) {
    uint8_t data[] = { 0x1, minutes };
    _send_cmd(CMD_WORKING_PERIOD, data, 2);
    return _read_response(CMD_WORKING_PERIOD) && crc_ok() && _buf[3] == 0x1 && _buf[4] == minutes;
}

bool Sds011::get_working_period(uint8_t& minutes) {
    _send_cmd(CMD_WORKING_PERIOD, NULL, 0);
    bool ok = _read_response(CMD_WORKING_PERIOD);
    if (!(ok && crc_ok() && _buf[3] == 0x0)) { return false; }
    minutes = _buf[4];
    return true;
}

bool Sds011::query_data(int& pm25, int& pm10) {
    _send_cmd(CMD_QUERY_DATA, NULL, 0);
    bool ok = _read_response(CMD_QUERY_DATA);
    if (!ok || !crc_ok()) {
        return false;
    }

    pm25 = _buf[2] | (static_cast<unsigned int>(_buf[3]) << 8);
    pm10 = _buf[4] | (static_cast<unsigned int>(_buf[5]) << 8);
    return true;
}

bool Sds011::query_data(int& pm25, int& pm10, int n) {
    int pm25_table[n];
    int pm10_table[n];

    for (int i = 0; n > 0 && i < n; ++i) {
        bool ok = query_data(pm25_table[i], pm10_table[i]);
        if (!ok) {
            --n;
            --i;
            continue;
        }
        delay(1000);
    }

    filter_data(n, pm25_table, pm10_table, pm25, pm10);
    return n > 0;
}

bool Sds011::query_data_auto(int& pm25, int& pm10) {
    bool ok = _read_response(CMD_QUERY_DATA);
    if (!ok || !crc_ok()) {
        return false;
    }

    pm25 = _buf[2] | (static_cast<unsigned int>(_buf[3]) << 8);
    pm10 = _buf[4] | (static_cast<unsigned int>(_buf[5]) << 8);
    return true;
}

bool Sds011::timeout() {
    return _timeout;
}

bool Sds011::crc_ok() {
    uint8_t crc = 0;
    for (int i = 2; i < 8; ++i) {
        crc += _buf[i];
    }
    return crc == _buf[8];
}

void Sds011::_send_cmd(enum Command cmd, const uint8_t* data, uint8_t len) {
    uint8_t i;
    uint8_t crc;

    _buf[0] = 0xAA;
    _buf[1] = 0xB4;
    _buf[2] = cmd;
    _buf[15] = 0xff;
    _buf[16] = 0xff;
    _buf[18] = 0xAB;

    crc = cmd + _buf[15] + _buf[16];

    for (i = 0; i < 12; ++i) {
        if (i < len) {
            _buf[3 + i] = data[i];
        }
        else {
            _buf[3 + i] = 0;
        }
        crc += _buf[3 + i];
    }

    _buf[17] = crc;

    _clear_responses();
    _out.write(_buf, sizeof(_buf));
}

int Sds011::_read_byte(long unsigned deadline) {
    uint32_t start = millis();
    while (!_out.available()) {
        if (millis() - start < deadline) {
            delay(1);
            continue;
        }
        _timeout = true;
        return -1;
    }

    _timeout = false;
    return _out.read();
}

void Sds011::_clear_responses() {
    _out.flush();
    auto avail = _out.available();
    while (avail-- && _out.read() >= 0) {}
}

bool Sds011::_read_response(enum Command cmd) {
    uint8_t i = 0;
    const long unsigned deadline = 1000;
    while (i < 3) {
        _buf[i] = _read_byte(deadline);
        if (timeout()) { break; }
        switch (i++) {
        case 0: if (_buf[0] != 0xAA) i = 0; break;
        case 1: if (_buf[1] != ((cmd == CMD_QUERY_DATA) ? 0xC0 : 0xC5)) i = 0; break;
        case 2: if (cmd != CMD_QUERY_DATA && _buf[2] != cmd) i = 0; break;
        }
    }
    for (i = 3; i < 10; i++) {
        if (timeout()) { break; }
        _buf[i] = _read_byte(deadline);
    }

    bool succ = !timeout() && _buf[9] == 0xAB;
    return succ;
}

String Sds011::_buf_to_string(uint8_t size) {
    String ret;
    for (int i = 0; i < size; i++) {
        if (_buf[i] < 0x10) ret += '0';
        ret += String(_buf[i], 16);
        if (7 == (i % 8)) ret += ' ';
        if (i < 18) ret += ' ';
    }
    return ret;
}

bool Sds011::filter_data(int n, const int* pm25_table, const int* pm10_table, int& pm25, int& pm10) {
    if (n < 1) return false;

    int pm25_min, pm25_max, pm10_min, pm10_max, pm25_sum, pm10_sum;

    pm10_sum = pm10_min = pm10_max = pm10_table[0];
    pm25_sum = pm25_min = pm25_max = pm25_table[0];

    for (int i = 1; i < n; ++i) {
        if (pm10_table[i] < pm10_min) {
            pm10_min = pm10_table[i];
        }
        if (pm10_table[i] > pm10_max) {
            pm10_max = pm10_table[i];
        }
        if (pm25_table[i] < pm25_min) {
            pm25_min = pm25_table[i];
        }
        if (pm25_table[i] > pm25_max) {
            pm25_max = pm25_table[i];
        }
        pm10_sum += pm10_table[i];
        pm25_sum += pm25_table[i];
    }

    if (n > 2) {
        pm10 = (pm10_sum - pm10_max - pm10_min) / (n - 2);
        pm25 = (pm25_sum - pm25_max - pm25_min) / (n - 2);
    }
    else if (n > 1) {
        pm10 = (pm10_sum - pm10_min) / (n - 1);
        pm25 = (pm25_sum - pm25_min) / (n - 1);
    }
    else {
        pm10 = pm10_sum / n;
        pm25 = pm25_sum / n;
    }

    return true;
}

bool Sds011Async_Base::query_data_auto_async(int n, int* pm25_table, int* pm10_table) {
    if (QDA_OFF != query_data_auto_state) return false;
    query_data_auto_n = n;
    query_data_auto_pm25_ptr = pm25_table;
    query_data_auto_pm10_ptr = pm10_table;
    query_data_auto_collected = 0;

    query_data_auto_state = QDA_WAITCOLLECT;
    onReceive([this](int avail) {
        int estimatedMsgCnt = avail / 10;
        int pm25;
        int pm10;
        int dataAutoCnt = 0;
        while (estimatedMsgCnt--) if (query_data_auto(pm25, pm10)) {
            ++dataAutoCnt;
        }
        // estimate 1s cutting into rampup per data_auto msg
        if (dataAutoCnt > 0) {
            --dataAutoCnt;

            query_data_auto_state = QDA_RAMPUP;
            query_data_auto_start = millis();
            query_data_auto_deadline = (rampup_s - dataAutoCnt) * 1000UL;
            onReceive([this](int avail) {
                unsigned long deadlineExpired = millis() - query_data_auto_start;
                if (deadlineExpired < query_data_auto_deadline) {
                    _get_out().flush();
                    return;
                }
                int pm25;
                int pm10;
                // discard estimated msgs prior to deadline expiration
                while (avail >= 10 && deadlineExpired - query_data_auto_deadline >= 1000UL) {
                    avail -= 10;
                    if (query_data_auto(pm25, pm10)) deadlineExpired -= 1000UL;
                }

                query_data_auto_state = QDA_COLLECTING;
                query_data_auto_start = millis();
                query_data_auto_deadline = 1000UL / 4UL * rampup_s;
                onReceive([this](int avail) {
                    int pm25;
                    int pm10;
                    while (avail >= 10 && query_data_auto_collected < query_data_auto_n) {
                        avail -= 10;
                        if (query_data_auto(pm25, pm10)) {
                            *query_data_auto_pm25_ptr++ = pm25;
                            *query_data_auto_pm10_ptr++ = pm10;
                            ++query_data_auto_collected;
                        }
                        query_data_auto_start = millis();
                    }
                    if (query_data_auto_collected >= query_data_auto_n) {
                        if (query_data_auto_handler) query_data_auto_handler(query_data_auto_collected);
                        query_data_auto_handler = nullptr;
                        query_data_auto_state = QDA_OFF;
                        query_data_auto_pm25_ptr = 0;
                        query_data_auto_pm10_ptr = 0;
                        query_data_auto_collected = 0;
                        onReceive(nullptr);
                    }
                    });
                });
        }
        });
    return true;
}

void Sds011Async_Base::perform_work_query_data_auto() {
    // check if collecting deadline has expired
    if (QDA_COLLECTING == query_data_auto_state &&
        millis() - query_data_auto_start > query_data_auto_deadline) {
        if (query_data_auto_handler) query_data_auto_handler(query_data_auto_collected);
        query_data_auto_handler = nullptr;
        query_data_auto_state = QDA_OFF;
        query_data_auto_pm25_ptr = 0;
        query_data_auto_pm10_ptr = 0;
        query_data_auto_collected = 0;
        onReceive(nullptr);
    }
}
