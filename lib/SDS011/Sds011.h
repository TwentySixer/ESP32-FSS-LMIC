/*
Sds011.h

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

#ifndef _SDS011_H
#define _SDS011_H

#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

#include <Stream.h>
#include <circular_queue/Delegate.h>

class Sds011 {
public:
    enum Report_mode {
        REPORT_ACTIVE = 0,
        REPORT_QUERY = 1
    };

    Sds011(Stream& out) : _out(out) {
    }

    bool device_info(String& firmware_version, uint16_t& device_id);
    bool set_data_reporting_mode(Report_mode mode);
    bool get_data_reporting_mode(Report_mode& mode);
    bool set_device_id(uint16_t device_id);
    bool set_sleep(bool sleep);
    bool get_sleep(bool& sleep);
    bool set_working_period(uint8_t minutes);
    bool get_working_period(uint8_t& minutes);
    bool set_data_rampup(int secs) {
        rampup_s = secs;
        return true;
    }
    bool get_data_rampup(int& secs) {
        secs = rampup_s;
        return true;
    }
    bool query_data(int& pm25, int& pm10);
    bool query_data(int& pm25, int& pm10, int n);
    bool query_data_auto(int& pm25, int& pm10);
    bool timeout();
    bool crc_ok();

    bool filter_data(int n, const int* pm25_table, const int* pm10_table, int& pm25, int& pm10);

protected:
    enum Command {
        CMD_DATA_REPORTING_MODE = 2,
        CMD_QUERY_DATA = 4,
        CMD_SET_DEVICE_ID = 5,
        CMD_SLEEP_AND_WORK = 6,
        CMD_FIRMWARE_VERSION = 7,
        CMD_WORKING_PERIOD = 8
    };

    void _send_cmd(enum Command cmd, const uint8_t* buf, uint8_t len);
    int _read_byte(long unsigned deadline);
    String _buf_to_string(uint8_t size);
    void _clear_responses();
    bool _read_response(enum Command cmd);

    Stream& _out;
    uint8_t _buf[19];
    bool _timeout = false;

    unsigned rampup_s = 10;
};

class Sds011Async_Base : public Sds011 {
public:
    Sds011Async_Base(Stream& out) : Sds011(out) {
    }
    Sds011Async_Base(const Sds011Async_Base&) = delete;
    Sds011Async_Base& operator= (const Sds011Async_Base&) = delete;

    virtual void perform_work() {
        perform_work_query_data_auto();
    }

    // Starts collecting up to n contiguous measurements.
    // Stops measurement early if no data arrives during rampup / 4 interval.
    // Reports measurement entries into the provided tables through ...completed
    // event handler.
    bool query_data_auto_async(int n, int* pm25_table, int* pm10_table);

    void on_query_data_auto_completed(Delegate<void(int n), void*> handler) {
        query_data_auto_handler = handler;
    }

protected:
    Stream& _get_out() { return _out; }
    virtual void onReceive(Delegate<void(int available), void*> handler) = 0;

    void perform_work_query_data_auto();

    Delegate<void(int n), void*> query_data_auto_handler;

    enum QueryDataAutoState { QDA_OFF, QDA_WAITCOLLECT, QDA_RAMPUP, QDA_COLLECTING };
    QueryDataAutoState query_data_auto_state = QDA_OFF;
    uint32_t query_data_auto_start;
    uint32_t query_data_auto_deadline;
    int query_data_auto_n = 0;
    int* query_data_auto_pm25_ptr = 0;
    int* query_data_auto_pm10_ptr = 0;
    int query_data_auto_collected = 0;
};

template< class S > class Sds011Async : public Sds011Async_Base {
    static_assert(std::is_base_of<Stream, S>::value, "S must derive from Stream");
public:
    Sds011Async(S& out) : Sds011Async_Base(out) {
    }

    void perform_work() override {
        _get_out().perform_work();
        Sds011Async_Base::perform_work();
    }

protected:
    S& _get_out() { return static_cast<S&>(_out); }
    void onReceive(Delegate<void(int available), void*> handler) override {
        _get_out().onReceive(handler);
    }
};

template<> class Sds011Async<HardwareSerial> : public Sds011Async_Base {
public:
    Sds011Async(HardwareSerial& out) : Sds011Async_Base(out) {
    }

    void perform_work() override {
        if (receiveHandler) {
            int avail = _get_out().available();
            if (avail) { receiveHandler(avail); }
        }
        Sds011Async_Base::perform_work();
    }

protected:
    HardwareSerial& _get_out() { return static_cast<HardwareSerial&>(_out); }
    void onReceive(Delegate<void(int available), void*> handler) override {
        receiveHandler = handler;
    }

    Delegate<void(int available), void*> receiveHandler;
};

#endif
