#include <Wire.h>
#include "I2CDev.h"
#include "RTC.h"

#define DS1338 0x68
#define MCP79411 0x6F
#define PCF8563 0x51

static uint8_t BCDtoDEC(uint8_t n) {
    return ((n >> 4)  * 10) + (n & 0x0F);
}

static uint8_t DECtoBCD(uint8_t n) {
    return (((n / 10) << 4) & 0xF0) | ((uint8_t)(n % 10) & 0x0F);
}

static uint8_t type = DS1338;

bool RTC_init() {
    if (CheckI2CDevice(DS1338)) {
        type = DS1338;
        // Serial.println("RTC chip found DS1338");
    } else if (CheckI2CDevice(MCP79411)) {
        type = MCP79411;
        // Serial.println("RTC chip found MCP79411");
    } else if (CheckI2CDevice(PCF8563)) {
        type = PCF8563;
        // Serial.println("RTC chip found PCF8563");
    } else {
        // Serial.println("Error, Not found RTC device");
        return false;
    }

    if (type == DS1338) {
        if (I2CMemWrite(type, 0x07, 0x00) != 0) {
            return false;
        }
    }

    if (type == MCP79411) {
        uint8_t rtcsec = 0;
        if (I2CMemRead(type, 0x00, &rtcsec) != 0) {
            return false;
        }
        // Serial.println((byte) rtcsec, HEX);
        
        if ((rtcsec & 0x80) == 0) { // ST flag is 0
            Serial.printf("Set start flag 0x%02x\n", rtcsec);
            if (I2CMemWrite(MCP79411, 0x00, rtcsec | 0x80) != 0) {
                return false;
            }
        }

        if (I2CMemWrite(type, 0x07, 0x00) != 0) { // Write 0 to EXTOSC flag, Disable external 32.768 kHz input
            return false;
        }
    }

    if (type == PCF8563) {
        if (I2CMemWrite(type, 0x00, 0x00) != 0) { // Write 0 to Oscillator Stop Flag for start
            return false;
        }
    }

    return true;
}

bool RTC_read(struct tm* timeinfo) {
    if (!timeinfo) {
        return false;
    }

    uint8_t buff[7];
    if (I2CMemRead(type, type == PCF8563 ? 0x02 : 0x00, buff, 7) != 0) {
        return false;
    }

    timeinfo->tm_sec = BCDtoDEC(buff[0] & 0x7F);
    timeinfo->tm_min = BCDtoDEC(buff[1] & 0x7F);
    timeinfo->tm_hour = BCDtoDEC(buff[2] & 0x3F);
    if (type == PCF8563) {
        timeinfo->tm_mday = BCDtoDEC(buff[3] & 0x3F);
        timeinfo->tm_wday = BCDtoDEC(buff[4] & 0x07);
    } else {
        timeinfo->tm_wday = BCDtoDEC(buff[3] & 0x07);
        timeinfo->tm_mday = BCDtoDEC(buff[4] & 0x3F);
    }
    timeinfo->tm_mon = BCDtoDEC(buff[5] & 0x1F);
    timeinfo->tm_year = BCDtoDEC(buff[6]) + 2000 - 1900;

    return true;
}

bool RTC_write(struct tm* timeinfo) {
    uint8_t buff[7];
    buff[0] = DECtoBCD(timeinfo->tm_sec) & 0x7F;
    if (type == MCP79411) {
        buff[0] |= 0x80;
    }
    buff[1] = DECtoBCD(timeinfo->tm_min) & 0x7F;
    buff[2] = DECtoBCD(timeinfo->tm_hour) & 0x3F;
    if (type == PCF8563) {
        buff[3] = DECtoBCD(timeinfo->tm_mday) & 0x3F;
        buff[4] = DECtoBCD(timeinfo->tm_wday) & 0x07;
    } else {
        buff[3] = DECtoBCD(timeinfo->tm_wday) & 0x07;
        if (type == MCP79411) {
            buff[3] |= (1 << 3); // Set VBATEN flag
        }
        buff[4] = DECtoBCD(timeinfo->tm_mday) & 0x3F;
    }
    buff[5] = DECtoBCD(timeinfo->tm_mon) & 0x1F;
    buff[6] = DECtoBCD((timeinfo->tm_year + 1900) % 100);

    if (I2CMemWrite(type, type == PCF8563 ? 0x02 : 0x00, buff, 7) != 0) {
        return false;
    }
    
    return true;
}
