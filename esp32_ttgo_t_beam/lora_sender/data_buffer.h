#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

//#define __DEBUG_PACKER_RSSI_ADD

#include "freertos/FreeRTOS.h"
#include "sys/time.h"
#include <stdlib.h>
#include <sstream>
#include "hex.h"
#include "mac_helper.h"

typedef struct
{
    uint8_t address[6];
    int32_t rssi_sum;
    uint32_t rssi_count;
    int mac_type;
    uint32_t tx_power_sum;
    std::string name;    
} mac_address_data;

class DataBuffer {    
    private:
        int _idx;
        int _size;
        int _counter;
        mac_address_data* _data;
        char* _tag;
        char* _this_device_mac_address;
        char* _this_device_ble_mac_address;

        mac_address_data* find(const esp_bd_addr_t* address) {
            if(!_data) return NULL;

            for(int i = 0; i < _idx; i++) {
                if(
                    _data[i].address[0] == ((uint8_t*) (address))[0] &&
                    _data[i].address[1] == ((uint8_t*) (address))[1] &&
                    _data[i].address[2] == ((uint8_t*) (address))[2] &&
                    _data[i].address[3] == ((uint8_t*) (address))[3] &&
                    _data[i].address[4] == ((uint8_t*) (address))[4] &&
                    _data[i].address[5] == ((uint8_t*) (address))[5]
                )
                    return &_data[i];
            }
            return NULL;
        }

    public:
        DataBuffer(int size, char* this_device_ble_mac_address) {                        
            _size = size;            
            _idx = 0;
            _this_device_ble_mac_address = this_device_ble_mac_address;    
            _data = new mac_address_data[_size];
        }

        void add(const esp_bd_addr_t* address, int8_t rssi, std::string name, int8_t tx_power, int mac_type) {               
            mac_address_data* mac_data = find(address);
            if(!mac_data) {
                if(_idx == _size)
                    return;
          
                for(uint8_t i = 0; i < 6; i++)
                    _data[_idx].address[i] = ((uint8_t*) (address))[i];
                _data[_idx].rssi_count = 0;
                _data[_idx].rssi_sum = 0;
                _data[_idx].tx_power_sum = 0;
                _data[_idx].name = name;
                _data[_idx].mac_type = mac_type;
                mac_data = &_data[_idx];
                _idx++;

#ifdef __DEBUG_PACKER_RSSI_ADD
                Serial.printf("new buffer size: %d\n", _idx);
#endif            
            }
            mac_data->rssi_count++;
            mac_data->rssi_sum += rssi;
            mac_data->tx_power_sum += tx_power;
            mac_data->name = name;
            mac_data->mac_type = mac_type;
            
#ifdef __DEBUG_PACKER_RSSI_ADD
            Serial.printf("ADDR=%02x:%02x:%02x:%02x:%02x:%02x MAC_TYPE=%d RSSI_AVG=%f RSSI=%d RSSI_SUM=%d RSSI_COUNT=%d TXPOWER_SUM=%d NAME=%s\n",
                            address[0], address[1], address[2], address[3], address[4], address[5],
                            mac_type,
                            (float) mac_data->rssi_sum / (float) mac_data->rssi_count,
                            rssi, 
                            mac_data->rssi_sum, 
                            mac_data->rssi_count,
                            mac_data->tx_power_sum,
                            mac_data->name);
#endif
        }

        int size() {
            return _idx;
        }    

        void flushToSerial() {
            int s = size();
            std::stringstream ss;        
            for (int i = 0; i < _idx; i++ ) {
                for(int j = 0; j < 6; j++) {
                    char buff[2];
                    make_hex_string(_data[i].address[j], buff);
                    ss << buff;
                    if(j != 5)
                        ss << ":";
                }
                ss << " MAC_TYPE " << _data[i].mac_type 
                   << " RSSI_SUM " << _data[i].rssi_sum
                   << " RSSI_COUNT " << _data[i].rssi_count 
                   << " RSSI_AVG " << (float)_data[i].rssi_sum / (float)_data[i].rssi_count
                   << " TXPOWER_SUM " << _data[i].tx_power_sum
                   << " NAME " << _data[i].name
                   << "\n";
            }
            reset();
            Serial.printf("Buffer Data: Total items = %d \n%s\n\n", s, ss.str().c_str());
        }

        void flush(char* buffer, bool clear) {
            std::stringstream ss;
            ss << /*message format version*/ "1;" << _this_device_mac_address << ";" << _this_device_ble_mac_address << ";" << _tag << ";";
            for (int i = 0; i < _idx; i++ ) {
                for(int j = 0; j < 6; j++) {
                    char buff[2];
                    make_hex_string(_data[i].address[j], buff);
                    ss << buff;
                    // if(j != 5)
                    //     ss << ":";
                }
                ss << ","                   
                   << _data[i].mac_type << ","
                   << (int)_data[i].rssi_count << ","
                   << (int)_data[i].rssi_sum << ","                   
                   << (int)_data[i].tx_power_sum << ","                   
                   << _data[i].name //TODO: validate name                   
                   << ";";
            }
            if(clear)
                reset();
            //Serial.printf("Buffer Data: \n%s\n\n", ss.str().c_str());
            strcpy(buffer, ss.str().c_str());
        }

        void reset() {
            _idx = 0;
        }
};

#endif //DATA_BUFFER_H
