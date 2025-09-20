#include <Arduino.h>

// TLV Parser Class
class TLVParser;

// TLV Structure Definition
typedef struct {
    uint8_t type;
    uint16_t length;
    uint8_t* value;
} tlv_t;

// Example TLV types (you can define your own)
enum TLVTypes {
    TLV_TYPE_SET_TARGET_ANGLE = 0x01,
    TLV_TYPE_SET_PRINCIPLE    = 0x02,
    TLV_TYPE_SET_INTEGRAL     = 0x03,
    TLV_TYPE_SET_DERIVITIVE   = 0x04,
    TLV_TYPE_SET_DEADBAND     = 0x05,
    TLV_TYPE_SET_BASESPEED    = 0x06,
    TLV_TYPE_GET_TARGET_ANGLE = 0x07,
    TLV_TYPE_GET_PRINCIPLE    = 0x08,
    TLV_TYPE_GET_INTEGRAL     = 0x09,
    TLV_TYPE_GET_DERIVITIVE   = 0x0A,
    TLV_TYPE_GET_DEADBAND     = 0x0B,
    TLV_TYPE_GET_BASESPEED    = 0x0C,
    TLV_TYPE_CURRENT_ANGLE    = 0x0D,
    TLV_TYPE_STOP             = 0x0E,
};
