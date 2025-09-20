#include <Arduino.h>

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

// TLV Parser Class
class TLVParser {
private:
    static const size_t MAX_BUFFER_SIZE = 1024;
    static const size_t MAX_TLV_COUNT = 32;
    
    uint8_t buffer[MAX_BUFFER_SIZE];
    size_t buffer_pos;
    tlv_t tlv_entries[MAX_TLV_COUNT];
    size_t tlv_count;
    
    // Utility function to convert float to bytes
    void floatToBytes(float value, uint8_t* bytes, bool big_endian = true) {
        union {
            float f;
            uint32_t i;
        } converter;
        
        converter.f = value;
        
        if (big_endian) {
            bytes[0] = (converter.i >> 24) & 0xFF;
            bytes[1] = (converter.i >> 16) & 0xFF;
            bytes[2] = (converter.i >> 8) & 0xFF;
            bytes[3] = converter.i & 0xFF;
        } else {
            bytes[3] = (converter.i >> 24) & 0xFF;
            bytes[2] = (converter.i >> 16) & 0xFF;
            bytes[1] = (converter.i >> 8) & 0xFF;
            bytes[0] = converter.i & 0xFF;
        }
    }
    
public:
    TLVParser() : buffer_pos(0), tlv_count(0) {
        memset(buffer, 0, MAX_BUFFER_SIZE);
        memset(tlv_entries, 0, sizeof(tlv_entries));
    }
    
    // Parse TLV data from buffer
    bool parseTLV(const uint8_t* data, size_t data_len) {
        tlv_count = 0;
        size_t pos = 0;
        
        while (pos < data_len && tlv_count < MAX_TLV_COUNT) {
            // Check if we have enough bytes for Type and Length
            if (pos + 3 > data_len) {
                Serial.println("Error: Incomplete TLV header");
                return false;
            }
            
            // Parse Type (1 byte)
            uint8_t type = data[pos++];
            
            // Parse Length (2 bytes, big-endian)
            uint16_t length = (data[pos] << 8) | data[pos + 1];
            pos += 2;
            
            // Check if we have enough bytes for the Value
            if (pos + length > data_len) {
                Serial.println("Error: Incomplete TLV value");
                return false;
            }
            
            // Store TLV entry
            tlv_entries[tlv_count].type = type;
            tlv_entries[tlv_count].length = length;
            tlv_entries[tlv_count].value = (uint8_t*)&data[pos];
            
            pos += length;
            tlv_count++;
        }
        
        return true;
    }
    
    // Read TLV data from Serial
    bool readTLVFromSerial() {
        if (!Serial.available()) {
            return false;
        }
        
        // Read data into buffer
        size_t bytes_read = Serial.readBytes(buffer + buffer_pos, 
                                           MAX_BUFFER_SIZE - buffer_pos);
        buffer_pos += bytes_read;
        
        // Try to parse complete TLV structures
        size_t processed = 0;
        while (processed < buffer_pos) {
            // Check if we have at least a TLV header
            if (buffer_pos - processed < 3) {
                break;
            }
            
            // Get length of current TLV
            uint16_t tlv_length = (buffer[processed + 1] << 8) | buffer[processed + 2];
            size_t total_tlv_size = 3 + tlv_length; // Type + Length + Value
            
            // Check if we have the complete TLV
            if (processed + total_tlv_size > buffer_pos) {
                break;
            }
            
            // Parse this TLV
            if (!parseTLV(buffer + processed, total_tlv_size)) {
                Serial.println("Error parsing TLV");
                return false;
            }
            
            processed += total_tlv_size;
        }
        
        // Move remaining data to beginning of buffer
        if (processed < buffer_pos) {
            memmove(buffer, buffer + processed, buffer_pos - processed);
            buffer_pos -= processed;
        } else {
            buffer_pos = 0;
        }
        
        return tlv_count > 0;
    }
    
    // Get number of parsed TLV entries
    size_t getTLVCount() const {
        return tlv_count;
    }
    
    // Get TLV entry by index
    const tlv_t* getTLV(size_t index) const {
        if (index < tlv_count) {
            return &tlv_entries[index];
        }
        return nullptr;
    }
    
    // Find TLV by type
    const tlv_t* findTLVByType(uint8_t type) const {
        for (size_t i = 0; i < tlv_count; i++) {
            if (tlv_entries[i].type == type) {
                return &tlv_entries[i];
            }
        }
        return nullptr;
    }
    
    // Print all parsed TLVs
    void printTLVs() const {
        Serial.printf("Found %d TLV entries:\n", tlv_count);
        for (size_t i = 0; i < tlv_count; i++) {
            Serial.printf("TLV %d: Type=0x%02X, Length=%d, Value=", 
                         i, tlv_entries[i].type, tlv_entries[i].length);
            
            // Print value as hex
            for (uint16_t j = 0; j < tlv_entries[i].length; j++) {
                Serial.printf("%02X ", tlv_entries[i].value[j]);
            }
            Serial.println();
        }
    }
    
    // Print TLV value as string (if it's text data)
    void printTLVAsString(size_t index) const {
        if (index < tlv_count) {
            const tlv_t* tlv = &tlv_entries[index];
            Serial.printf("TLV %d as string: ", index);
            for (uint16_t i = 0; i < tlv->length; i++) {
                if (isprint(tlv->value[i])) {
                    Serial.print((char)tlv->value[i]);
                } else {
                    Serial.print('.');
                }
            }
            Serial.println();
        }
    }
    
    // Get TLV value as integer (for numeric data)
    bool getTLVAsInt(uint8_t type, uint32_t& result) const {
        const tlv_t* tlv = findTLVByType(type);
        if (tlv == nullptr) {
            return false;
        }
        
        result = 0;
        for (uint16_t i = 0; i < tlv->length && i < 4; i++) {
            result = (result << 8) | tlv->value[i];
        }
        return true;
    }
    
    // Get TLV value as float (IEEE 754 single precision)
    bool getTLVAsFloat(uint8_t type, float& result, bool big_endian = true) const {
        const tlv_t* tlv = findTLVByType(type);
        if (tlv == nullptr || tlv->length != 4) {
            return false;
        }
        
        union {
            uint32_t i;
            float f;
        } converter;
        
        if (big_endian) {
            // Big-endian byte order (network byte order)
            converter.i = (tlv->value[0] << 24) | 
                         (tlv->value[1] << 16) | 
                         (tlv->value[2] << 8) | 
                         tlv->value[3];
        } else {
            // Little-endian byte order
            converter.i = (tlv->value[3] << 24) | 
                         (tlv->value[2] << 16) | 
                         (tlv->value[1] << 8) | 
                         tlv->value[0];
        }
        
        result = converter.f;
        return true;
    }
    
    // Get TLV value as double (IEEE 754 double precision)
    bool getTLVAsDouble(uint8_t type, double& result, bool big_endian = true) const {
        const tlv_t* tlv = findTLVByType(type);
        if (tlv == nullptr || tlv->length != 8) {
            return false;
        }
        
        union {
            uint64_t i;
            double d;
        } converter;
        
        converter.i = 0;
        if (big_endian) {
            // Big-endian byte order
            for (int i = 0; i < 8; i++) {
                converter.i = (converter.i << 8) | tlv->value[i];
            }
        } else {
            // Little-endian byte order
            for (int i = 7; i >= 0; i--) {
                converter.i = (converter.i << 8) | tlv->value[i];
            }
        }
        
        result = converter.d;
        return true;
    }
    
    // Get TLV value as float from string representation
    bool getTLVAsFloatFromString(uint8_t type, float& result) const {
        const tlv_t* tlv = findTLVByType(type);
        if (tlv == nullptr) {
            return false;
        }
        
        // Create null-terminated string
        char temp_str[tlv->length + 1];
        memcpy(temp_str, tlv->value, tlv->length);
        temp_str[tlv->length] = '\0';
        
        char* endptr;
        result = strtof(temp_str, &endptr);
        
        // Check if conversion was successful
        return (endptr != temp_str);
    }
    
    // Clear buffer and reset parser
    void reset() {
        buffer_pos = 0;
        tlv_count = 0;
        memset(buffer, 0, MAX_BUFFER_SIZE);
    }
    
    // Send a TLV to Serial
    void sendTLV(uint8_t type, uint16_t length, const uint8_t* value) {
        // Send Type (1 byte)
        Serial.write(type);
        
        // Send Length (2 bytes, big-endian)
        Serial.write((length >> 8) & 0xFF);
        Serial.write(length & 0xFF);
        
        // Send Value
        Serial.write(value, length);
    }
    
    // Send integer as TLV (1, 2, or 4 bytes)
    void sendIntTLV(uint8_t type, uint32_t value, uint8_t bytes = 4) {
        uint8_t data[4];
        
        if (bytes > 4) bytes = 4;
        
        // Convert to big-endian
        for (int i = bytes - 1; i >= 0; i--) {
            data[bytes - 1 - i] = (value >> (i * 8)) & 0xFF;
        }
        
        sendTLV(type, bytes, data);
    }
    
    // Send float as TLV (IEEE 754, 4 bytes)
    void sendFloatTLV(uint8_t type, float value, bool big_endian = true) {
        uint8_t data[4];
        floatToBytes(value, data, big_endian);
        sendTLV(type, 4, data);
    }
    
    // Send double as TLV (IEEE 754, 8 bytes)
    void sendDoubleTLV(uint8_t type, double value, bool big_endian = true) {
        union {
            double d;
            uint64_t i;
        } converter;
        
        converter.d = value;
        uint8_t data[8];
        
        if (big_endian) {
            for (int i = 0; i < 8; i++) {
                data[i] = (converter.i >> ((7 - i) * 8)) & 0xFF;
            }
        } else {
            for (int i = 0; i < 8; i++) {
                data[i] = (converter.i >> (i * 8)) & 0xFF;
            }
        }
        
        sendTLV(type, 8, data);
    }
    
    // Send string as TLV
    void sendStringTLV(uint8_t type, const char* str) {
        uint16_t length = strlen(str);
        sendTLV(type, length, (const uint8_t*)str);
    }
    
    // Send binary data as TLV
    void sendBinaryTLV(uint8_t type, const uint8_t* data, uint16_t length) {
        sendTLV(type, length, data);
    }
    
    // Print TLV in hex format (for debugging)
    void printTLVHex(uint8_t type, uint16_t length, const uint8_t* value) {
        Serial.printf("TLV Hex: %02X %02X %02X ", type, (length >> 8) & 0xFF, length & 0xFF);
        for (uint16_t i = 0; i < length; i++) {
            Serial.printf("%02X ", value[i]);
        }
        Serial.println();
    }
};
