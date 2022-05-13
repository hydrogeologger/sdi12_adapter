#include <SDI12Slave.h>
#include <SDI12CRC.h>
#include <Wire.h>

/* Sensor Adapter Details, overwrite the default values */
#define SDI12SENSOR_SDI12_PROTOCOL "14"  // Respresent v1.4
#define SDI12SENSOR_COMPANY "UQGEC___"  // 8 Charactors depicting company name
#define SDI12SENSOR_MODEL "ADA001"  // 6 Characters specifying sensor model
#define SDI12SENSOR_VERSION "1.0"  // 3 characters specifying sensor version
#define SDI12SENSOR_OTHER_INFO "SDI12ADAPTER"  // (optional) up to 13 char for serial or other sensor info

#include <SDI12Sensor.h>

#define SDI12_PIN 2  /*!< The pin of the SDI-12 data bus */
// TODO: Define eeprom address to use
/*!< EEPROM location to store sensor address (or -1 if not storing sensor address in eeprom) */
#define EEPROM_ADDR -1
#define SENSOR_ADDRESS '0' /*!< Sensor address to initialize with if not using eeprom address */

/* Pins used to determine adapter mode setup */
#define UART_MODE_PIN 8
#define ANALOG_MODE_PIN 9

/* Pins used for logging
* UART Mode: Hardware serial pins
* Analog Mode: A4 and A5
* I2C Mode: A4 (SDA) and A5 (SCL)
*/

/* Analog Settings */
#define ANALOG_IN_1_PIN A4
#define ANALOG_IN_2_PIN A5

#define SERIAL_DEFAULT_BAUDRATE 9600

/* Set up pre determined array size for data storage */
#define MEASUREMENT_ARRAY_MAX_SIZE 9 // Max size of floats/double array to hold sensor data
#define DATA_OUT_BUFFER_ARR_SIZE 10 // Max number of array elements for 0Dx! string response

typedef enum DeviceMode_e: uint8_t {
    kModeAnalog = 1,
    kModeUART = 2,
    kModeI2C = 3
} DeviceMode_e;

// Create object by which to communicate with the SDI-12 bus on SDIPIN
SDI12Slave slaveSDI12(SDI12_PIN);
SDI12Sensor sensor(SENSOR_ADDRESS, EEPROM_ADDR);
DeviceMode_e mode = kModeAnalog; // Set default to analog

/**
 * @brief Ingests a command from an SDI-12 master, sends the applicable response, and
 * (when applicable) sets a flag to initiate a measurement
 * 
 * @param[in] command SDI-12 message received from master for parsing
 * @param[out] parsed_cmd Object in memory to store command set structure
 */
void parseSdi12Cmd(const String command, SDI12CommandSet_s *parsed_cmd) {
    // First char of command is always either (a) the address of the device being
    // probed OR (b) a '?' for address query.
    // Do nothing if this command is addressed to a different device
    *parsed_cmd = SDI12Sensor::ParseCommand(command.c_str());
    if (parsed_cmd->address == sensor.Address()) {
        sensor.SetActive();
    } else if (parsed_cmd->primary == kAddressQuery) {
        sensor.SetActive();
        return;
    } else {
        return;
    }

    SDI12Sensor::GetActive()->ConfigureState(*parsed_cmd);

    // If execution reaches this point, the slave should respond with something in
    // the form:   <address><responseStr><Carriage Return><Line Feed>
    // The following if-switch-case block determines what to put into <responseStr>,
    // and the full response will be constructed afterward. For '?!' (address query)
    // or 'a!' (acknowledge active) commands, responseStr is blank so section is skipped
    String responseStr = "";
    responseStr = SDI12Sensor::GetActive()->Address();
    // Only perform some basic sensor operations here, i.e aI! and a!
    if (parsed_cmd->primary == kIdentify && parsed_cmd->secondary == kUnknown) {
        // Identify command
        // Slave should respond with ID message: 2-char SDI-12 version + 8-char
        // company name + 6-char sensor model + 3-char sensor version + 0-13
        // char S/N
        responseStr += SDI12SENSOR_SDI12_PROTOCOL \
                SDI12SENSOR_COMPANY \
                SDI12SENSOR_MODEL \
                SDI12SENSOR_VERSION \
                SDI12SENSOR_OTHER_INFO;
        SDI12Sensor::ClearActive();
    } else if (parsed_cmd->primary == kAcknowledge) {
        SDI12Sensor::ClearActive();
    } else if (parsed_cmd->primary == kAddressChange) {
        // Change address command
        // Slave should respond with blank message (just the [new] address +
        // <CR> + <LF>)
        SDI12Sensor::GetActive()->SetAddress(parsed_cmd->param1);
        responseStr = SDI12Sensor::GetActive()->Address();
        SDI12Sensor::ClearActive();
    } else if (parsed_cmd->primary == kUnknown) {
        // For DEBUG
        responseStr += "UNK\r\n";
        SDI12Sensor::ClearActive();
    }

    if (!SDI12Sensor::IsActiveSet()) {
        responseStr += "\r\n";
        slaveSDI12.sendResponse(responseStr);
    }
}

/**
 * @brief Ingests an array of floats and produces Strings in SDI-12 output format
 * 
 * @param[in] measurementValues Array of measurements in memory for parsing
 * @param[out] data_values Buffer array in memory to store data
 * @param[in] max_char_size Maximum size allowed for data string (exclude null terminator)
 * @param[in] measurement_count (Optional) Number of values to parse
 */
void formatOutputSDI(const float *measurementValues, String *data_values, const uint8_t max_char_size, uint8_t measurement_count = 0) {
    if (measurement_count <= 0) {
        measurement_count = sizeof(*measurementValues) / sizeof(char*);
    }
    uint8_t data_values_size = sizeof(*data_values) / sizeof(char*);

    *data_values = "";
    uint8_t data_values_index = 0; // Index location of data_values array
    char data_buffer[(SDI12_VALUE_STR_SIZE + 1)] = ""; // Temp string buffer + null terminator
    uint8_t len_data = 0; // Storage for string length of digit to alpha conversion

    // upper limit on i should be number of elements in measurementValues
    for (uint8_t i = 0; i < measurement_count; i++) {
        // Read float value "i" as a String with 6 deceimal digits
        // (NOTE: SDI-12 specifies max of 7 digits per value; we can only use 6
        //  decimal place precision if integer part is one digit)
        len_data = dtoa(measurementValues[i], data_buffer, 6, SDI12_VALUE_STR_SIZE);
        // Append data_values[j] if it will not exceed 35 (aM!) or 75 (aC!) characters
        if (data_values[data_values_index].length() + len_data < max_char_size) {
            data_values[data_values_index] += data_buffer;
        } else {
            // Start a new data_values "line" if appending would exceed 35/75 characters
            data_values[++data_values_index] = data_buffer;
        }
    }

    // Fill rest of data_values with blank strings
    while (data_values_index < data_values_size) {
        data_values[++data_values_index] = "";
    }
}


void setup() {
    pinMode(UART_MODE_PIN, INPUT);
    pinMode(ANALOG_MODE_PIN, INPUT);
    slaveSDI12.begin();
    delay(500);
    if (digitalRead(ANALOG_MODE_PIN) == HIGH) {
        mode = kModeAnalog;
    } else if (digitalRead(UART_MODE_PIN) == HIGH) {
        mode = kModeUART;
    } else {
        mode = kModeI2C;
    }

    switch (mode) {
        case kModeAnalog:
            pinMode(ANALOG_IN_1_PIN, INPUT);
            pinMode(ANALOG_IN_2_PIN, INPUT);
            break;
        case kModeUART:
            Serial.begin(SERIAL_DEFAULT_BAUDRATE);
            break;
        case kModeI2C:
            Wire.begin();
            break;
        default:
            // Fall back and analog mode
            pinMode(ANALOG_IN_1_PIN, INPUT);
            pinMode(ANALOG_IN_2_PIN, INPUT);
            break;
    }
    slaveSDI12.forceListen();  // sets SDIPIN as input to prepare for incoming message
}

void loop() {
    static float measurementValues[MEASUREMENT_ARRAY_MAX_SIZE];  // floats to hold sensor data
    static String data_out_buffer_arr[DATA_OUT_BUFFER_ARR_SIZE];  // String objects to hold the responses to aD0!-aD9! commands
    static String commandReceived = "";  // String object to hold the incoming command
    SDI12CommandSet_s parsed_cmd;
    String response = "";
    uint8_t measurement_count = 0;


    // If a byte is available, an SDI message is queued up. Read in the entire message
    // before proceding.  It may be more robust to add a single character per loop()
    // iteration to a static char buffer; however, the SDI-12 spec requires a precise
    // response time, and this method is invariant to the remaining loop() contents.
    int avail = slaveSDI12.available();
    if (avail < 0) {
        // Buffer is full; clear
        slaveSDI12.clearBuffer();
    } else if (avail > 0) {
        for (int a = 0; a < avail; a++) {
            char charReceived = slaveSDI12.read();
            // Character '!' indicates the end of an SDI-12 command; if the current
            // character is '!', stop listening and respond to the command
            if (charReceived == '!') {
                // eliminate the chance of getting anything else after the '!'
                // Command string is completed; do something with it
                parseSdi12Cmd(commandReceived, &parsed_cmd);
                slaveSDI12.forceListen(); // Force listen if command is not recognized
                // Clear command string to reset for next command
                commandReceived = "";
                // '!' should be the last available character anyway, but exit the "for"
                // loop just in case there are any stray characters
                slaveSDI12.ClearLineMarkingReceived(); // Clear detected break marking
                slaveSDI12.clearBuffer();
                break;
            } else {
                // If the current character is anything but '!', it is part of the command
                // string.  Append the commandReceived String object.
                // Append command string with new character
                commandReceived += String(charReceived);
            }
        }
    }

    if (sensor.IsActive() || parsed_cmd.primary == kAddressQuery) {
        response = sensor.Address();
        switch ((SDI12SensorState_e)sensor.state_) {
            case kStateLowPower:
                break;

            case kStateReady:
                if (parsed_cmd.primary == kAddressQuery) {
                    // Do nothing for a!, response is already appropriate
                } else if (parsed_cmd.primary == kDataRequest) {
                    // For aDx!
                    if (parsed_cmd.param1 < DATA_OUT_BUFFER_ARR_SIZE) {
                        response += data_out_buffer_arr[parsed_cmd.param1];
                    }
                    // Add CRC if requested for appropriate commands
                    if (sensor.CrcRequested()) {
                        SDI12CRC crc(response.c_str());
                        response += crc.GetAscii();
                    }
                } else if (parsed_cmd.primary == kByteDataRequest) {
                    // For aDBx!
                    /* Not implemented, return respond with
                     1 byte - ascii address
                     2 byte - packet size
                     1 byte - data type
                     2 byte - crc if requested
                     */
                    slaveSDI12.sendResponse(""); // Empty send response just to send line marking
                    slaveSDI12.writeBytes(sensor.Address());
                    slaveSDI12.writeBytes((uint16_t)0); // Packet size
                    slaveSDI12.writeBytes((uint8_t)kInvalidDataType); // Data type
                    // No binary data payload to transmit
                    if (sensor.CrcRequested()) {
                        SDI12CRC crc(response.c_str()); // CRC of address
                        crc.Add((uint16_t)0); // CRC of packet size
                        crc.Add((uint8_t)kInvalidDataType); // CRC of data type
                        // No binary data payload to calcualte CRC
                        slaveSDI12.writeBytes(crc.Get()); // Write CRC to data line
                    }
                    sensor.SetActive(false); // Stop further ascii transmission
                } else if (parsed_cmd.primary == kIdentify) {
                    // For aIX_001! - aIX_999!
                    // Identify Meta Group, for measurement field information,
                    // max length is 75 not including crc and <CR><LF>
                    /*
                     Response should be in the following format ,<field1>,<field2>,<additional>;
                     deliminated using ',' and ends with ';' followed by <CR><LF>
                     field1 - is ideally using SHEF codes
                     field2 - describes parameter units
                     additional (optional) - additional fields are to deliminated using ','
                    */
                    switch (mode) {
                        case kModeAnalog:
                            if (parsed_cmd.secondary == kMeasurement ||
                                    parsed_cmd.secondary == kConcurrentMeasurement) {
                                if (parsed_cmd.param1 == 0 && parsed_cmd.param2 > 0 &&
                                        parsed_cmd.param2 <= 2) {
                                    response += "ADC,unitless,10bit ADC;";
                                }
                            }
                            break;

                        case kModeUART:
                        case kModeI2C:
                            break;
                    }
                    if (parsed_cmd.secondary == kContinuousMeasurement) {
                        // Currently not implemented, add appropriate parameter
                        // condition check for response
                    } else if (parsed_cmd.secondary == kHighVolumeASCII) {
                        // Currently not implemented, add appropriate parameter
                        // condition check for response
                    } else if (parsed_cmd.secondary == kHighVolumeByte) {
                        // Currently not implemented, add appropriate parameter
                        // condition check for response
                    } else if (parsed_cmd.secondary == kVerify) {
                        // Currently not implemented, add appropriate parameter
                        // condition check for response
                    }

                    // Add CRC if requested for appropriate commands
                    if (parsed_cmd.param2 > 0 && sensor.CrcRequested()) {
                        SDI12CRC crc(response.c_str());
                        response += crc.GetAscii();
                    }
                }

                break;

            case kStateMeasurement:
                // For aM! and aMx! commands
                // Do whatever the sensor is supposed to do here
                // For this example, we will just create arbitrary "simulated" sensor data
                // NOTE: Your application might have a different data type (e.g. int) and
                //       number of values to report!
                // Response should be in following format atttn<CR><LF>
                measurement_count = 0;
                switch (mode) {
                    case kModeAnalog:
                        if (parsed_cmd.param1 == 0) {
                            measurement_count = 1;
                            response += "0021";
                            // No need to perform measurement if identification requested
                            if (parsed_cmd.primary == kIdentify) { break; }
                            response += "\r\n";
                            slaveSDI12.sendResponse(response);

                            measurementValues[0] = analogRead(ANALOG_IN_1_PIN);

                        } else if (parsed_cmd.param1 == 1) {
                            measurement_count = 1;
                            response += "0021";
                            // No need to perform measurement if identification requested
                            if (parsed_cmd.primary == kIdentify) { break; }
                            response += "\r\n";
                            slaveSDI12.sendResponse(response);

                            measurementValues[0] = analogRead(ANALOG_IN_2_PIN);

                        }
                        break;

                    case kModeUART:
                    case kModeI2C:
                        break;
                }

                if (measurement_count == 0) {
                    response += "0000";
                    // No need to perform measurement if identification requested
                    if (parsed_cmd.primary == kIdentify) { break; }
                    response += "\r\n";
                    slaveSDI12.sendResponse(response);
                    sensor.SetActive(false);
                } else if (parsed_cmd.primary == kIdentify) { break; }

                // For compliance to cancel measurement if a line break is detected
                if (slaveSDI12.LineBreakReceived() || !sensor.IsActive()) {
                    for (size_t i = 0; i < (sizeof(data_out_buffer_arr)/sizeof(*data_out_buffer_arr)); i++) {
                        data_out_buffer_arr[i] = "";
                    }
                    sensor.SetActive(false);
                } else {
                    // Populate the "data_values" String array with the values in SDI-12 format
                    formatOutputSDI(measurementValues, data_out_buffer_arr, SDI12_VALUES_STR_SIZE_35, measurement_count);
                    // For aM!, Send "service request" (<address><CR><LF>) when data is ready
                    response = sensor.Address();
                }
                slaveSDI12.ClearLineMarkingReceived();
                break;

            case kStateConcurrent:
                // For aC! and aCx! commands
                // Do whatever the sensor is supposed to do here
                // For this example, we will just create arbitrary "simulated" sensor data
                // NOTE: Your application might have a different data type (e.g. int) and
                //       number of values to report!
                // Response should be in following format atttnn<CR><LF>
                measurement_count = 0;
                switch (mode) {
                    case kModeAnalog:
                        if (parsed_cmd.param1 == 0) {
                            measurement_count = 2;
                            response += "00202";
                            if (parsed_cmd.primary == kIdentify) { break; }
                            response += "\r\n";
                            slaveSDI12.sendResponse(response);

                            measurementValues[0] = analogRead(ANALOG_IN_1_PIN);
                            measurementValues[1] = analogRead(ANALOG_IN_2_PIN);
                        }
                        break;

                    case kModeUART:
                    case kModeI2C:
                        break;
                }

                if (measurement_count == 0) {
                    response += "00000";
                    // No need to perform measurement if identification requested
                    if (parsed_cmd.primary == kIdentify) { break; }
                    response += "\r\n";
                    slaveSDI12.sendResponse(response);
                    sensor.SetActive(false);
                } else if (parsed_cmd.primary == kIdentify) { break; }

                // For compliance to cancel measurement if a correct address is detected
                for (int a = 0; a < slaveSDI12.available(); a++) {
                    char charReceived = slaveSDI12.read();
                    if (charReceived == '!') {
                        slaveSDI12.clearBuffer();
                        break;
                    } else {
                        commandReceived += charReceived;
                    }
                }
                if (!sensor.IsActive() ||
                    SDI12Sensor::ParseCommand(commandReceived.c_str()).address == sensor.Address()) {
                    for (size_t i = 0; i < (sizeof(data_out_buffer_arr)/sizeof(*data_out_buffer_arr)); i++) {
                        data_out_buffer_arr[i] = "";
                    }
                } else {
                    // Populate the "data_values" String array with the values in SDI-12 format
                    formatOutputSDI(measurementValues, data_out_buffer_arr, SDI12_VALUES_STR_SIZE_75, measurement_count);
                }
                // Sensor not expected to transmit anything at this point, make inactive
                sensor.SetActive(false);
                break;

            case kStateHighMeasurement:
                // For aHA! and aHB! commands
                // Response should be in following format atttnnn<CR><LF>

                // Do nothing, not implemented
                response += "000000";
                // No need to perform measurement if identification requested
                if (parsed_cmd.primary == kIdentify) { break; }
                response += "\r\n";
                slaveSDI12.sendResponse(response);
                sensor.SetActive(false); // Not implemented, clear data

                // For compliance to cancel measurement if a correct address is detected
                for (int a = 0; a < slaveSDI12.available(); a++) {
                    char charReceived = slaveSDI12.read();
                    if (charReceived == '!') {
                        slaveSDI12.clearBuffer();
                        break;
                    } else {
                        commandReceived += charReceived;
                    }
                }
                if (!sensor.IsActive() ||
                    SDI12Sensor::ParseCommand(commandReceived.c_str()).address == sensor.Address()) {
                    if (parsed_cmd.primary == kHighVolumeASCII) {
                        for (size_t i = 0; i < (sizeof(data_out_buffer_arr)/sizeof(*data_out_buffer_arr)); i++) {
                            data_out_buffer_arr[i] = "";
                        }
                    } else if (parsed_cmd.primary == kHighVolumeByte) {
                        // Clear data
                    }
                } else if (parsed_cmd.primary == kHighVolumeASCII) {
                    // Populate the "data_out_buffer_arr" String array with the values in SDI-12 format
                    formatOutputSDI(measurementValues, data_out_buffer_arr, SDI12_VALUES_STR_SIZE_75);
                } else if (parsed_cmd.primary == kHighVolumeByte) {
                    // Perform data storage here
                }
                // Sensor not expected to transmit anything at this point, make inactive
                sensor.SetActive(false);
                break;

            case kStateContinuous:
                // For aRx! commands
                // Data should be available and broadcasted immediately similar to aDx! commands
                // Message <values> length is limited to 75 characters long

                // Do nothing, not implemented
                switch (mode) {
                    case kModeAnalog:
                        break;
                    case kModeUART:
                        break;
                    case kModeI2C:
                        break;
                }

                // Add CRC if requested for appropriate commands
                if (sensor.CrcRequested()) {
                    SDI12CRC crc(response.c_str());
                    response += crc.GetAscii();
                }
                break;

            case kStateVerify:
                // For aV!
                // Response should be in following format atttn<CR><LF>
                // Not implemented, return respond with "00000"
                response += "0000";
                // No need to perform measurement if identification requested
                if (parsed_cmd.primary == kIdentify) { break; }
                response += "\r\n";
                slaveSDI12.sendResponse(response);
                // For aV!, Send "service request" (<address><CR><LF>) when data is ready
                // response = sensor.Address();

                // aV! does not have a way to cancel
                // Populate the "data_out_buffer_arr" String array with the values in
                // SDI-12 format up to 35 characters long max
                for (size_t i = 0; i < (sizeof(data_out_buffer_arr)/sizeof(*data_out_buffer_arr)); i++) {
                    data_out_buffer_arr[i] = "";
                }
                // Sensor not expected to transmit anything at this point, make inactive
                sensor.SetActive(false);
                break;
        }

        if (sensor.IsActive() || parsed_cmd.primary == kAddressQuery) {
            response += "\r\n";
            slaveSDI12.sendResponse(response);
            sensor.SetActive(false);
        }
        sensor.SetState(kStateReady);
        slaveSDI12.forceListen();   // sets SDI-12 pin as input to prepare for
                                    // incoming message AGAIN
    }
}
