#include "epos4/signals/Errors.hpp"

#include <cstdio>

namespace epos4::signals
{

namespace
{

constexpr const char * kE1000_causes[] = {
  "Unspecific error occurred",
};
constexpr const char * kE1000_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE1000_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE1080_causes[] = {
  "Critical error occurred during boot-up",
};
constexpr const char * kE1080_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE1080_recovery[] = {
  "Reset device. If the problem persists, contact your supplier.",
};

constexpr const char * kE1090_causes[] = {
  "Incompatible extension firmware version detected",
};
constexpr const char * kE1090_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE1090_recovery[] = {
  "First, try to resolve by resetting the device.",
  "If reset fails, update the EPOS4 firmware with the extension attached.",
  "If the problem persists, contact your supplier.",
};

constexpr const char * kE2310_causes[] = {
  "Short circuit in motor winding",
  "Controller gains too high and/or deceleration too high",
  "Damaged power stage",
};
constexpr const char * kE2310_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE2310_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE2320_causes[] = {
  "Short circuit of motor winding against ground",
  "Short circuit of motor winding against operating voltage Vcc",
  "Damaged power stage",
  "Strong motor ripple (on top of a high peak current draw)",
  "High deceleration or acceleration demands (which push the control to its limits)",
  "Max. peak current configured which is close to the power stage current protection level",
  "Poor current control parameter set",
  "Sudden STO input interruption or loose contact",
};
constexpr const char * kE2320_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE2320_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE3210_causes[] = {
  "Power supply voltage too high",
};
constexpr const char * kE3210_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE3210_recovery[] = {
  "In most cases this error occurs at deceleration where the motor works as a generator and the energy flows from motor to power supply (resulting in an increased voltage).",
  "Usually, a capacitor (for example 2200 F) close to the device will solve the problem. If not, a shunt regulator will be necessary to dissipate brake energy.",
  "Reset fault with Controlword (only possible if supply voltage is in valid range).",
};

constexpr const char * kE3220_causes[] = {
  "Supply voltage is too low for operation",
  "Power supply cannot supply required acceleration current",
};
constexpr const char * kE3220_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE3220_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE4210_causes[] = {
  "Temperature at device's power stage too high",
};
constexpr const char * kE4210_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE4210_recovery[] = {
  "Reset fault with Controlword (only possible if temperature is in valid range)",
};

constexpr const char * kE4380_causes[] = {
  "Temperature at motor too high or sensor not connected",
};
constexpr const char * kE4380_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE4380_recovery[] = {
  "Reset fault with Controlword (only possible if temperature is in valid range)",
};

constexpr const char * kE5113_causes[] = {
  "Logic supply voltage is too low for operation",
};
constexpr const char * kE5113_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE5113_recovery[] = {
  "Reset fault with Controlword (only possible if supply voltage is in valid range)",
};

constexpr const char * kE5280_causes[] = {
  "Hardware problem detected",
};
constexpr const char * kE5280_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE5280_recovery[] = {
  "Reset device. If the problem persists, contact your supplier.",
};

constexpr const char * kE5281_causes[] = {
  "An incompatible hardware combination was detected",
};
constexpr const char * kE5281_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE5281_recovery[] = {
  "Reset device. If the problem persists, contact your supplier.",
};

constexpr const char * kE5282_causes[] = {
  "STO card type not detected",
};
constexpr const char * kE5282_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE5282_recovery[] = {
  "Make sure that the card is fully inserted. If the problem continues, replace the card.",
};

constexpr const char * kE5480_causes[] = {
  "A hardware problem was detected",
};
constexpr const char * kE5480_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE5480_recovery[] = {
  "Reset device. If the problem persists, contact your supplier.",
};

constexpr const char * kE6080_causes[] = {
  "Problem with connection to extension 1:",
  "Overload situation",
  "Extension hardware failure",
};
constexpr const char * kE6080_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE6080_recovery[] = {
  "Reset fault with Controlword",
  "Check that extension 1 is firmly connected",
  "If problem reoccurs frequently:",
  "Update firmware",
  "Contact your supplier",
};

constexpr const char * kE6081_causes[] = {
  "Connection loss to extension 1:",
  "Overload situation",
  "Extension hardware failure",
};
constexpr const char * kE6081_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE6081_recovery[] = {
  "Reset fault with Controlword",
  "Check that extension 1 is firmly connected",
  "If problem reoccurs frequently:",
  "Update firmware",
  "Contact your supplier",
};

constexpr const char * kE6180_causes[] = {
  "An internal software error occurred",
};
constexpr const char * kE6180_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE6180_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE6320_causes[] = {
  "Corrupt parameter detected: The constant velocity phase is too long due to high Target position and low Profile velocity",
};
constexpr const char * kE6320_effects[] = {
  "Fault reaction defined in Fault reaction option code",
};
constexpr const char * kE6320_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE6380_causes[] = {
  "Persistent parameters are corrupt or inconsistent (wrong CRC)",
};
constexpr const char * kE6380_effects[] = {
  "Default parameters are set",
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE6380_recovery[] = {
  "Reset fault with Controlword",
  "Set or load device parameters again",
};

constexpr const char * kE7320_causes[] = {
  "Detected position of position sensor is no longer valid due to...",
  "changed/wrong position sensor parameters",
  "other errors that influence the absolute position detection (such as Hall Sensor Error,",
  "Position Sensor Index Error, etc.)",
};
constexpr const char * kE7320_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7320_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE7380_causes[] = {
  "Position sensor supervision has detected a bad working condition due to...",
  "wrong/broken wiring of encoder",
  "defective encoder",
  "regulation parameter are not well tuned (Current control parameter set)",
};
constexpr const char * kE7380_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7380_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE7381_causes[] = {
  "Encoder pulses counted between the first two index pulses do not fit the resolution",
  "Setting of encoder resolution is wrong",
};
constexpr const char * kE7381_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7381_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE7382_causes[] = {
  "Encoder index signal was not found within two turns at start-up due to...",
  "incorrect wiring of encoder cables",
  "encoder without or with defective index channel",
  "wrong sensor type",
  "setting for encoder resolution too low",
};
constexpr const char * kE7382_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7382_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE7388_causes[] = {
  "Motor Hall sensors report an impossible signal combination due to...",
  "incorrect wiring of Hall sensors",
  "incorrect wiring of Hall sensor supply voltage",
  "damaged Hall sensors",
  "big Hall sensor signal noise",
};
constexpr const char * kE7388_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7388_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE7389_causes[] = {
  "No Hall sensor 3 edge found within first motor turn due to...",
  "wrong wiring of Hall sensors",
  "defective Hall sensors",
  "setting for encoder resolution too low",
};
constexpr const char * kE7389_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7389_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE738A_causes[] = {
  "Angle difference measured between encoder and Hall sensors is too high due to...",
  "wrong wiring of Hall sensors",
  "defective Hall sensors",
  "wrong wiring of encoder",
  "defective encoder",
  "wrong setting of encoder resolution or pole pairs",
};
constexpr const char * kE738A_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE738A_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE738C_causes[] = {
  "SSI sensor driver could not sample position data",
};
constexpr const char * kE738C_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE738C_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE738D_causes[] = {
  "Invalid SSI sensor data frame. Start and/or stop bits have invalid state:",
  "wrong wiring of SSI sensor",
  "defective SSI sensor",
  "wrong setting of encoder data bits",
};
constexpr const char * kE738D_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE738D_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE7390_causes[] = {
  "No main sensor available. Adapt settings in Axis configuration.",
};
constexpr const char * kE7390_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7390_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE7391_causes[] = {
  "No commutation sensor available. Adapt settings in Axis configuration.",
};
constexpr const char * kE7391_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7391_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE7392_causes[] = {
  "Position sensor supervision has detected a turn-away of the motor in the opposite direction due to...",
  "wrong setting of sensor polarity",
  "wrong position sensor wiring",
  "wrong motor wiring",
};
constexpr const char * kE7392_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE7392_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8110_causes[] = {
  "One of the CAN mail boxes experienced an overflow caused by too high communication rate",
};
constexpr const char * kE8110_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8110_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8111_causes[] = {
  "Execution of CAN communication had an overrun caused by too high communication rate",
};
constexpr const char * kE8111_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8111_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8120_causes[] = {
  "Device changed to CAN passive mode due to...",
  "CAN bit rate of one CAN node in network wrong",
  "CAN network not connected",
  "hardware wiring of CAN bus not correct",
};
constexpr const char * kE8120_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8120_recovery[] = {
  "Send NMT command reset communication, then reset fault with Controlword",
};

constexpr const char * kE8130_causes[] = {
  "CANopen Heartbeat Consumer Procedure or Life Guarding have detected a timeout.",
  "Probably, the procedure has failed due to wrong configuration.",
  "Heartbeat Consumers will be disabled if Consumer heartbeat time = 0.",
};
constexpr const char * kE8130_effects[] = {
  "Fault reaction defined in Abort connection option code",
  "State transition defined in Communication error",
};
constexpr const char * kE8130_recovery[] = {
  "Send NMT command reset communication, then reset fault with Controlword",
};

constexpr const char * kE8150_causes[] = {
  "Possibly, another CAN node has configured the same transmit PDO COB-ID.",
  "Device has received a bad transmit PDO request (valid COB-ID without RTR bit set).",
};
constexpr const char * kE8150_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8150_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8180_causes[] = {
  "EtherCAT communication error during operation enable (link lost)",
};
constexpr const char * kE8180_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8180_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8181_causes[] = {
  "Initialization of the Ethernet module has failed",
};
constexpr const char * kE8181_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE8181_recovery[] = {
  "Reset device.",
  "If the problem persists, perform firmware update or contact your supplier.",
};

constexpr const char * kE8182_causes[] = {
  "The EtherCAT receive queue had an overrun caused by ...",
  "Too high communication rate",
  "Timing violation (eg. non-Real-time system)",
};
constexpr const char * kE8182_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8182_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8183_causes[] = {
  "Internal communication of the EtherCAT module has failed",
};
constexpr const char * kE8183_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8183_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8184_causes[] = {
  "EtherCAT communication error because of invalid cycle time",
};
constexpr const char * kE8184_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8184_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE81FD_causes[] = {
  "CAN controller has entered CAN bus off state",
};
constexpr const char * kE81FD_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE81FD_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE81FE_causes[] = {
  "One of the CAN receive queues had an overrun caused by too high communication rate",
};
constexpr const char * kE81FE_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE81FE_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE81FF_causes[] = {
  "One of the CAN transmit queues had an overrun caused by too high communication rate due to...",
  "load on CAN bus too high",
  "event-triggered PDOs defined with too small inhibit time",
  "PDO communication configured too high (synchronous) for actual cycle time",
  "CAN bus inactive but heartbeat producer enabled (Producer heartbeat time)",
};
constexpr const char * kE81FF_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE81FF_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8210_causes[] = {
  "Received PDO was not processed due to length error (too short)",
};
constexpr const char * kE8210_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8210_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8250_causes[] = {
  "Interpolation aborted in cyclic mode due to no PDO received after elapsed interpolation time period",
  "The error also occurs if the master aborts communication, e.g. due to timing violations of the synchronous PDO transfer",
};
constexpr const char * kE8250_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8250_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8280_causes[] = {
  "EtherCAT module detected an error at Process Data (PDO) communication",
};
constexpr const char * kE8280_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8280_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8281_causes[] = {
  "EtherCAT module detected an error at Service Data (SDO) communication",
};
constexpr const char * kE8281_effects[] = {
  "Fault reaction defined in Abort connection option code",
};
constexpr const char * kE8281_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8611_causes[] = {
  "Difference between Position demand value and Position actual value higher than Following error window",
};
constexpr const char * kE8611_effects[] = {
  "Fault reaction defined in Fault reaction option code",
};
constexpr const char * kE8611_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8A80_causes[] = {
  "Negative limit switch was/is active",
  "Wrong configuration of limit switch function in Digital inputs",
};
constexpr const char * kE8A80_effects[] = {
  "Fault reaction defined in Fault reaction option code",
};
constexpr const char * kE8A80_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8A81_causes[] = {
  "Positive limit switch was/is active",
  "Wrong configuration of limit switch function in Digital inputs",
};
constexpr const char * kE8A81_effects[] = {
  "Fault reaction defined in Fault reaction option code",
};
constexpr const char * kE8A81_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8A82_causes[] = {
  "Movement commanded or actual position runs out of software position limit",
};
constexpr const char * kE8A82_effects[] = {
  "Fault reaction defined in Fault reaction option code",
};
constexpr const char * kE8A82_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8A88_causes[] = {
  "Error when STO is not active. STO functionality was triggered while power stage was enabled",
};
constexpr const char * kE8A88_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kE8A88_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kE8A8A_causes[] = {
  "The STO card is in or has changed to ready to release state",
};
constexpr const char * kE8A8A_effects[] = {
  "Device disabled",
  "Warning flag set in Statusword",
};
constexpr const char * kE8A8A_recovery[] = {
  "Check the card and the system connections. Reset fault by clearing Diagnosis History /",
  "Error history",
};

constexpr const char * kE8A8B_causes[] = {
  "The STO card is in error or power down state",
};
constexpr const char * kE8A8B_effects[] = {
  "Device disabled",
  "RED LED on",
  "Error flag set in Statusword",
};
constexpr const char * kE8A8B_recovery[] = {
  "Check the STO signals. Reset fault with Controlword",
};

constexpr const char * kEFF01_causes[] = {
  "Device has not enough free resources to process new commands",
};
constexpr const char * kEFF01_effects[] = {
  "Warning bit set in Statusword",
};
constexpr const char * kEFF01_recovery[] = {
  "Reset fault by clearing Diagnosis History / Error history",
};

constexpr const char * kEFF02_causes[] = {
  "Device reset by watchdog occurred due to fatal system overload or system fault",
};
constexpr const char * kEFF02_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF02_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF0B_causes[] = {
  "The device has not enough free resources to provide proper regulation",
};
constexpr const char * kEFF0B_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF0B_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF10_causes[] = {
  "Control function not possible due to bad controller gains",
};
constexpr const char * kEFF10_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
  "Fault reaction defined in Fault reaction option code",
};
constexpr const char * kEFF10_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF11_causes[] = {
  "An error occurred during auto tuning identification",
};
constexpr const char * kEFF11_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF11_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF12_causes[] = {
  "Current limit occurred during auto tuning identification",
};
constexpr const char * kEFF12_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF12_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF13_causes[] = {
  "Identification current could not be reached during auto tuning identification",
};
constexpr const char * kEFF13_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF13_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF14_causes[] = {
  "Data sampling initialization has failed",
};
constexpr const char * kEFF14_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF14_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF15_causes[] = {
  "Sample data mismatched during auto tuning identification",
};
constexpr const char * kEFF15_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF15_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF16_causes[] = {
  "Wrong parameter for auto tuning identification. Error during identification auto tuning process",
};
constexpr const char * kEFF16_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF16_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF17_causes[] = {
  "Nominal actual amplitude mismatch during auto tuning identification",
};
constexpr const char * kEFF17_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF17_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF19_causes[] = {
  "Auto tuning identification timeout. Termination requirements not met",
};
constexpr const char * kEFF19_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF19_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF20_causes[] = {
  "Motor did not reach standstill during auto tuning. Make sure that the motor is not moving when starting the tuning process.",
};
constexpr const char * kEFF20_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF20_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF21_causes[] = {
  "Motor movement has been obstructed during tuning process. This might be caused e.g. by the motor being connected to a rigid gear. Ensure that the motor movement is not obstructed.",
};
constexpr const char * kEFF21_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF21_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF22_causes[] = {
  "Max system speed exceeded during auto tuning identification. Reduce step amplitude to reduce max speed.",
};
constexpr const char * kEFF22_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF22_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF23_causes[] = {
  "Identification current is very small. Check the motor connection.",
};
constexpr const char * kEFF23_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF23_recovery[] = {
  "Reset fault with Controlword",
};

constexpr const char * kEFF24_causes[] = {
  "Sensor signal was not found during tuning identification. Check the sensor and motor connections.",
};
constexpr const char * kEFF24_effects[] = {
  "Device disabled",
  "Red LED \"ON\"",
  "Error flag set in Statusword",
};
constexpr const char * kEFF24_recovery[] = {
  "Reset fault with Controlword",
};

constexpr DeviceError kDeviceErrors[] = {
  {0x0000, 0x0000, "No Error", 0x00, FaultReaction::kNone, false, nullptr, 0, nullptr, 0, nullptr,
    0},
  {0x1000, 0x1000, "Generic error", 0x01, FaultReaction::kNoSecureMovement, false, kE1000_causes, 1,
    kE1000_effects, 3, kE1000_recovery, 1},
  {0x1080, 0x1088, "Generic initialization error", 0x01, FaultReaction::kNoSecureMovement, false,
    kE1080_causes, 1, kE1080_effects, 3, kE1080_recovery, 1},
  {0x1090, 0x1090, "Firmware incompatibility error", 0x01, FaultReaction::kNoSecureMovement, false,
    kE1090_causes, 1, kE1090_effects, 3, kE1090_recovery, 3},
  {0x2310, 0x2310, "Overcurrent error", 0x02, FaultReaction::kNoSecureMovement, false,
    kE2310_causes, 3, kE2310_effects, 3, kE2310_recovery, 1},
  {0x2320, 0x2320, "Power stage protection error", 0x02, FaultReaction::kNoSecureMovement, false,
    kE2320_causes, 8, kE2320_effects, 3, kE2320_recovery, 1},
  {0x3210, 0x3210, "Overvoltage error", 0x04, FaultReaction::kNoSecureMovement, false,
    kE3210_causes, 1, kE3210_effects, 3, kE3210_recovery, 3},
  {0x3220, 0x3220, "Undervoltage error", 0x04, FaultReaction::kNoSecureMovement, false,
    kE3220_causes, 2, kE3220_effects, 3, kE3220_recovery, 1},
  {0x4210, 0x4210, "Thermal overload error", 0x08, FaultReaction::kNoSecureMovement, false,
    kE4210_causes, 1, kE4210_effects, 3, kE4210_recovery, 1},
  {0x4380, 0x4380, "Thermal motor overload error", 0x08, FaultReaction::kNoSecureMovement, false,
    kE4380_causes, 1, kE4380_effects, 3, kE4380_recovery, 1},
  {0x5113, 0x5113, "Logic supply voltage too low error", 0x04, FaultReaction::kNoSecureMovement,
    false, kE5113_causes, 1, kE5113_effects, 3, kE5113_recovery, 1},
  {0x5280, 0x5280, "Hardware defect error", 0x01, FaultReaction::kNoSecureMovement, false,
    kE5280_causes, 1, kE5280_effects, 3, kE5280_recovery, 1},
  {0x5281, 0x5281, "Hardware incompatibility error", 0x01, FaultReaction::kNoSecureMovement, false,
    kE5281_causes, 1, kE5281_effects, 3, kE5281_recovery, 1},
  {0x5282, 0x5282, "STO card detection error", 0x01, FaultReaction::kNoSecureMovement, false,
    kE5282_causes, 1, kE5282_effects, 3, kE5282_recovery, 1},
  {0x5480, 0x5483, "Hardware error", 0x01, FaultReaction::kNoSecureMovement, false, kE5480_causes,
    1, kE5480_effects, 3, kE5480_recovery, 1},
  {0x6080, 0x6080, "Sign of life error", 0x01, FaultReaction::kNoSecureMovement, false,
    kE6080_causes, 3, kE6080_effects, 3, kE6080_recovery, 5},
  {0x6081, 0x6081, "Extension 1 watchdog error", 0x01, FaultReaction::kAbortConnectionOption, false,
    kE6081_causes, 3, kE6081_effects, 3, kE6081_recovery, 5},
  {0x6180, 0x61F0, "Internal software error", 0x01, FaultReaction::kNoSecureMovement, false,
    kE6180_causes, 1, kE6180_effects, 3, kE6180_recovery, 1},
  {0x6320, 0x6320, "Software parameter error", 0x01, FaultReaction::kFaultReactionOption, false,
    kE6320_causes, 1, kE6320_effects, 1, kE6320_recovery, 1},
  {0x6380, 0x6380, "Persistent parameter corrupt error", 0x01, FaultReaction::kNoSecureMovement,
    false, kE6380_causes, 1, kE6380_effects, 4, kE6380_recovery, 2},
  {0x7320, 0x7320, "Position sensor error", 0x20, FaultReaction::kNoSecureMovement, false,
    kE7320_causes, 4, kE7320_effects, 3, kE7320_recovery, 1},
  {0x7380, 0x7380, "Position sensor breach error", 0x20, FaultReaction::kNoSecureMovement, true,
    kE7380_causes, 4, kE7380_effects, 3, kE7380_recovery, 1},
  {0x7381, 0x7381, "Position sensor resolution error", 0x20, FaultReaction::kNoSecureMovement, true,
    kE7381_causes, 2, kE7381_effects, 3, kE7381_recovery, 1},
  {0x7382, 0x7382, "Position sensor index error", 0x20, FaultReaction::kNoSecureMovement, true,
    kE7382_causes, 5, kE7382_effects, 3, kE7382_recovery, 1},
  {0x7388, 0x7388, "Hall sensor error", 0x20, FaultReaction::kNoSecureMovement, true, kE7388_causes,
    5, kE7388_effects, 3, kE7388_recovery, 1},
  {0x7389, 0x7389, "Hall sensor not found error", 0x20, FaultReaction::kNoSecureMovement, true,
    kE7389_causes, 4, kE7389_effects, 3, kE7389_recovery, 1},
  {0x738A, 0x738A, "Hall angle detection error", 0x20, FaultReaction::kNoSecureMovement, true,
    kE738A_causes, 6, kE738A_effects, 3, kE738A_recovery, 1},
  {0x738C, 0x738C, "SSI sensor error", 0x20, FaultReaction::kNoSecureMovement, false, kE738C_causes,
    1, kE738C_effects, 3, kE738C_recovery, 1},
  {0x738D, 0x738D, "SSI sensor frame error", 0x20, FaultReaction::kNoSecureMovement, false,
    kE738D_causes, 4, kE738D_effects, 3, kE738D_recovery, 1},
  {0x7390, 0x7390, "Missing main sensor error", 0x20, FaultReaction::kNoSecureMovement, false,
    kE7390_causes, 1, kE7390_effects, 3, kE7390_recovery, 1},
  {0x7391, 0x7391, "Missing commutation sensor error", 0x20, FaultReaction::kNoSecureMovement,
    false, kE7391_causes, 1, kE7391_effects, 3, kE7391_recovery, 1},
  {0x7392, 0x7392, "Main sensor direction error", 0x20, FaultReaction::kNoSecureMovement, true,
    kE7392_causes, 4, kE7392_effects, 3, kE7392_recovery, 1},
  {0x8110, 0x8110, "CAN overrun error (object lost)", 0x10, FaultReaction::kAbortConnectionOption,
    false, kE8110_causes, 1, kE8110_effects, 1, kE8110_recovery, 1},
  {0x8111, 0x8111, "CAN overrun error", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE8111_causes, 1, kE8111_effects, 1, kE8111_recovery, 1},
  {0x8120, 0x8120, "CAN passive mode error", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE8120_causes, 4, kE8120_effects, 1, kE8120_recovery, 1},
  {0x8130, 0x8130, "CAN heartbeat error", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE8130_causes, 3, kE8130_effects, 2, kE8130_recovery, 1},
  {0x8150, 0x8150, "CAN PDO COB-ID collision", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE8150_causes, 2, kE8150_effects, 1, kE8150_recovery, 1},
  {0x8180, 0x8180, "EtherCAT communication error", 0x10, FaultReaction::kAbortConnectionOption,
    false, kE8180_causes, 1, kE8180_effects, 1, kE8180_recovery, 1},
  {0x8181, 0x8181, "EtherCAT initialization error", 0x10, FaultReaction::kNoSecureMovement, false,
    kE8181_causes, 1, kE8181_effects, 3, kE8181_recovery, 2},
  {0x8182, 0x8182, "EtherCAT Rx queue overflow", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE8182_causes, 3, kE8182_effects, 1, kE8182_recovery, 1},
  {0x8183, 0x8183, "EtherCAT communication error (internal)", 0x10,
    FaultReaction::kAbortConnectionOption, false, kE8183_causes, 1, kE8183_effects, 1,
    kE8183_recovery, 1},
  {0x8184, 0x8184, "EtherCAT communication cycle time error", 0x10,
    FaultReaction::kAbortConnectionOption, false, kE8184_causes, 1, kE8184_effects, 1,
    kE8184_recovery, 1},
  {0x81FD, 0x81FD, "CAN bus turned off", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE81FD_causes, 1, kE81FD_effects, 1, kE81FD_recovery, 1},
  {0x81FE, 0x81FE, "CAN Rx queue overflow", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE81FE_causes, 1, kE81FE_effects, 1, kE81FE_recovery, 1},
  {0x81FF, 0x81FF, "CAN Tx queue overflow", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE81FF_causes, 5, kE81FF_effects, 1, kE81FF_recovery, 1},
  {0x8210, 0x8210, "CAN PDO length error", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE8210_causes, 1, kE8210_effects, 1, kE8210_recovery, 1},
  {0x8250, 0x8250, "RPDO timeout", 0x10, FaultReaction::kAbortConnectionOption, false,
    kE8250_causes, 2, kE8250_effects, 1, kE8250_recovery, 1},
  {0x8280, 0x8280, "EtherCAT PDO communication error", 0x10, FaultReaction::kAbortConnectionOption,
    false, kE8280_causes, 1, kE8280_effects, 1, kE8280_recovery, 1},
  {0x8281, 0x8281, "EtherCAT SDO communication error", 0x10, FaultReaction::kAbortConnectionOption,
    false, kE8281_causes, 1, kE8281_effects, 1, kE8281_recovery, 1},
  {0x8611, 0x8611, "Following error", 0x80, FaultReaction::kFaultReactionOption, false,
    kE8611_causes, 1, kE8611_effects, 1, kE8611_recovery, 1},
  {0x8A80, 0x8A80, "Negative limit switch error", 0x80, FaultReaction::kFaultReactionOption, false,
    kE8A80_causes, 2, kE8A80_effects, 1, kE8A80_recovery, 1},
  {0x8A81, 0x8A81, "Positive limit switch error", 0x80, FaultReaction::kFaultReactionOption, false,
    kE8A81_causes, 2, kE8A81_effects, 1, kE8A81_recovery, 1},
  {0x8A82, 0x8A82, "Software position limit error", 0x80, FaultReaction::kFaultReactionOption,
    false, kE8A82_causes, 1, kE8A82_effects, 1, kE8A82_recovery, 1},
  {0x8A88, 0x8A88, "STO error", 0x01, FaultReaction::kNoSecureMovement, false, kE8A88_causes, 1,
    kE8A88_effects, 3, kE8A88_recovery, 1},
  {0x8A8A, 0x8A8A, "STO card ready warning", 0x01, FaultReaction::kNoSecureMovement, false,
    kE8A8A_causes, 1, kE8A8A_effects, 2, kE8A8A_recovery, 2},
  {0x8A8B, 0x8A8B, "STO card inactive state error", 0x01, FaultReaction::kNoSecureMovement, false,
    kE8A8B_causes, 1, kE8A8B_effects, 3, kE8A8B_recovery, 1},
  {0xFF01, 0xFF01, "System overloaded error", 0x00, FaultReaction::kWarning, false, kEFF01_causes,
    1, kEFF01_effects, 1, kEFF01_recovery, 1},
  {0xFF02, 0xFF02, "Watchdog error", 0x01, FaultReaction::kNoSecureMovement, true, kEFF02_causes, 1,
    kEFF02_effects, 3, kEFF02_recovery, 1},
  {0xFF0B, 0xFF0B, "System peak overloaded error", 0x01, FaultReaction::kNoSecureMovement, false,
    kEFF0B_causes, 1, kEFF0B_effects, 3, kEFF0B_recovery, 1},
  {0xFF10, 0xFF10, "Controller gain error", 0x20, FaultReaction::kFaultReactionOption, false,
    kEFF10_causes, 1, kEFF10_effects, 4, kEFF10_recovery, 1},
  {0xFF11, 0xFF11, "Auto tuning identification error", 0x20, FaultReaction::kNoSecureMovement,
    false, kEFF11_causes, 1, kEFF11_effects, 3, kEFF11_recovery, 1},
  {0xFF12, 0xFF12, "Auto tuning current limit error", 0x20, FaultReaction::kNoSecureMovement, false,
    kEFF12_causes, 1, kEFF12_effects, 3, kEFF12_recovery, 1},
  {0xFF13, 0xFF13, "Auto tuning identification current error", 0x20,
    FaultReaction::kNoSecureMovement, false, kEFF13_causes, 1, kEFF13_effects, 3,
    kEFF13_recovery, 1},
  {0xFF14, 0xFF14, "Auto tuning data sampling error", 0x20, FaultReaction::kNoSecureMovement, false,
    kEFF14_causes, 1, kEFF14_effects, 3, kEFF14_recovery, 1},
  {0xFF15, 0xFF15, "Auto tuning sample mismatch error", 0x20, FaultReaction::kNoSecureMovement,
    false, kEFF15_causes, 1, kEFF15_effects, 3, kEFF15_recovery, 1},
  {0xFF16, 0xFF16, "Auto tuning parameter error", 0x20, FaultReaction::kNoSecureMovement, false,
    kEFF16_causes, 1, kEFF16_effects, 3, kEFF16_recovery, 1},
  {0xFF17, 0xFF17, "Auto tuning amplitude mismatch error", 0x20, FaultReaction::kNoSecureMovement,
    false, kEFF17_causes, 1, kEFF17_effects, 3, kEFF17_recovery, 1},
  {0xFF19, 0xFF19, "Auto tuning timeout error", 0x20, FaultReaction::kNoSecureMovement, false,
    kEFF19_causes, 1, kEFF19_effects, 3, kEFF19_recovery, 1},
  {0xFF20, 0xFF20, "Auto tuning standstill error", 0x20, FaultReaction::kNoSecureMovement, false,
    kEFF20_causes, 1, kEFF20_effects, 3, kEFF20_recovery, 1},
  {0xFF21, 0xFF21, "Auto tuning torque invalid error", 0x20, FaultReaction::kNoSecureMovement,
    false, kEFF21_causes, 1, kEFF21_effects, 3, kEFF21_recovery, 1},
  {0xFF22, 0xFF22, "Auto tuning max system speed error", 0x20, FaultReaction::kNoSecureMovement,
    false, kEFF22_causes, 1, kEFF22_effects, 3, kEFF22_recovery, 1},
  {0xFF23, 0xFF23, "Auto tuning motor connection error", 0x20, FaultReaction::kNoSecureMovement,
    false, kEFF23_causes, 1, kEFF23_effects, 3, kEFF23_recovery, 1},
  {0xFF24, 0xFF24, "Auto tuning sensor signal error", 0x20, FaultReaction::kNoSecureMovement, false,
    kEFF24_causes, 1, kEFF24_effects, 3, kEFF24_recovery, 1},
};

constexpr AbortCode kAbortCodes[] = {
  {0x00000000u, "No abort", "Communication successful"},
  {0x05030000u, "Toggle error", "Toggle bit not alternated"},
  {0x05040000u, "SDO timeout", "SDO protocol timed out"},
  {0x05040001u, "Command unknown", "Command specifier unknown"},
  {0x05040004u, "CRC error", "CRC check failed"},
  {0x06010000u, "Access error", "Unsupported access to an object"},
  {0x06010001u, "Write only error", "Read command to a write only object"},
  {0x06010002u, "Read only error", "Write command to a read only object"},
  {0x06010003u, "Subindex cannot be written",
    "Subindex cannot be written, subindex 0 must be \"0\" (zero) for write access"},
  {0x06010004u, "SDO complete access not supported",
    "The object cannot be accessed via complete access"},
  {0x06020000u, "Object does not exist error",
    "Last read or write command had wrong object index or subindex"},
  {0x06040041u, "PDO mapping error", "Object is not mappable to the PDO"},
  {0x06040042u, "PDO length error",
    "Number and length of objects to be mapped would exceed PDO length"},
  {0x06040043u, "General parameter error", "General parameter incompatibility"},
  {0x06040047u, "General internal incompatibility error",
    "General internal incompatibility in device"},
  {0x06060000u, "Hardware error", "Access failed due to hardware error"},
  {0x06070010u, "Service parameter error",
    "Data type does not match, length or service parameter do not match"},
  {0x06070013u, "Service parameter too short error",
    "Data type does not match, length of service parameter too low"},
  {0x06090011u, "Subindex error", "Last read or write command had wrong object subindex"},
  {0x06090030u, "Value range error", "Value range of parameter exceeded"},
  {0x08000000u, "General error", "General error"},
  {0x08000020u, "Transfer or store error", "Data cannot be transferred or stored"},
  {0x08000022u, "Wrong device state error",
    "Data cannot be transferred or stored to application because of present device state"},
  {0x0F00FFBEu, "Password error", "Password is incorrect"},
  {0x0F00FFBFu, "Illegal command error", "Command code is illegal (does not exist)"},
  {0x0F00FFC0u, "Wrong NMT state error", "Device is in wrong NMT state"},
};

}  // namespace

const char *
ToString(FaultReaction value)
{
  switch (value) {
    case FaultReaction::kNone: return "none";
    case FaultReaction::kAbortConnectionOption: return "Abort connection option code (0x6007)";
    case FaultReaction::kFaultReactionOption: return "Fault reaction option code (0x605E)";
    case FaultReaction::kNoSecureMovement: return "secure movement no longer possible";
    case FaultReaction::kWarning: return "warning, device keeps running";
  }
  return "unknown";
}

const DeviceError *
FindDeviceError(std::uint16_t code)
{
  // Linear scan on purpose: 73 entries, and ranges make a binary search a
  // trap for anyone who later assumes the keys are unique points.
  for (const auto & e : kDeviceErrors) {
    if (code >= e.code && code <= e.codeEnd) {
      return &e;
    }
  }
  return nullptr;
}

std::string
DeviceErrorName(std::uint16_t code)
{
  const DeviceError * e = FindDeviceError(code);
  char buffer[128];
  if (e == nullptr) {
    std::snprintf(
      buffer, sizeof(buffer),
      "0x%04X unknown error (not in the chapter 7 table; newer firmware?)", code);
    return buffer;
  }
  std::snprintf(buffer, sizeof(buffer), "0x%04X %s", code, e->name);
  return buffer;
}

bool
IsWarning(std::uint16_t code)
{
  const DeviceError * e = FindDeviceError(code);
  return e != nullptr && e->faultReaction == FaultReaction::kWarning;
}

bool
ClearsPosition(std::uint16_t code)
{
  const DeviceError * e = FindDeviceError(code);
  return e != nullptr && e->clearsPosition;
}

std::string
DescribeErrorRegister(std::uint8_t value)
{
  if (value == 0) {
    return "no error flags";
  }
  struct Flag {std::uint8_t bit; const char * name;};
  static constexpr Flag kFlags[] = {
    {error_register::kGeneric, "generic"},
    {error_register::kCurrent, "current"},
    {error_register::kVoltage, "voltage"},
    {error_register::kTemperature, "temperature"},
    {error_register::kCommunication, "communication"},
    {error_register::kDeviceProfile, "device profile"},
    {error_register::kMotion, "motion"},
  };
  std::string out;
  for (const auto & f : kFlags) {
    if (value & f.bit) {
      if (!out.empty()) {out += ", ";}
      out += f.name;
    }
  }
  return out;
}

std::string
DescribeDeviceError(std::uint16_t code)
{
  const DeviceError * e = FindDeviceError(code);
  if (e == nullptr) {
    return DeviceErrorName(code);
  }

  std::string out = DeviceErrorName(code);
  if (e->code != e->codeEnd) {
    char range[64];
    std::snprintf(range, sizeof(range), " (range 0x%04X-0x%04X)", e->code, e->codeEnd);
    out += range;
  }
  out += "\n  register : " + DescribeErrorRegister(e->errorRegister);
  out += "\n  reaction : ";
  out += ToString(e->faultReaction);
  if (e->clearsPosition) {
    out += "\n  WARNING  : resetting this error clears the position; the axis "
      "must be homed again";
  }

  auto section = [&out](const char * label, const char * const * items, std::size_t n) {
      if (items == nullptr || n == 0) {return;}
      out += "\n  ";
      out += label;
      out += " :";
      for (std::size_t i = 0; i < n; ++i) {
        out += "\n      - ";
        out += items[i];
      }
    };
  section("cause", e->causes, e->causeCount);
  section("effect", e->effects, e->effectCount);
  section("recovery", e->recovery, e->recoveryCount);
  return out;
}

const AbortCode *
FindAbortCode(std::uint32_t code)
{
  for (const auto & a : kAbortCodes) {
    if (a.code == code) {
      return &a;
    }
  }
  return nullptr;
}

std::string
DescribeAbortCode(std::uint32_t code)
{
  const AbortCode * a = FindAbortCode(code);
  char buffer[96];
  if (a == nullptr) {
    std::snprintf(buffer, sizeof(buffer), "0x%08X unknown abort code", code);
    return buffer;
  }
  std::snprintf(buffer, sizeof(buffer), "0x%08X %s: ", code, a->name);
  return std::string(buffer) + a->cause;
}

std::string
Describe(const EmergencyMessage & message)
{
  std::string out = DescribeDeviceError(message.errorCode);
  out += "\n  emcy reg : " + DescribeErrorRegister(message.errorRegister);
  return out;
}

}  // namespace epos4::signals
