#pragma once

#include <cstdint>

// Object dictionary of the maxon EPOS4.
//
// Generated from the device EDS (EPOS4 Module 50/15) and cross-checked against
// the EPOS4 Firmware Specification. Index constants live here so that no other
// file in the library contains a bare hexadecimal number.
//
// The split below is not cosmetic. Everything under Cia402 is defined by the
// device profile and would work unchanged against any CiA 402 drive; the maxon
// namespaces are vendor specific and would not. Code written against Cia402
// stays portable.

namespace epos4::od
{

struct Entry
{
  std::uint16_t index;
  std::uint8_t subindex;
};

constexpr Entry At(std::uint16_t index, std::uint8_t subindex = 0)
{
  return Entry{index, subindex};
}

// ---------------------------------------------------------------------------
// Communication profile (CiA 301) - 33 objects
// ---------------------------------------------------------------------------
namespace comm
{
constexpr std::uint16_t kDeviceType = 0x1000;
constexpr std::uint16_t kErrorRegister = 0x1001;
constexpr std::uint16_t kErrorHistory = 0x1003;
constexpr Entry kErrorHistory_ErrorHistory1 = At(0x1003, 1);  // Error history 1
constexpr Entry kErrorHistory_ErrorHistory2 = At(0x1003, 2);  // Error history 2
constexpr Entry kErrorHistory_ErrorHistory3 = At(0x1003, 3);  // Error history 3
constexpr Entry kErrorHistory_ErrorHistory4 = At(0x1003, 4);  // Error history 4
constexpr Entry kErrorHistory_ErrorHistory5 = At(0x1003, 5);  // Error history 5
constexpr std::uint16_t kCOBIDSYNC = 0x1005;
constexpr std::uint16_t kManufacturerDeviceName = 0x1008;
constexpr std::uint16_t kStoreParameters = 0x1010;
constexpr Entry kStoreParameters_SaveAllParameters = At(0x1010, 1);  // Save all parameters
constexpr std::uint16_t kRestoreDefaultParameters = 0x1011;
constexpr Entry kRestoreDefaultParameters_RestoreAllDefaultParameters = At(0x1011, 1);  // Restore all default parameters
constexpr Entry kRestoreDefaultParameters_InternalRestoreFactoryDefaultParameters = At(0x1011, 4);  // Internal restore factory default parameters
constexpr std::uint16_t kCOBIDEMCY = 0x1014;
constexpr std::uint16_t kConsumerHeartbeatTime = 0x1016;
constexpr Entry kConsumerHeartbeatTime_Consumer1HeartbeatTime = At(0x1016, 1);  // Consumer 1 heartbeat time
constexpr Entry kConsumerHeartbeatTime_Consumer2HeartbeatTime = At(0x1016, 2);  // Consumer 2 heartbeat time
constexpr std::uint16_t kProducerHeartbeatTime = 0x1017;
constexpr std::uint16_t kIdentityObject = 0x1018;
constexpr Entry kIdentityObject_VendorID = At(0x1018, 1);  // Vendor ID
constexpr Entry kIdentityObject_ProductCode = At(0x1018, 2);  // Product code
constexpr Entry kIdentityObject_RevisionNumber = At(0x1018, 3);  // Revision number
constexpr Entry kIdentityObject_SerialNumber = At(0x1018, 4);  // Serial number
constexpr std::uint16_t kErrorBehavior = 0x1029;
constexpr Entry kErrorBehavior_CommunicationError = At(0x1029, 1);  // Communication error
constexpr std::uint16_t kSDOServerParameter = 0x1200;
constexpr Entry kSDOServerParameter_COBIDSDOClientToServer = At(0x1200, 1);  // COB-ID SDO client to server
constexpr Entry kSDOServerParameter_COBIDSDOServerToClient = At(0x1200, 2);  // COB-ID SDO server to client
constexpr std::uint16_t kReceivePDO1Parameter = 0x1400;
constexpr Entry kReceivePDO1Parameter_COBIDUsedByRxpdo1 = At(0x1400, 1);  // COB-ID used by RxPDO 1
constexpr Entry kReceivePDO1Parameter_TransmissionTypeRxpdo1 = At(0x1400, 2);  // Transmission type RxPDO 1
constexpr std::uint16_t kReceivePDO2Parameter = 0x1401;
constexpr Entry kReceivePDO2Parameter_COBIDUsedByRxpdo2 = At(0x1401, 1);  // COB-ID used by RxPDO 2
constexpr Entry kReceivePDO2Parameter_TransmissionTypeRxpdo2 = At(0x1401, 2);  // Transmission type RxPDO 2
constexpr std::uint16_t kReceivePDO3Parameter = 0x1402;
constexpr Entry kReceivePDO3Parameter_COBIDUsedByRxpdo3 = At(0x1402, 1);  // COB-ID used by RxPDO 3
constexpr Entry kReceivePDO3Parameter_TransmissionTypeRxpdo3 = At(0x1402, 2);  // Transmission type RxPDO 3
constexpr std::uint16_t kReceivePDO4Parameter = 0x1403;
constexpr Entry kReceivePDO4Parameter_COBIDUsedByRxpdo4 = At(0x1403, 1);  // COB-ID used by RxPDO 4
constexpr Entry kReceivePDO4Parameter_TransmissionTypeRxpdo4 = At(0x1403, 2);  // Transmission type RxPDO 4
constexpr std::uint16_t kReceivePDO1Mapping = 0x1600;
constexpr Entry kReceivePDO1Mapping_N1stMappedObjectInRxpdo1 = At(0x1600, 1);  // 1st mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N2ndMappedObjectInRxpdo1 = At(0x1600, 2);  // 2nd mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N3rdMappedObjectInRxpdo1 = At(0x1600, 3);  // 3rd mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N4thMappedObjectInRxpdo1 = At(0x1600, 4);  // 4th mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N5thMappedObjectInRxpdo1 = At(0x1600, 5);  // 5th mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N6thMappedObjectInRxpdo1 = At(0x1600, 6);  // 6th mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N7thMappedObjectInRxpdo1 = At(0x1600, 7);  // 7th mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N8thMappedObjectInRxpdo1 = At(0x1600, 8);  // 8th mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N9thMappedObjectInRxpdo1 = At(0x1600, 9);  // 9th mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N10thMappedObjectInRxpdo1 = At(0x1600, 10);  // 10th mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N11thMappedObjectInRxpdo1 = At(0x1600, 11);  // 11th mapped object in RxPDO 1
constexpr Entry kReceivePDO1Mapping_N12thMappedObjectInRxpdo1 = At(0x1600, 12);  // 12th mapped object in RxPDO 1
constexpr std::uint16_t kReceivePDO2Mapping = 0x1601;
constexpr Entry kReceivePDO2Mapping_N1stMappedObjectInRxpdo2 = At(0x1601, 1);  // 1st mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N2ndMappedObjectInRxpdo2 = At(0x1601, 2);  // 2nd mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N3rdMappedObjectInRxpdo2 = At(0x1601, 3);  // 3rd mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N4thMappedObjectInRxpdo2 = At(0x1601, 4);  // 4th mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N5thMappedObjectInRxpdo2 = At(0x1601, 5);  // 5th mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N6thMappedObjectInRxpdo2 = At(0x1601, 6);  // 6th mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N7thMappedObjectInRxpdo2 = At(0x1601, 7);  // 7th mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N8thMappedObjectInRxpdo2 = At(0x1601, 8);  // 8th mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N9thMappedObjectInRxpdo2 = At(0x1601, 9);  // 9th mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N10thMappedObjectInRxpdo2 = At(0x1601, 10);  // 10th mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N11thMappedObjectInRxpdo2 = At(0x1601, 11);  // 11th mapped object in RxPDO 2
constexpr Entry kReceivePDO2Mapping_N12thMappedObjectInRxpdo2 = At(0x1601, 12);  // 12th mapped object in RxPDO 2
constexpr std::uint16_t kReceivePDO3Mapping = 0x1602;
constexpr Entry kReceivePDO3Mapping_N1stMappedObjectInRxpdo3 = At(0x1602, 1);  // 1st mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N2ndMappedObjectInRxpdo3 = At(0x1602, 2);  // 2nd mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N3rdMappedObjectInRxpdo3 = At(0x1602, 3);  // 3rd mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N4thMappedObjectInRxpdo3 = At(0x1602, 4);  // 4th mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N5thMappedObjectInRxpdo3 = At(0x1602, 5);  // 5th mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N6thMappedObjectInRxpdo3 = At(0x1602, 6);  // 6th mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N7thMappedObjectInRxpdo3 = At(0x1602, 7);  // 7th mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N8thMappedObjectInRxpdo3 = At(0x1602, 8);  // 8th mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N9thMappedObjectInRxpdo3 = At(0x1602, 9);  // 9th mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N10thMappedObjectInRxpdo3 = At(0x1602, 10);  // 10th mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N11thMappedObjectInRxpdo3 = At(0x1602, 11);  // 11th mapped object in RxPDO 3
constexpr Entry kReceivePDO3Mapping_N12thMappedObjectInRxpdo3 = At(0x1602, 12);  // 12th mapped object in RxPDO 3
constexpr std::uint16_t kReceivePDO4Mapping = 0x1603;
constexpr Entry kReceivePDO4Mapping_N1stMappedObjectInRxpdo4 = At(0x1603, 1);  // 1st mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N2ndMappedObjectInRxpdo4 = At(0x1603, 2);  // 2nd mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N3rdMappedObjectInRxpdo4 = At(0x1603, 3);  // 3rd mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N4thMappedObjectInRxpdo4 = At(0x1603, 4);  // 4th mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N5thMappedObjectInRxpdo4 = At(0x1603, 5);  // 5th mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N6thMappedObjectInRxpdo4 = At(0x1603, 6);  // 6th mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N7thMappedObjectInRxpdo4 = At(0x1603, 7);  // 7th mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N8thMappedObjectInRxpdo4 = At(0x1603, 8);  // 8th mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N9thMappedObjectInRxpdo4 = At(0x1603, 9);  // 9th mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N10thMappedObjectInRxpdo4 = At(0x1603, 10);  // 10th mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N11thMappedObjectInRxpdo4 = At(0x1603, 11);  // 11th mapped object in RxPDO 4
constexpr Entry kReceivePDO4Mapping_N12thMappedObjectInRxpdo4 = At(0x1603, 12);  // 12th mapped object in RxPDO 4
constexpr std::uint16_t kTransmitPDO1Parameter = 0x1800;
constexpr Entry kTransmitPDO1Parameter_COBIDUsedByTxpdo1 = At(0x1800, 1);  // COB-ID used by TxPDO 1
constexpr Entry kTransmitPDO1Parameter_TransmissionTypeTxpdo1 = At(0x1800, 2);  // Transmission type TxPDO 1
constexpr Entry kTransmitPDO1Parameter_InhibitTimeTxpdo1 = At(0x1800, 3);  // Inhibit time TxPDO 1
constexpr std::uint16_t kTransmitPDO2Parameter = 0x1801;
constexpr Entry kTransmitPDO2Parameter_COBIDUsedByTxpdo2 = At(0x1801, 1);  // COB-ID used by TxPDO 2
constexpr Entry kTransmitPDO2Parameter_TransmissionTypeTxpdo2 = At(0x1801, 2);  // Transmission type TxPDO 2
constexpr Entry kTransmitPDO2Parameter_InhibitTimeTxpdo2 = At(0x1801, 3);  // Inhibit time TxPDO 2
constexpr std::uint16_t kTransmitPDO3Parameter = 0x1802;
constexpr Entry kTransmitPDO3Parameter_COBIDUsedByTxpdo3 = At(0x1802, 1);  // COB-ID used by TxPDO 3
constexpr Entry kTransmitPDO3Parameter_TransmissionTypeTxpdo3 = At(0x1802, 2);  // Transmission type TxPDO 3
constexpr Entry kTransmitPDO3Parameter_InhibitTimeTxpdo3 = At(0x1802, 3);  // Inhibit time TxPDO 3
constexpr std::uint16_t kTransmitPDO4Parameter = 0x1803;
constexpr Entry kTransmitPDO4Parameter_COBIDUsedByTxpdo4 = At(0x1803, 1);  // COB-ID used by TxPDO 4
constexpr Entry kTransmitPDO4Parameter_TransmissionTypeTxpdo4 = At(0x1803, 2);  // Transmission type TxPDO 4
constexpr Entry kTransmitPDO4Parameter_InhibitTimeTxpdo4 = At(0x1803, 3);  // Inhibit time TxPDO 4
constexpr std::uint16_t kTransmitPDO1Mapping = 0x1A00;
constexpr Entry kTransmitPDO1Mapping_N1stMappedObjectInTxpdo1 = At(0x1A00, 1);  // 1st mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N2ndMappedObjectInTxpdo1 = At(0x1A00, 2);  // 2nd mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N3rdMappedObjectInTxpdo1 = At(0x1A00, 3);  // 3rd mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N4thMappedObjectInTxpdo1 = At(0x1A00, 4);  // 4th mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N5thMappedObjectInTxpdo1 = At(0x1A00, 5);  // 5th mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N6thMappedObjectInTxpdo1 = At(0x1A00, 6);  // 6th mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N7thMappedObjectInTxpdo1 = At(0x1A00, 7);  // 7th mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N8thMappedObjectInTxpdo1 = At(0x1A00, 8);  // 8th mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N9thMappedObjectInTxpdo1 = At(0x1A00, 9);  // 9th mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N10thMappedObjectInTxpdo1 = At(0x1A00, 10);  // 10th mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N11thMappedObjectInTxpdo1 = At(0x1A00, 11);  // 11th mapped object in TxPDO 1
constexpr Entry kTransmitPDO1Mapping_N12thMappedObjectInTxpdo1 = At(0x1A00, 12);  // 12th mapped object in TxPDO 1
constexpr std::uint16_t kTransmitPDO2Mapping = 0x1A01;
constexpr Entry kTransmitPDO2Mapping_N1stMappedObjectInTxpdo2 = At(0x1A01, 1);  // 1st mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N2ndMappedObjectInTxpdo2 = At(0x1A01, 2);  // 2nd mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N3rdMappedObjectInTxpdo2 = At(0x1A01, 3);  // 3rd mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N4thMappedObjectInTxpdo2 = At(0x1A01, 4);  // 4th mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N5thMappedObjectInTxpdo2 = At(0x1A01, 5);  // 5th mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N6thMappedObjectInTxpdo2 = At(0x1A01, 6);  // 6th mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N7thMappedObjectInTxpdo2 = At(0x1A01, 7);  // 7th mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N8thMappedObjectInTxpdo2 = At(0x1A01, 8);  // 8th mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N9thMappedObjectInTxpdo2 = At(0x1A01, 9);  // 9th mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N10thMappedObjectInTxpdo2 = At(0x1A01, 10);  // 10th mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N11thMappedObjectInTxpdo2 = At(0x1A01, 11);  // 11th mapped object in TxPDO 2
constexpr Entry kTransmitPDO2Mapping_N12thMappedObjectInTxpdo2 = At(0x1A01, 12);  // 12th mapped object in TxPDO 2
constexpr std::uint16_t kTransmitPDO3Mapping = 0x1A02;
constexpr Entry kTransmitPDO3Mapping_N1stMappedObjectInTxpdo3 = At(0x1A02, 1);  // 1st mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N2ndMappedObjectInTxpdo3 = At(0x1A02, 2);  // 2nd mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N3rdMappedObjectInTxpdo3 = At(0x1A02, 3);  // 3rd mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N4thMappedObjectInTxpdo3 = At(0x1A02, 4);  // 4th mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N5thMappedObjectInTxpdo3 = At(0x1A02, 5);  // 5th mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N6thMappedObjectInTxpdo3 = At(0x1A02, 6);  // 6th mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N7thMappedObjectInTxpdo3 = At(0x1A02, 7);  // 7th mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N8thMappedObjectInTxpdo3 = At(0x1A02, 8);  // 8th mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N9thMappedObjectInTxpdo3 = At(0x1A02, 9);  // 9th mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N10thMappedObjectInTxpdo3 = At(0x1A02, 10);  // 10th mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N11thMappedObjectInTxpdo3 = At(0x1A02, 11);  // 11th mapped object in TxPDO 3
constexpr Entry kTransmitPDO3Mapping_N12thMappedObjectInTxpdo3 = At(0x1A02, 12);  // 12th mapped object in TxPDO 3
constexpr std::uint16_t kTransmitPDO4Mapping = 0x1A03;
constexpr Entry kTransmitPDO4Mapping_N1stMappedObjectInTxpdo4 = At(0x1A03, 1);  // 1st mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N2ndMappedObjectInTxpdo4 = At(0x1A03, 2);  // 2nd mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N3rdMappedObjectInTxpdo4 = At(0x1A03, 3);  // 3rd mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N4thMappedObjectInTxpdo4 = At(0x1A03, 4);  // 4th mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N5thMappedObjectInTxpdo4 = At(0x1A03, 5);  // 5th mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N6thMappedObjectInTxpdo4 = At(0x1A03, 6);  // 6th mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N7thMappedObjectInTxpdo4 = At(0x1A03, 7);  // 7th mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N8thMappedObjectInTxpdo4 = At(0x1A03, 8);  // 8th mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N9thMappedObjectInTxpdo4 = At(0x1A03, 9);  // 9th mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N10thMappedObjectInTxpdo4 = At(0x1A03, 10);  // 10th mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N11thMappedObjectInTxpdo4 = At(0x1A03, 11);  // 11th mapped object in TxPDO 4
constexpr Entry kTransmitPDO4Mapping_N12thMappedObjectInTxpdo4 = At(0x1A03, 12);  // 12th mapped object in TxPDO 4
constexpr std::uint16_t kProgramData = 0x1F50;
constexpr Entry kProgramData_ProgramNumber1 = At(0x1F50, 1);  // Program number 1
constexpr std::uint16_t kProgramControl = 0x1F51;
constexpr Entry kProgramControl_ProgramNumber1 = At(0x1F51, 1);  // Program number 1
constexpr std::uint16_t kProgramSoftwareIdentification = 0x1F56;
constexpr Entry kProgramSoftwareIdentification_ProgramNumber1 = At(0x1F56, 1);  // Program number 1
constexpr std::uint16_t kFlashStatusIdentification = 0x1F57;
constexpr Entry kFlashStatusIdentification_ProgramNumber1 = At(0x1F57, 1);  // Program number 1
}  // namespace comm

// ---------------------------------------------------------------------------
// maxon communication - 12 objects
// ---------------------------------------------------------------------------
namespace maxon_comm
{
constexpr std::uint16_t kNodeID = 0x2000;
constexpr std::uint16_t kCANBitRate = 0x2001;
constexpr std::uint16_t kRS232BitRate = 0x2002;
constexpr std::uint16_t kRS232FrameTimeout = 0x2005;
constexpr std::uint16_t kUSBFrameTimeout = 0x2006;
constexpr std::uint16_t kCANBitRateDisplay = 0x200A;
constexpr std::uint16_t kActiveFieldbus = 0x2010;
constexpr std::uint16_t kAdditionalIdentity = 0x2100;
constexpr Entry kAdditionalIdentity_SerialNumberComplete = At(0x2100, 1);  // Serial number complete
constexpr std::uint16_t kCustomPersistentMemory = 0x210C;
constexpr Entry kCustomPersistentMemory_CustomPersistentMemory1 = At(0x210C, 1);  // Custom persistent memory 1
constexpr Entry kCustomPersistentMemory_CustomPersistentMemory2 = At(0x210C, 2);  // Custom persistent memory 2
constexpr Entry kCustomPersistentMemory_CustomPersistentMemory3 = At(0x210C, 3);  // Custom persistent memory 3
constexpr Entry kCustomPersistentMemory_CustomPersistentMemory4 = At(0x210C, 4);  // Custom persistent memory 4
constexpr std::uint16_t kPowerSupply = 0x2200;
constexpr Entry kPowerSupply_PowerSupplyVoltage = At(0x2200, 1);  // Power supply voltage
constexpr Entry kPowerSupply_InternalValidLogicSupply = At(0x2200, 2);  // Internal valid logic supply
constexpr std::uint16_t kPowerSupplySupervision = 0x2201;
constexpr Entry kPowerSupplySupervision_PowerSupplyUndervoltageLimit = At(0x2201, 1);  // Power supply undervoltage limit
constexpr Entry kPowerSupplySupervision_PowerSupplyOvervoltageLimit = At(0x2201, 2);  // Power supply overvoltage limit
constexpr std::uint16_t kInternalDeviceControl = 0x2EC0;
constexpr Entry kInternalDeviceControl_InternalErrorControl = At(0x2EC0, 1);  // Internal error control
constexpr Entry kInternalDeviceControl_InternalCANNMTState = At(0x2EC0, 2);  // Internal CAN NMT state
constexpr Entry kInternalDeviceControl_InternalBootloaderRevisionNumber = At(0x2EC0, 3);  // Internal bootloader revision number
constexpr Entry kInternalDeviceControl_InternalBootloaderProductCode = At(0x2EC0, 4);  // Internal bootloader product code
constexpr Entry kInternalDeviceControl_InternalExtensionCommunicationErrorCode = At(0x2EC0, 5);  // Internal Extension communication error code
constexpr Entry kInternalDeviceControl_InternalESMState = At(0x2EC0, 6);  // Internal ESM state
constexpr Entry kInternalDeviceControl_InternalLEDIdentification = At(0x2EC0, 8);  // Internal LED Identification
constexpr Entry kInternalDeviceControl_InternalDebugState = At(0x2EC0, 9);  // Internal debug state
}  // namespace maxon_comm

// ---------------------------------------------------------------------------
// maxon device configuration - 61 objects
// ---------------------------------------------------------------------------
namespace maxon
{
constexpr std::uint16_t kAxisConfiguration = 0x3000;
constexpr Entry kAxisConfiguration_SensorsConfiguration = At(0x3000, 1);  // Sensors configuration
constexpr Entry kAxisConfiguration_ControlStructure = At(0x3000, 2);  // Control structure
constexpr Entry kAxisConfiguration_CommutationSensors = At(0x3000, 3);  // Commutation sensors
constexpr Entry kAxisConfiguration_AxisConfigurationMiscellaneous = At(0x3000, 4);  // Axis configuration miscellaneous
constexpr Entry kAxisConfiguration_MainSensorResolution = At(0x3000, 5);  // Main sensor resolution
constexpr Entry kAxisConfiguration_MaxSystemSpeed = At(0x3000, 6);  // Max system speed
constexpr std::uint16_t kMotorData = 0x3001;
constexpr Entry kMotorData_NominalCurrent = At(0x3001, 1);  // Nominal current
constexpr Entry kMotorData_OutputCurrentLimit = At(0x3001, 2);  // Output current limit
constexpr Entry kMotorData_NumberOfPolePairs = At(0x3001, 3);  // Number of pole pairs
constexpr Entry kMotorData_ThermalTimeConstantWinding = At(0x3001, 4);  // Thermal time constant winding
constexpr Entry kMotorData_TorqueConstant = At(0x3001, 5);  // Torque constant
constexpr std::uint16_t kElectricalSystemParameters = 0x3002;
constexpr Entry kElectricalSystemParameters_ElectricalResistance = At(0x3002, 1);  // Electrical resistance
constexpr Entry kElectricalSystemParameters_ElectricalInductance = At(0x3002, 2);  // Electrical inductance
constexpr std::uint16_t kGearConfiguration = 0x3003;
constexpr Entry kGearConfiguration_GearReductionNumerator = At(0x3003, 1);  // Gear reduction numerator
constexpr Entry kGearConfiguration_GearReductionDenominator = At(0x3003, 2);  // Gear reduction denominator
constexpr Entry kGearConfiguration_MaxGearInputSpeed = At(0x3003, 3);  // Max gear input speed
constexpr Entry kGearConfiguration_GearMiscellaneousConfiguration = At(0x3003, 4);  // Gear miscellaneous configuration
constexpr std::uint16_t kDigitalIncrementalEncoder1 = 0x3010;
constexpr Entry kDigitalIncrementalEncoder1_DigitalIncrementalEncoder1NumberOfPulses =
  At(0x3010, 1);                                                                                       // Digital incremental encoder 1 number of pulses
constexpr Entry kDigitalIncrementalEncoder1_DigitalIncrementalEncoder1Type = At(0x3010, 2);  // Digital incremental encoder 1 type
constexpr Entry kDigitalIncrementalEncoder1_DigitalIncrementalEncoder1IndexPosition = At(0x3010, 4);  // Digital incremental encoder 1 index position
constexpr std::uint16_t kAnalogIncrementalEncoder = 0x3011;
constexpr Entry kAnalogIncrementalEncoder_AnalogIncrementalEncoderType = At(0x3011, 1);  // Analog incremental encoder type
constexpr Entry kAnalogIncrementalEncoder_AnalogIncrementalEncoderResolution = At(0x3011, 2);  // Analog incremental encoder resolution
constexpr Entry kAnalogIncrementalEncoder_AnalogIncrementalEncoderIndexPosition = At(0x3011, 3);  // Analog incremental encoder index position
constexpr std::uint16_t kSSIAbsoluteEncoder = 0x3012;
constexpr Entry kSSIAbsoluteEncoder_SSIDataRate = At(0x3012, 1);  // SSI data rate
constexpr Entry kSSIAbsoluteEncoder_SSINumberOfDataBits = At(0x3012, 2);  // SSI number of data bits
constexpr Entry kSSIAbsoluteEncoder_SSIEncodingType = At(0x3012, 3);  // SSI encoding type
constexpr Entry kSSIAbsoluteEncoder_SSITimeoutTime = At(0x3012, 5);  // SSI timeout time
constexpr Entry kSSIAbsoluteEncoder_SSISpecialBitsTrailingData = At(0x3012, 6);  // SSI special bits trailing data
constexpr Entry kSSIAbsoluteEncoder_SSIRefreshFrequency = At(0x3012, 7);  // SSI refresh frequency
constexpr Entry kSSIAbsoluteEncoder_SSIPowerUpTime = At(0x3012, 8);  // SSI power up time
constexpr Entry kSSIAbsoluteEncoder_SSIPositionRawValue = At(0x3012, 9);  // SSI position raw value
constexpr Entry kSSIAbsoluteEncoder_SSICommutationOffsetValue = At(0x3012, 10);  // SSI commutation offset value
constexpr Entry kSSIAbsoluteEncoder_SSIPositionBits = At(0x3012, 11);  // SSI position bits
constexpr Entry kSSIAbsoluteEncoder_SSISpecialBitsLeadingData = At(0x3012, 12);  // SSI special bits leading data
constexpr Entry kSSIAbsoluteEncoder_SSIPositionRawValueComplete = At(0x3012, 13);  // SSI position raw value complete
constexpr std::uint16_t kDigitalHallSensor = 0x301A;
constexpr Entry kDigitalHallSensor_DigitalHallSensorType = At(0x301A, 1);  // Digital Hall sensor type
constexpr Entry kDigitalHallSensor_DigitalHallSensorPattern = At(0x301A, 2);  // Digital Hall sensor pattern
constexpr std::uint16_t kDigitalIncrementalEncoder2 = 0x3020;
constexpr Entry kDigitalIncrementalEncoder2_DigitalIncrementalEncoder2NumberOfPulses =
  At(0x3020, 1);                                                                                       // Digital incremental encoder 2 number of pulses
constexpr Entry kDigitalIncrementalEncoder2_DigitalIncrementalEncoder2Type = At(0x3020, 2);  // Digital incremental encoder 2 type
constexpr Entry kDigitalIncrementalEncoder2_DigitalIncrementalEncoder2IndexPosition = At(0x3020, 4);  // Digital incremental encoder 2 index position
constexpr std::uint16_t kCurrentControlParameterSet = 0x30A0;
constexpr Entry kCurrentControlParameterSet_CurrentControllerPGain = At(0x30A0, 1);  // Current controller P gain
constexpr Entry kCurrentControlParameterSet_CurrentControllerIGain = At(0x30A0, 2);  // Current controller I gain
constexpr std::uint16_t kPositionControlParameterSet = 0x30A1;
constexpr Entry kPositionControlParameterSet_PositionControllerPGain = At(0x30A1, 1);  // Position controller P gain
constexpr Entry kPositionControlParameterSet_PositionControllerIGain = At(0x30A1, 2);  // Position controller I gain
constexpr Entry kPositionControlParameterSet_PositionControllerDGain = At(0x30A1, 3);  // Position controller D gain
constexpr Entry kPositionControlParameterSet_PositionControllerFFVelocityGain = At(0x30A1, 4);  // Position controller FF velocity gain
constexpr Entry kPositionControlParameterSet_PositionControllerFFAccelerationGain = At(0x30A1, 5);  // Position controller FF acceleration gain
constexpr Entry kPositionControlParameterSet_SIUnitPositionControllerIGain = At(0x30A1, 9);  // SI unit position controller I gain
constexpr std::uint16_t kVelocityControlParameterSet = 0x30A2;
constexpr Entry kVelocityControlParameterSet_VelocityControllerPGain = At(0x30A2, 1);  // Velocity controller P gain
constexpr Entry kVelocityControlParameterSet_VelocityControllerIGain = At(0x30A2, 2);  // Velocity controller I gain
constexpr Entry kVelocityControlParameterSet_VelocityControllerFFVelocityGain = At(0x30A2, 3);  // Velocity controller FF velocity gain
constexpr Entry kVelocityControlParameterSet_VelocityControllerFFAccelerationGain = At(0x30A2, 4);  // Velocity controller FF acceleration gain
constexpr Entry kVelocityControlParameterSet_VelocityControllerFilterCutOffFrequency =
  At(0x30A2, 5);                                                                                       // Velocity controller filter cut-off frequency
constexpr std::uint16_t kVelocityObserverParameterSet = 0x30A3;
constexpr Entry kVelocityObserverParameterSet_VelocityObserverPositionCorrectionGain =
  At(0x30A3, 1);                                                                                       // Velocity observer position correction gain
constexpr Entry kVelocityObserverParameterSet_VelocityObserverVelocityCorrectionGain =
  At(0x30A3, 2);                                                                                       // Velocity observer velocity correction gain
constexpr Entry kVelocityObserverParameterSet_VelocityObserverLoadCorrectionGain = At(0x30A3, 3);  // Velocity observer load correction gain
constexpr Entry kVelocityObserverParameterSet_VelocityObserverFriction = At(0x30A3, 4);  // Velocity observer friction
constexpr Entry kVelocityObserverParameterSet_VelocityObserverInertia = At(0x30A3, 5);  // Velocity observer inertia
constexpr std::uint16_t kInternalVelocityFilterParameterSet = 0x30A4;
constexpr Entry kInternalVelocityFilterParameterSet_InternalVelocityFilterBandwidthCorrectionFactor
  =
  At(0x30A4, 1);                                                                                                      // Internal velocity filter bandwidth correction factor
constexpr std::uint16_t kDualLoopPositionControlParameterSet = 0x30AE;
constexpr Entry kDualLoopPositionControlParameterSet_MainLoopPGainLowBandwidth = At(0x30AE, 1);  // Main loop P gain low bandwidth
constexpr Entry kDualLoopPositionControlParameterSet_MainLoopPGainHighBandwidth = At(0x30AE, 2);  // Main loop P gain high bandwidth
constexpr Entry kDualLoopPositionControlParameterSet_MainLoopGainSchedulingWeight = At(0x30AE, 3);  // Main loop gain scheduling weight
constexpr Entry kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientA = At(0x30AE, 16);  // Main loop filter coefficient a
constexpr Entry kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientB = At(0x30AE, 17);  // Main loop filter coefficient b
constexpr Entry kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientC = At(0x30AE, 18);  // Main loop filter coefficient c
constexpr Entry kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientD = At(0x30AE, 19);  // Main loop filter coefficient d
constexpr Entry kDualLoopPositionControlParameterSet_MainLoopFilterCoefficientE = At(0x30AE, 20);  // Main loop filter coefficient e
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopPGain = At(0x30AE, 32);  // Auxiliary loop P gain
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopIGain = At(0x30AE, 33);  // Auxiliary loop I gain
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopFFVelocityGain = At(0x30AE, 34);  // Auxiliary loop FF velocity gain
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopFFAccelerationGain =
  At(0x30AE, 35);                                                                                       // Auxiliary loop FF acceleration gain
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverPositionCorrectionGain =
  At(0x30AE, 48);                                                                                                   // Auxiliary loop observer position correction gain
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverVelocityCorrectionGain =
  At(0x30AE, 49);                                                                                                   // Auxiliary loop observer velocity correction gain
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverLoadCorrectionGain = At(
  0x30AE, 50);                                                                                                  // Auxiliary loop observer load correction gain
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverFriction = At(0x30AE, 51);  // Auxiliary loop observer friction
constexpr Entry kDualLoopPositionControlParameterSet_AuxiliaryLoopObserverInertia = At(0x30AE, 52);  // Auxiliary loop observer inertia
constexpr Entry kDualLoopPositionControlParameterSet_DualLoopConfigurationMiscellaneous = At(
  0x30AE,
  64);                                                                                                     // Dual loop configuration miscellaneous
constexpr std::uint16_t kHomePosition = 0x30B0;
constexpr std::uint16_t kHomeOffsetMoveDistance = 0x30B1;
constexpr std::uint16_t kCurrentThresholdForHomingMode = 0x30B2;
constexpr std::uint16_t kInternalHomingData = 0x30B5;
constexpr Entry kInternalHomingData_InternalAbsoluteHomeReference = At(0x30B5, 1);  // Internal absolute home reference
constexpr Entry kInternalHomingData_InternalAbsoluteHomeReferenceState = At(0x30B5, 2);  // Internal absolute home reference state
constexpr std::uint16_t kCurrentDemandValue = 0x30D0;
constexpr std::uint16_t kCurrentActualValues = 0x30D1;
constexpr Entry kCurrentActualValues_CurrentActualValueAveraged = At(0x30D1, 1);  // Current actual value averaged
constexpr Entry kCurrentActualValues_CurrentActualValue = At(0x30D1, 2);  // Current actual value
constexpr std::uint16_t kTorqueActualValues = 0x30D2;
constexpr Entry kTorqueActualValues_TorqueActualValueAveraged = At(0x30D2, 1);  // Torque actual value averaged
constexpr std::uint16_t kVelocityActualValues = 0x30D3;
constexpr Entry kVelocityActualValues_VelocityActualValueAveraged = At(0x30D3, 1);  // Velocity actual value averaged
constexpr std::uint16_t kStandstillWindowConfiguration = 0x30E0;
constexpr Entry kStandstillWindowConfiguration_StandstillWindow = At(0x30E0, 1);  // Standstill window
constexpr Entry kStandstillWindowConfiguration_StandstillWindowTime = At(0x30E0, 2);  // Standstill window time
constexpr Entry kStandstillWindowConfiguration_StandstillWindowTimeout = At(0x30E0, 3);  // Standstill window timeout
constexpr std::uint16_t kDigitalInputProperties = 0x3141;
constexpr Entry kDigitalInputProperties_DigitalInputsLogicState = At(0x3141, 1);  // Digital inputs logic state
constexpr Entry kDigitalInputProperties_DigitalInputsPolarity = At(0x3141, 2);  // Digital inputs polarity
constexpr std::uint16_t kConfigurationOfDigitalInputs = 0x3142;
constexpr Entry kConfigurationOfDigitalInputs_DigitalInput1Configuration = At(0x3142, 1);  // Digital input 1 configuration
constexpr Entry kConfigurationOfDigitalInputs_DigitalInput2Configuration = At(0x3142, 2);  // Digital input 2 configuration
constexpr Entry kConfigurationOfDigitalInputs_DigitalInput3Configuration = At(0x3142, 3);  // Digital input 3 configuration
constexpr Entry kConfigurationOfDigitalInputs_DigitalInput4Configuration = At(0x3142, 4);  // Digital input 4 configuration
constexpr Entry kConfigurationOfDigitalInputs_HighSpeedDigitalInput1Configuration = At(0x3142, 5);  // High-speed digital input 1 configuration
constexpr Entry kConfigurationOfDigitalInputs_HighSpeedDigitalInput2Configuration = At(0x3142, 6);  // High-speed digital input 2 configuration
constexpr Entry kConfigurationOfDigitalInputs_HighSpeedDigitalInput3Configuration = At(0x3142, 7);  // High-speed digital input 3 configuration
constexpr Entry kConfigurationOfDigitalInputs_HighSpeedDigitalInput4Configuration = At(0x3142, 8);  // High-speed digital input 4 configuration
constexpr std::uint16_t kDigitalOutputProperties = 0x3150;
constexpr Entry kDigitalOutputProperties_DigitalOutputsLogicState = At(0x3150, 1);  // Digital outputs logic state
constexpr Entry kDigitalOutputProperties_DigitalOutputsPolarity = At(0x3150, 2);  // Digital outputs polarity
constexpr std::uint16_t kConfigurationOfDigitalOutputs = 0x3151;
constexpr Entry kConfigurationOfDigitalOutputs_DigitalOutput1Configuration = At(0x3151, 1);  // Digital output 1 configuration
constexpr Entry kConfigurationOfDigitalOutputs_DigitalOutput2Configuration = At(0x3151, 2);  // Digital output 2 configuration
constexpr Entry kConfigurationOfDigitalOutputs_HighSpeedDigitalOutput1Configuration = At(0x3151, 3);  // High-speed digital output 1 configuration
constexpr std::uint16_t kHoldingBrakeParameters = 0x3158;
constexpr Entry kHoldingBrakeParameters_HoldingBrakeRiseTime = At(0x3158, 1);  // Holding brake rise time
constexpr Entry kHoldingBrakeParameters_HoldingBrakeFallTime = At(0x3158, 2);  // Holding brake fall time
constexpr Entry kHoldingBrakeParameters_HoldingBrakeState = At(0x3158, 3);  // Holding brake state
constexpr std::uint16_t kAnalogInputProperties = 0x3160;
constexpr Entry kAnalogInputProperties_AnalogInput1Voltage = At(0x3160, 1);  // Analog input 1 voltage
constexpr Entry kAnalogInputProperties_AnalogInput2Voltage = At(0x3160, 2);  // Analog input 2 voltage
constexpr std::uint16_t kConfigurationOfAnalogInputs = 0x3161;
constexpr Entry kConfigurationOfAnalogInputs_AnalogInput1Configuration = At(0x3161, 1);  // Analog input 1 configuration
constexpr Entry kConfigurationOfAnalogInputs_AnalogInput2Configuration = At(0x3161, 2);  // Analog input 2 configuration
constexpr std::uint16_t kAnalogInputGeneralPurpose = 0x3162;
constexpr Entry kAnalogInputGeneralPurpose_AnalogInputGeneralPurposeA = At(0x3162, 1);  // Analog input general purpose A
constexpr Entry kAnalogInputGeneralPurpose_AnalogInputGeneralPurposeB = At(0x3162, 2);  // Analog input general purpose B
constexpr std::uint16_t kAnalogInputAdjustment = 0x3163;
constexpr Entry kAnalogInputAdjustment_AnalogInput1AdjustmentOffset = At(0x3163, 1);  // Analog input 1 adjustment offset
constexpr Entry kAnalogInputAdjustment_AnalogInput1AdjustmentGainFactor = At(0x3163, 2);  // Analog input 1 adjustment gain factor
constexpr Entry kAnalogInputAdjustment_AnalogInput2AdjustmentOffset = At(0x3163, 3);  // Analog input 2 adjustment offset
constexpr Entry kAnalogInputAdjustment_AnalogInput2AdjustmentGainFactor = At(0x3163, 4);  // Analog input 2 adjustment gain factor
constexpr std::uint16_t kAnalogInputCurrentSetValueProperties = 0x3170;
constexpr Entry kAnalogInputCurrentSetValueProperties_CurrentSetValueFirstVoltage = At(0x3170, 1);  // Current set value first voltage
constexpr Entry kAnalogInputCurrentSetValueProperties_CurrentSetValueFirstCurrent = At(0x3170, 2);  // Current set value first current
constexpr Entry kAnalogInputCurrentSetValueProperties_CurrentSetValueSecondVoltage = At(0x3170, 3);  // Current set value second voltage
constexpr Entry kAnalogInputCurrentSetValueProperties_CurrentSetValueSecondCurrent = At(0x3170, 4);  // Current set value second current
constexpr std::uint16_t kAnalogInputVelocitySetValueProperties = 0x3171;
constexpr Entry kAnalogInputVelocitySetValueProperties_VelocitySetValueFirstVoltage = At(0x3171, 1);  // Velocity set value first voltage
constexpr Entry kAnalogInputVelocitySetValueProperties_VelocitySetValueFirstVelocity =
  At(0x3171, 2);                                                                                       // Velocity set value first velocity
constexpr Entry kAnalogInputVelocitySetValueProperties_VelocitySetValueSecondVoltage =
  At(0x3171, 3);                                                                                       // Velocity set value second voltage
constexpr Entry kAnalogInputVelocitySetValueProperties_VelocitySetValueSecondVelocity =
  At(0x3171, 4);                                                                                        // Velocity set value second velocity
constexpr std::uint16_t kAnalogOutputProperties = 0x3180;
constexpr Entry kAnalogOutputProperties_AnalogOutput1Voltage = At(0x3180, 1);  // Analog output 1 voltage
constexpr Entry kAnalogOutputProperties_AnalogOutput2Voltage = At(0x3180, 2);  // Analog output 2 voltage
constexpr std::uint16_t kConfigurationOfAnalogOutputs = 0x3181;
constexpr Entry kConfigurationOfAnalogOutputs_AnalogOutput1Configuration = At(0x3181, 1);  // Analog output 1 configuration
constexpr Entry kConfigurationOfAnalogOutputs_AnalogOutput2Configuration = At(0x3181, 2);  // Analog output 2 configuration
constexpr std::uint16_t kAnalogOutputGeneralPurpose = 0x3182;
constexpr Entry kAnalogOutputGeneralPurpose_AnalogOutputGeneralPurposeA = At(0x3182, 1);  // Analog output general purpose A
constexpr Entry kAnalogOutputGeneralPurpose_AnalogOutputGeneralPurposeB = At(0x3182, 2);  // Analog output general purpose B
constexpr std::uint16_t kInternalDataRecorderVariableIndex = 0x31E0;
constexpr Entry kInternalDataRecorderVariableIndex_InternalDataRecorderVariableIndex1 =
  At(0x31E0, 1);                                                                                        // Internal data recorder variable index 1
constexpr Entry kInternalDataRecorderVariableIndex_InternalDataRecorderVariableIndex2 =
  At(0x31E0, 2);                                                                                        // Internal data recorder variable index 2
constexpr Entry kInternalDataRecorderVariableIndex_InternalDataRecorderVariableIndex3 =
  At(0x31E0, 3);                                                                                        // Internal data recorder variable index 3
constexpr Entry kInternalDataRecorderVariableIndex_InternalDataRecorderVariableIndex4 =
  At(0x31E0, 4);                                                                                        // Internal data recorder variable index 4
constexpr std::uint16_t kInternalDataRecorderConfiguration = 0x31E1;
constexpr Entry kInternalDataRecorderConfiguration_InternalDataRecorderSamplingPeriod =
  At(0x31E1, 1);                                                                                        // Internal data recorder sampling period
constexpr Entry kInternalDataRecorderConfiguration_InternalDataRecorderMaxNumberOfSamples = At(
  0x31E1, 2);                                                                                               // Internal data recorder max number of samples
constexpr std::uint16_t kInternalDataRecorderTrigger = 0x31E2;
constexpr Entry kInternalDataRecorderTrigger_InternalDataRecorderTriggerVariableIndex =
  At(0x31E2, 1);                                                                                        // Internal data recorder trigger variable index
constexpr Entry kInternalDataRecorderTrigger_InternalDataRecorderTriggerMode = At(0x31E2, 2);  // Internal data recorder trigger mode
constexpr Entry kInternalDataRecorderTrigger_InternalDataRecorderTriggerHighValue = At(0x31E2, 3);  // Internal data recorder trigger high value
constexpr Entry kInternalDataRecorderTrigger_InternalDataRecorderTriggerLowValue = At(0x31E2, 4);  // Internal data recorder trigger low value
constexpr Entry kInternalDataRecorderTrigger_InternalDataRecorderTriggerMask = At(0x31E2, 5);  // Internal data recorder trigger mask
constexpr Entry kInternalDataRecorderTrigger_InternalDataRecorderTriggerNumberOfPrecedingSamples =
  At(0x31E2, 6);                                                                                                   // Internal data recorder trigger number of preceding samples
constexpr std::uint16_t kInternalDataRecorderControl = 0x31E4;
constexpr Entry kInternalDataRecorderControl_InternalDataRecorderControlword = At(0x31E4, 1);  // Internal data recorder controlword
constexpr Entry kInternalDataRecorderControl_InternalDataRecorderStatusword = At(0x31E4, 2);  // Internal data recorder statusword
constexpr std::uint16_t kInternalDataRecorderData = 0x31E5;
constexpr Entry kInternalDataRecorderData_InternalDataRecorderNumberOfRecordedSamples =
  At(0x31E5, 1);                                                                                        // Internal data recorder number of recorded samples
constexpr Entry kInternalDataRecorderData_InternalRecorderBufferTimestamp = At(0x31E5, 2);  // Internal recorder buffer timestamp
constexpr Entry kInternalDataRecorderData_InternalDataRecorderSnapshot = At(0x31E5, 3);  // Internal data recorder snapshot
constexpr std::uint16_t kInternalDataRecorderBufferVariable1 = 0x31E6;
constexpr Entry kInternalDataRecorderBufferVariable1_InternalDataRecorderBufferVariable1Complete =
  At(0x31E6, 1);                                                                                                   // Internal data recorder buffer variable 1 complete
constexpr Entry kInternalDataRecorderBufferVariable1_InternalDataRecorderBufferVariable1Part1 = At(
  0x31E6, 2);                                                                                                   // Internal data recorder buffer variable 1 part 1
constexpr std::uint16_t kInternalDataRecorderBufferVariable2 = 0x31E7;
constexpr Entry kInternalDataRecorderBufferVariable2_InternalDataRecorderBufferVariable2Complete =
  At(0x31E7, 1);                                                                                                   // Internal data recorder buffer variable 2 complete
constexpr Entry kInternalDataRecorderBufferVariable2_InternalDataRecorderBufferVariable2Part1 = At(
  0x31E7, 2);                                                                                                   // Internal data recorder buffer variable 2 part 1
constexpr std::uint16_t kInternalDataRecorderBufferVariable3 = 0x31E8;
constexpr Entry kInternalDataRecorderBufferVariable3_InternalDataRecorderBufferVariable3Complete =
  At(0x31E8, 1);                                                                                                   // Internal data recorder buffer variable 3 complete
constexpr Entry kInternalDataRecorderBufferVariable3_InternalDataRecorderBufferVariable3Part1 = At(
  0x31E8, 2);                                                                                                   // Internal data recorder buffer variable 3 part 1
constexpr std::uint16_t kInternalDataRecorderBufferVariable4 = 0x31E9;
constexpr Entry kInternalDataRecorderBufferVariable4_InternalDataRecorderBufferVariable4Complete =
  At(0x31E9, 1);                                                                                                   // Internal data recorder buffer variable 4 complete
constexpr Entry kInternalDataRecorderBufferVariable4_InternalDataRecorderBufferVariable4Part1 = At(
  0x31E9, 2);                                                                                                   // Internal data recorder buffer variable 4 part 1
constexpr std::uint16_t kPowerLimitation = 0x3200;
constexpr Entry kPowerLimitation_I2tLevelMotor = At(0x3200, 1);  // I2t level motor
constexpr Entry kPowerLimitation_I2tLevelPowerStage = At(0x3200, 2);  // I2t level power stage
constexpr std::uint16_t kThermalOverloadProtection = 0x3201;
constexpr Entry kThermalOverloadProtection_TemperaturePowerStage = At(0x3201, 1);  // Temperature power stage
constexpr Entry kThermalOverloadProtection_MaximalTemperaturePowerStage = At(0x3201, 4);  // Maximal temperature power stage
constexpr std::uint16_t kFunctionalSafety = 0x3202;
constexpr Entry kFunctionalSafety_STOInputStates = At(0x3202, 1);  // STO input states
constexpr std::uint16_t kMotorControl = 0x3203;
constexpr Entry kMotorControl_PWMDutyCycleActualValue = At(0x3203, 1);  // PWM duty cycle actual value
constexpr std::uint16_t kInternalTestObject = 0x3240;
constexpr Entry kInternalTestObject_InternalSpiExtensionTest = At(0x3240, 1);  // Internal spi extension test
constexpr Entry kInternalTestObject_InternalADCEncoder2ChannelAAnalogValue = At(0x3240, 2);  // Internal ADC encoder 2 channel A analog value
constexpr Entry kInternalTestObject_InternalADCEncoder2ChannelBAnalogValue = At(0x3240, 3);  // Internal ADC encoder 2 channel B analog value
constexpr std::uint16_t kInternalMotorControl = 0x3241;
constexpr Entry kInternalMotorControl_InternalADCCurrentOffsetSimultaneousPhaseU = At(0x3241, 1);  // Internal ADC current offset simultaneous phase U
constexpr Entry kInternalMotorControl_InternalADCCurrentOffsetSimultaneousPhaseV = At(0x3241, 2);  // Internal ADC current offset simultaneous phase V
constexpr Entry kInternalMotorControl_InternalADCCurrentOffsetSequentialPhaseU = At(0x3241, 3);  // Internal ADC current offset sequential phase U
constexpr Entry kInternalMotorControl_InternalADCCurrentOffsetSequentialPhaseV = At(0x3241, 4);  // Internal ADC current offset sequential phase V
constexpr Entry kInternalMotorControl_InternalADCCurrentOffsetSequentialPhaseW = At(0x3241, 5);  // Internal ADC current offset sequential phase W
constexpr Entry kInternalMotorControl_InternalElectricalRotorAngle = At(0x3241, 6);  // Internal electrical rotor angle
constexpr std::uint16_t kInternalConfigurationElectricalSystemTuning = 0x3248;
constexpr Entry kInternalConfigurationElectricalSystemTuning_InternalLRIdentificationCurrent = At(
  0x3248, 1);                                                                                                  // Internal LR identification current
constexpr Entry kInternalConfigurationElectricalSystemTuning_InternalLRIdentificationFrequency = At(
  0x3248, 2);                                                                                                    // Internal LR identification frequency
constexpr std::uint16_t kInternalParametersElectricalSystemTuning = 0x3249;
constexpr Entry kInternalParametersElectricalSystemTuning_InternalResistance = At(0x3249, 1);  // Internal resistance
constexpr Entry kInternalParametersElectricalSystemTuning_InternalInductance = At(0x3249, 2);  // Internal inductance
constexpr Entry kInternalParametersElectricalSystemTuning_InternalFrequency = At(0x3249, 3);  // Internal frequency
constexpr std::uint16_t kInternalConfigurationMechanicalSystemTuning = 0x324B;
constexpr Entry kInternalConfigurationMechanicalSystemTuning_InternalCutOffFrequencyOfOscillation =
  At(0x324B, 1);                                                                                                    // Internal cut-off frequency of oscillation
constexpr Entry kInternalConfigurationMechanicalSystemTuning_InternalTargetAmplitudeOfOscillation =
  At(0x324B, 2);                                                                                                    // Internal target amplitude of oscillation
constexpr Entry kInternalConfigurationMechanicalSystemTuning_InternalIdentifyKm = At(0x324B, 3);  // Internal identify Km
constexpr Entry
  kInternalConfigurationMechanicalSystemTuning_InternalProportionalGainForDynamicFriction = At(
  0x324B, 4);                                                                                                             // Internal proportional gain for dynamic friction
constexpr Entry kInternalConfigurationMechanicalSystemTuning_InternalIntegralGainForDynamicFriction
  =
  At(0x324B, 5);                                                                                                      // Internal integral gain for dynamic friction
constexpr Entry kInternalConfigurationMechanicalSystemTuning_InternalAuxiliaryTargetVelocity = At(
  0x324B, 32);                                                                                                  // Internal auxiliary target velocity
constexpr Entry kInternalConfigurationMechanicalSystemTuning_InternalAuxiliaryProfileAcceleration =
  At(0x324B, 33);                                                                                                    // Internal auxiliary profile acceleration
constexpr Entry kInternalConfigurationMechanicalSystemTuning_InternalAuxiliaryProfileDeceleration =
  At(0x324B, 34);                                                                                                    // Internal auxiliary profile deceleration
constexpr std::uint16_t kInternalParametersMechanicalSystemTuning = 0x324C;
constexpr Entry kInternalParametersMechanicalSystemTuning_InternalCycleDuration = At(0x324C, 1);  // Internal cycle duration
constexpr Entry kInternalParametersMechanicalSystemTuning_InternalSetValueOfCurrent = At(0x324C, 2);  // Internal set value of current
constexpr Entry kInternalParametersMechanicalSystemTuning_InternalActualPositionPeakValue = At(
  0x324C, 3);                                                                                               // Internal actual position peak value
constexpr Entry kInternalParametersMechanicalSystemTuning_InternalActualValueOfDynamicFriction = At(
  0x324C, 4);                                                                                                    // Internal actual value of dynamic friction
constexpr Entry kInternalParametersMechanicalSystemTuning_InternalExtraDelay = At(0x324C, 6);  // Internal extra delay
constexpr Entry kInternalParametersMechanicalSystemTuning_InternalAuxiliaryVelocityDemandValue = At(
  0x324C, 32);                                                                                                    // Internal auxiliary velocity demand value
constexpr Entry
  kInternalParametersMechanicalSystemTuning_InternalAuxiliaryVelocityActualValueAveraged = At(
  0x324C, 33);                                                                                                            // Internal auxiliary velocity actual value averaged
constexpr std::uint16_t kInternalPRBSVelocityTuningConfiguration = 0x324D;
constexpr Entry kInternalPRBSVelocityTuningConfiguration_InternalPRBSVelocityTuningStepAmplitude =
  At(0x324D, 1);                                                                                                   // Internal PRBS velocity tuning step amplitude
constexpr Entry kInternalPRBSVelocityTuningConfiguration_InternalPRBSVelocityTuningCycleTime = At(
  0x324D, 2);                                                                                                  // Internal PRBS velocity tuning cycle time
constexpr Entry kInternalPRBSVelocityTuningConfiguration_InternalPRBSVelocityTuningTargetAmplitude =
  At(0x324D, 3);                                                                                                     // Internal PRBS velocity tuning target amplitude
constexpr Entry
  kInternalPRBSVelocityTuningConfiguration_InternalPRBSVelocityTuningMaxCycleTimeFactor = At(
  0x324D, 4);                                                                                                           // Internal PRBS velocity tuning max cycle time factor
constexpr std::uint16_t kInternalPRBSVelocityTuningParameter = 0x324E;
constexpr Entry kInternalPRBSVelocityTuningParameter_InternalPRBSVelocityTuningExcitationAmplitude =
  At(0x324E, 1);                                                                                                     // Internal PRBS velocity tuning excitation amplitude
constexpr std::uint16_t kInternalPRBSVelocityTuningPositionValues = 0x324F;
constexpr Entry kInternalPRBSVelocityTuningPositionValues_InternalPRBSVelocityTuningPositionValues1
  =
  At(0x324F, 1);                                                                                                      // Internal PRBS velocity tuning position values 1
constexpr Entry kInternalPRBSVelocityTuningPositionValues_InternalPRBSVelocityTuningPositionValues2
  =
  At(0x324F, 2);                                                                                                      // Internal PRBS velocity tuning position values 2
constexpr Entry kInternalPRBSVelocityTuningPositionValues_InternalPRBSVelocityTuningPositionValues3
  =
  At(0x324F, 3);                                                                                                      // Internal PRBS velocity tuning position values 3
constexpr Entry kInternalPRBSVelocityTuningPositionValues_InternalPRBSVelocityTuningPositionValues4
  =
  At(0x324F, 4);                                                                                                      // Internal PRBS velocity tuning position values 4
constexpr Entry kInternalPRBSVelocityTuningPositionValues_InternalPRBSVelocityTuningPositionValues5
  =
  At(0x324F, 5);                                                                                                      // Internal PRBS velocity tuning position values 5
constexpr Entry kInternalPRBSVelocityTuningPositionValues_InternalPRBSVelocityTuningPositionValues6
  =
  At(0x324F, 6);                                                                                                      // Internal PRBS velocity tuning position values 6
constexpr Entry kInternalPRBSVelocityTuningPositionValues_InternalPRBSVelocityTuningPositionValues7
  =
  At(0x324F, 7);                                                                                                      // Internal PRBS velocity tuning position values 7
constexpr std::uint16_t kInternalPRBSVelocityTuningExcitationOffset = 0x3250;
constexpr Entry
  kInternalPRBSVelocityTuningExcitationOffset_InternalPRBSVelocityTuningExcitationOffset1 = At(
  0x3250, 1);                                                                                                             // Internal PRBS velocity tuning excitation offset 1
constexpr Entry
  kInternalPRBSVelocityTuningExcitationOffset_InternalPRBSVelocityTuningExcitationOffset2 = At(
  0x3250, 2);                                                                                                             // Internal PRBS velocity tuning excitation offset 2
constexpr Entry
  kInternalPRBSVelocityTuningExcitationOffset_InternalPRBSVelocityTuningExcitationOffset3 = At(
  0x3250, 3);                                                                                                             // Internal PRBS velocity tuning excitation offset 3
}  // namespace maxon

// ---------------------------------------------------------------------------
// Drive profile (CiA 402) - 58 objects
// ---------------------------------------------------------------------------
namespace cia402
{
constexpr std::uint16_t kAbortConnectionOptionCode = 0x6007;
constexpr std::uint16_t kErrorCode = 0x603F;
constexpr std::uint16_t kControlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kQuickStopOptionCode = 0x605A;
constexpr std::uint16_t kShutdownOptionCode = 0x605B;
constexpr std::uint16_t kDisableOperationOptionCode = 0x605C;
constexpr std::uint16_t kFaultReactionOptionCode = 0x605E;
constexpr std::uint16_t kModesOfOperation = 0x6060;
constexpr std::uint16_t kModesOfOperationDisplay = 0x6061;
constexpr std::uint16_t kPositionDemandValue = 0x6062;
constexpr std::uint16_t kPositionActualValue = 0x6064;
constexpr std::uint16_t kFollowingErrorWindow = 0x6065;
constexpr std::uint16_t kFollowingErrorTimeOut = 0x6066;
constexpr std::uint16_t kPositionWindow = 0x6067;
constexpr std::uint16_t kPositionWindowTime = 0x6068;
constexpr std::uint16_t kVelocityDemandValue = 0x606B;
constexpr std::uint16_t kVelocityActualValue = 0x606C;
constexpr std::uint16_t kTargetTorque = 0x6071;
constexpr std::uint16_t kMotorRatedTorque = 0x6076;
constexpr std::uint16_t kTorqueActualValue = 0x6077;
constexpr std::uint16_t kTargetPosition = 0x607A;
constexpr std::uint16_t kPositionRangeLimit = 0x607B;
constexpr Entry kPositionRangeLimit_MinPositionRangeLimit = At(0x607B, 1);  // Min position range limit
constexpr Entry kPositionRangeLimit_MaxPositionRangeLimit = At(0x607B, 2);  // Max position range limit
constexpr std::uint16_t kSoftwarePositionLimit = 0x607D;
constexpr Entry kSoftwarePositionLimit_MinPositionLimit = At(0x607D, 1);  // Min position limit
constexpr Entry kSoftwarePositionLimit_MaxPositionLimit = At(0x607D, 2);  // Max position limit
constexpr std::uint16_t kMaxProfileVelocity = 0x607F;
constexpr std::uint16_t kMaxMotorSpeed = 0x6080;
constexpr std::uint16_t kProfileVelocity = 0x6081;
constexpr std::uint16_t kProfileAcceleration = 0x6083;
constexpr std::uint16_t kProfileDeceleration = 0x6084;
constexpr std::uint16_t kQuickStopDeceleration = 0x6085;
constexpr std::uint16_t kMotionProfileType = 0x6086;
constexpr std::uint16_t kHomingMethod = 0x6098;
constexpr std::uint16_t kHomingSpeeds = 0x6099;
constexpr Entry kHomingSpeeds_SpeedForSwitchSearch = At(0x6099, 1);  // Speed for switch search
constexpr Entry kHomingSpeeds_SpeedForZeroSearch = At(0x6099, 2);  // Speed for zero search
constexpr std::uint16_t kHomingAcceleration = 0x609A;
constexpr std::uint16_t kSIUnitPosition = 0x60A8;
constexpr std::uint16_t kSIUnitVelocity = 0x60A9;
constexpr std::uint16_t kSIUnitAcceleration = 0x60AA;
constexpr std::uint16_t kPositionOffset = 0x60B0;
constexpr std::uint16_t kVelocityOffset = 0x60B1;
constexpr std::uint16_t kTorqueOffset = 0x60B2;
constexpr std::uint16_t kTouchProbeFunction = 0x60B8;
constexpr std::uint16_t kTouchProbeStatus = 0x60B9;
constexpr std::uint16_t kTouchProbe1PositiveEdge = 0x60BA;
constexpr std::uint16_t kTouchProbe1NegativeEdge = 0x60BB;
constexpr std::uint16_t kInterpolationTimePeriod = 0x60C2;
constexpr Entry kInterpolationTimePeriod_InterpolationTimePeriodValue = At(0x60C2, 1);  // Interpolation time period value
constexpr Entry kInterpolationTimePeriod_InterpolationTimeIndex = At(0x60C2, 2);  // Interpolation time index
constexpr std::uint16_t kMaxAcceleration = 0x60C5;
constexpr std::uint16_t kTouchProbeSource = 0x60D0;
constexpr Entry kTouchProbeSource_TouchProbe1Source = At(0x60D0, 1);  // Touch probe 1 source
constexpr std::uint16_t kTouchProbe1PositiveEdgeCounter = 0x60D5;
constexpr std::uint16_t kTouchProbe1NegativeEdgeCounter = 0x60D6;
constexpr std::uint16_t kSupportedHomingMethods = 0x60E3;
constexpr Entry kSupportedHomingMethods_N1stSupportedHomingMethod = At(0x60E3, 1);  // 1st supported homing method
constexpr Entry kSupportedHomingMethods_N2ndSupportedHomingMethod = At(0x60E3, 2);  // 2nd supported homing method
constexpr Entry kSupportedHomingMethods_N3rdSupportedHomingMethod = At(0x60E3, 3);  // 3rd supported homing method
constexpr Entry kSupportedHomingMethods_N4thSupportedHomingMethod = At(0x60E3, 4);  // 4th supported homing method
constexpr Entry kSupportedHomingMethods_N5thSupportedHomingMethod = At(0x60E3, 5);  // 5th supported homing method
constexpr Entry kSupportedHomingMethods_N6thSupportedHomingMethod = At(0x60E3, 6);  // 6th supported homing method
constexpr Entry kSupportedHomingMethods_N7thSupportedHomingMethod = At(0x60E3, 7);  // 7th supported homing method
constexpr Entry kSupportedHomingMethods_N8thSupportedHomingMethod = At(0x60E3, 8);  // 8th supported homing method
constexpr Entry kSupportedHomingMethods_N9thSupportedHomingMethod = At(0x60E3, 9);  // 9th supported homing method
constexpr Entry kSupportedHomingMethods_N10thSupportedHomingMethod = At(0x60E3, 10);  // 10th supported homing method
constexpr Entry kSupportedHomingMethods_N11thSupportedHomingMethod = At(0x60E3, 11);  // 11th supported homing method
constexpr Entry kSupportedHomingMethods_N12thSupportedHomingMethod = At(0x60E3, 12);  // 12th supported homing method
constexpr Entry kSupportedHomingMethods_N13thSupportedHomingMethod = At(0x60E3, 13);  // 13th supported homing method
constexpr Entry kSupportedHomingMethods_N14thSupportedHomingMethod = At(0x60E3, 14);  // 14th supported homing method
constexpr Entry kSupportedHomingMethods_N15thSupportedHomingMethod = At(0x60E3, 15);  // 15th supported homing method
constexpr std::uint16_t kAdditionalPositionActualValues = 0x60E4;
constexpr Entry kAdditionalPositionActualValues_PositionActualValueSensor1 = At(0x60E4, 1);  // Position actual value sensor 1
constexpr Entry kAdditionalPositionActualValues_PositionActualValueSensor2 = At(0x60E4, 2);  // Position actual value sensor 2
constexpr Entry kAdditionalPositionActualValues_PositionActualValueSensor3 = At(0x60E4, 3);  // Position actual value sensor 3
constexpr std::uint16_t kAdditionalVelocityActualValues = 0x60E5;
constexpr Entry kAdditionalVelocityActualValues_VelocityActualValueSensor1 = At(0x60E5, 1);  // Velocity actual value sensor 1
constexpr Entry kAdditionalVelocityActualValues_VelocityActualValueSensor2 = At(0x60E5, 2);  // Velocity actual value sensor 2
constexpr Entry kAdditionalVelocityActualValues_VelocityActualValueSensor3 = At(0x60E5, 3);  // Velocity actual value sensor 3
constexpr Entry kAdditionalVelocityActualValues_VelocityActualValueAveragedSensor1 = At(0x60E5, 9);  // Velocity actual value averaged sensor 1
constexpr Entry kAdditionalVelocityActualValues_VelocityActualValueAveragedSensor2 = At(0x60E5, 10);  // Velocity actual value averaged sensor 2
constexpr Entry kAdditionalVelocityActualValues_VelocityActualValueAveragedSensor3 = At(0x60E5, 11);  // Velocity actual value averaged sensor 3
constexpr std::uint16_t kFollowingErrorActualValue = 0x60F4;
constexpr std::uint16_t kDigitalInputs = 0x60FD;
constexpr std::uint16_t kDigitalOutputs = 0x60FE;
constexpr Entry kDigitalOutputs_PhysicalOutputs = At(0x60FE, 1);  // Physical outputs
constexpr std::uint16_t kTargetVelocity = 0x60FF;
constexpr std::uint16_t kMotorType = 0x6402;
constexpr std::uint16_t kSupportedDriveModes = 0x6502;
}  // namespace cia402

}  // namespace epos4::od
