/******************************************************************************
 *
 * INTEL CONFIDENTIAL
 *
 * Copyright (c) 2015-2017 Intel Corporation All Rights Reserved.
 *
 * The source code contained or described herein and all documents related to
 * the source code (Material) are owned by Intel Corporation or its suppliers
 * or licensors. Title to the Material remains with Intel Corporation or its
 * suppliers and licensors. The Material contains trade secrets and proprietary
 * and confidential information of Intel or its suppliers and licensors. The
 * Material is protected by worldwide copyright and trade secret laws and
 * treaty provisions. No part of the Material may be used, copied, reproduced,
 * modified, published, uploaded, posted, transmitted, distributed, or
 * disclosed in any way without Intel's prior express written permission.
 *
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise. Any license under such intellectual property rights must be
 * express and approved by Intel in writing.
 *
 ******************************************************************************/

#ifndef _ACPI_5_0_H_
#define _ACPI_5_0_H_

/* only copy part of Acpi50.h we need */
//
// General use definitions (ACPI_1.0)
//
#define EFI_ACPI_RESERVED_BYTE 0x00
#define EFI_ACPI_RESERVED_WORD 0x0000
#define EFI_ACPI_RESERVED_DWORD 0x00000000
#define EFI_ACPI_RESERVED_QWORD 0x0000000000000000

#pragma pack(1)
///
/// The common ACPI description table header.  This structure prefaces most ACPI
/// tables. (ACPI_1.0)
///
typedef struct {
  UINT32 Signature;
  UINT32 Length;
  UINT8 Revision;
  UINT8 Checksum;
  UINT8 OemId[6];
  UINT64 OemTableId;
  UINT32 OemRevision;
  UINT32 CreatorId;
  UINT32 CreatorRevision;
} EFI_ACPI_DESCRIPTION_HEADER;
#pragma pack()

//
// Ensure proper structure formats
//
#pragma pack(1)

///
/// ACPI 5.0 Generic Address Space definition
///
typedef struct {
  UINT8 AddressSpaceId;
  UINT8 RegisterBitWidth;
  UINT8 RegisterBitOffset;
  UINT8 AccessSize;
  UINT64 Address;
} EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE;

//
// Generic Address Space Address IDs
//
#define EFI_ACPI_5_0_SYSTEM_MEMORY 0
#define EFI_ACPI_5_0_SYSTEM_IO 1
#define EFI_ACPI_5_0_PCI_CONFIGURATION_SPACE 2
#define EFI_ACPI_5_0_EMBEDDED_CONTROLLER 3
#define EFI_ACPI_5_0_SMBUS 4
#define EFI_ACPI_5_0_PLATFORM_COMMUNICATION_CHANNEL 0x0A
#define EFI_ACPI_5_0_FUNCTIONAL_FIXED_HARDWARE 0x7F

//
// Generic Address Space Access Sizes
//
#define EFI_ACPI_5_0_UNDEFINED 0
#define EFI_ACPI_5_0_BYTE 1
#define EFI_ACPI_5_0_WORD 2
#define EFI_ACPI_5_0_DWORD 3
#define EFI_ACPI_5_0_QWORD 4

//
// ACPI 5.0 table structures
//

///
/// Root System Description Pointer Structure
///
typedef struct {
  UINT64 Signature;
  UINT8 Checksum;
  UINT8 OemId[6];
  UINT8 Revision;
  UINT32 RsdtAddress;
  UINT32 Length;
  UINT64 XsdtAddress;
  UINT8 ExtendedChecksum;
  UINT8 Reserved[3];
} EFI_ACPI_5_0_ROOT_SYSTEM_DESCRIPTION_POINTER;

///
/// RSD_PTR Revision (as defined in ACPI 5.0 spec.)
///
#define EFI_ACPI_5_0_ROOT_SYSTEM_DESCRIPTION_POINTER_REVISION                  \
  0x02 ///< ACPISpec (Revision 5.0) says current value is 2

///
/// XSDT Revision (as defined in ACPI 5.0 spec.)
///
#define EFI_ACPI_5_0_EXTENDED_SYSTEM_DESCRIPTION_TABLE_REVISION 0x01

///
/// Fixed ACPI Description Table Structure (FADT)
///
typedef struct {
  EFI_ACPI_DESCRIPTION_HEADER Header;
  UINT32 FirmwareCtrl;
  UINT32 Dsdt;
  UINT8 Reserved0;
  UINT8 PreferredPmProfile;
  UINT16 SciInt;
  UINT32 SmiCmd;
  UINT8 AcpiEnable;
  UINT8 AcpiDisable;
  UINT8 S4BiosReq;
  UINT8 PstateCnt;
  UINT32 Pm1aEvtBlk;
  UINT32 Pm1bEvtBlk;
  UINT32 Pm1aCntBlk;
  UINT32 Pm1bCntBlk;
  UINT32 Pm2CntBlk;
  UINT32 PmTmrBlk;
  UINT32 Gpe0Blk;
  UINT32 Gpe1Blk;
  UINT8 Pm1EvtLen;
  UINT8 Pm1CntLen;
  UINT8 Pm2CntLen;
  UINT8 PmTmrLen;
  UINT8 Gpe0BlkLen;
  UINT8 Gpe1BlkLen;
  UINT8 Gpe1Base;
  UINT8 CstCnt;
  UINT16 PLvl2Lat;
  UINT16 PLvl3Lat;
  UINT16 FlushSize;
  UINT16 FlushStride;
  UINT8 DutyOffset;
  UINT8 DutyWidth;
  UINT8 DayAlrm;
  UINT8 MonAlrm;
  UINT8 Century;
  UINT16 IaPcBootArch;
  UINT8 Reserved1;
  UINT32 Flags;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE ResetReg;
  UINT8 ResetValue;
  UINT8 Reserved2[3];
  UINT64 XFirmwareCtrl;
  UINT64 XDsdt;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE XPm1aEvtBlk;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE XPm1bEvtBlk;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE XPm1aCntBlk;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE XPm1bCntBlk;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE XPm2CntBlk;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE XPmTmrBlk;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE XGpe0Blk;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE XGpe1Blk;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE SleepControlReg;
  EFI_ACPI_5_0_GENERIC_ADDRESS_STRUCTURE SleepStatusReg;
} EFI_ACPI_5_0_FIXED_ACPI_DESCRIPTION_TABLE;

///
/// FADT Version (as defined in ACPI 5.0 spec.)
///
#define EFI_ACPI_5_0_FIXED_ACPI_DESCRIPTION_TABLE_REVISION 0x05

//
// Fixed ACPI Description Table Preferred Power Management Profile
//
#define EFI_ACPI_5_0_PM_PROFILE_UNSPECIFIED 0
#define EFI_ACPI_5_0_PM_PROFILE_DESKTOP 1
#define EFI_ACPI_5_0_PM_PROFILE_MOBILE 2
#define EFI_ACPI_5_0_PM_PROFILE_WORKSTATION 3
#define EFI_ACPI_5_0_PM_PROFILE_ENTERPRISE_SERVER 4
#define EFI_ACPI_5_0_PM_PROFILE_SOHO_SERVER 5
#define EFI_ACPI_5_0_PM_PROFILE_APPLIANCE_PC 6
#define EFI_ACPI_5_0_PM_PROFILE_PERFORMANCE_SERVER 7
#define EFI_ACPI_5_0_PM_PROFILE_TABLET 8

//
// Fixed ACPI Description Table Boot Architecture Flags
// All other bits are reserved and must be set to 0.
//
#define EFI_ACPI_5_0_LEGACY_DEVICES BIT0
#define EFI_ACPI_5_0_8042 BIT1
#define EFI_ACPI_5_0_VGA_NOT_PRESENT BIT2
#define EFI_ACPI_5_0_MSI_NOT_SUPPORTED BIT3
#define EFI_ACPI_5_0_PCIE_ASPM_CONTROLS BIT4
#define EFI_ACPI_5_0_CMOS_RTC_NOT_PRESENT BIT5

//
// Fixed ACPI Description Table Fixed Feature Flags
// All other bits are reserved and must be set to 0.
//
#define EFI_ACPI_5_0_WBINVD BIT0
#define EFI_ACPI_5_0_WBINVD_FLUSH BIT1
#define EFI_ACPI_5_0_PROC_C1 BIT2
#define EFI_ACPI_5_0_P_LVL2_UP BIT3
#define EFI_ACPI_5_0_PWR_BUTTON BIT4
#define EFI_ACPI_5_0_SLP_BUTTON BIT5
#define EFI_ACPI_5_0_FIX_RTC BIT6
#define EFI_ACPI_5_0_RTC_S4 BIT7
#define EFI_ACPI_5_0_TMR_VAL_EXT BIT8
#define EFI_ACPI_5_0_DCK_CAP BIT9
#define EFI_ACPI_5_0_RESET_REG_SUP BIT10
#define EFI_ACPI_5_0_SEALED_CASE BIT11
#define EFI_ACPI_5_0_HEADLESS BIT12
#define EFI_ACPI_5_0_CPU_SW_SLP BIT13
#define EFI_ACPI_5_0_PCI_EXP_WAK BIT14
#define EFI_ACPI_5_0_USE_PLATFORM_CLOCK BIT15
#define EFI_ACPI_5_0_S4_RTC_STS_VALID BIT16
#define EFI_ACPI_5_0_REMOTE_POWER_ON_CAPABLE BIT17
#define EFI_ACPI_5_0_FORCE_APIC_CLUSTER_MODEL BIT18
#define EFI_ACPI_5_0_FORCE_APIC_PHYSICAL_DESTINATION_MODE BIT19
#define EFI_ACPI_5_0_HW_REDUCED_ACPI BIT20
#define EFI_ACPI_5_0_LOW_POWER_S0_IDLE_CAPABLE BIT21

///
/// Firmware ACPI Control Structure
///
typedef struct {
  UINT32 Signature;
  UINT32 Length;
  UINT32 HardwareSignature;
  UINT32 FirmwareWakingVector;
  UINT32 GlobalLock;
  UINT32 Flags;
  UINT64 XFirmwareWakingVector;
  UINT8 Version;
  UINT8 Reserved0[3];
  UINT32 OspmFlags;
  UINT8 Reserved1[24];
} EFI_ACPI_5_0_FIRMWARE_ACPI_CONTROL_STRUCTURE;

///
/// FACS Version (as defined in ACPI 5.0 spec.)
///
#define EFI_ACPI_5_0_FIRMWARE_ACPI_CONTROL_STRUCTURE_VERSION 0x02

///
/// Firmware Control Structure Feature Flags
/// All other bits are reserved and must be set to 0.
///
#define EFI_ACPI_5_0_S4BIOS_F BIT0
#define EFI_ACPI_5_0_64BIT_WAKE_SUPPORTED_F BIT1

///
/// OSPM Enabled Firmware Control Structure Flags
/// All other bits are reserved and must be set to 0.
///
#define EFI_ACPI_5_0_OSPM_64BIT_WAKE_F BIT0

//
// Differentiated System Description Table,
// Secondary System Description Table
// and Persistent System Description Table,
// no definition needed as they are common description table header, the same
// with EFI_ACPI_DESCRIPTION_HEADER, followed by a definition block.
//
#define EFI_ACPI_5_0_DIFFERENTIATED_SYSTEM_DESCRIPTION_TABLE_REVISION 0x02
#define EFI_ACPI_5_0_SECONDARY_SYSTEM_DESCRIPTION_TABLE_REVISION 0x02

///
/// Multiple APIC Description Table header definition.  The rest of the table
/// must be defined in a platform specific manner.
///
typedef struct {
  EFI_ACPI_DESCRIPTION_HEADER Header;
  UINT32 LocalApicAddress;
  UINT32 Flags;
} EFI_ACPI_5_0_MULTIPLE_APIC_DESCRIPTION_TABLE_HEADER;

///
/// MADT Revision (as defined in ACPI 5.0 spec.)
///
#define EFI_ACPI_5_0_MULTIPLE_APIC_DESCRIPTION_TABLE_REVISION 0x03

///
/// Multiple APIC Flags
/// All other bits are reserved and must be set to 0.
///
#define EFI_ACPI_5_0_PCAT_COMPAT BIT0

//
// Multiple APIC Description Table APIC structure types
// All other values between 0x0D and 0x7F are reserved and
// will be ignored by OSPM. 0x80 ~ 0xFF are reserved for OEM.
//
#define EFI_ACPI_5_0_PROCESSOR_LOCAL_APIC 0x00
#define EFI_ACPI_5_0_IO_APIC 0x01
#define EFI_ACPI_5_0_INTERRUPT_SOURCE_OVERRIDE 0x02
#define EFI_ACPI_5_0_NON_MASKABLE_INTERRUPT_SOURCE 0x03
#define EFI_ACPI_5_0_LOCAL_APIC_NMI 0x04
#define EFI_ACPI_5_0_LOCAL_APIC_ADDRESS_OVERRIDE 0x05
#define EFI_ACPI_5_0_IO_SAPIC 0x06
#define EFI_ACPI_5_0_LOCAL_SAPIC 0x07
#define EFI_ACPI_5_0_PLATFORM_INTERRUPT_SOURCES 0x08
#define EFI_ACPI_5_0_PROCESSOR_LOCAL_X2APIC 0x09
#define EFI_ACPI_5_0_LOCAL_X2APIC_NMI 0x0A
#define EFI_ACPI_5_0_GIC 0x0B
#define EFI_ACPI_5_0_GICD 0x0C

//
// APIC Structure Definitions
//

///
/// Processor Local APIC Structure Definition
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT8 AcpiProcessorId;
  UINT8 ApicId;
  UINT32 Flags;
} EFI_ACPI_5_0_PROCESSOR_LOCAL_APIC_STRUCTURE;

///
/// Local APIC Flags.  All other bits are reserved and must be 0.
///
#define EFI_ACPI_5_0_LOCAL_APIC_ENABLED BIT0

///
/// IO APIC Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT8 IoApicId;
  UINT8 Reserved;
  UINT32 IoApicAddress;
  UINT32 GlobalSystemInterruptBase;
} EFI_ACPI_5_0_IO_APIC_STRUCTURE;

///
/// Interrupt Source Override Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT8 Bus;
  UINT8 Source;
  UINT32 GlobalSystemInterrupt;
  UINT16 Flags;
} EFI_ACPI_5_0_INTERRUPT_SOURCE_OVERRIDE_STRUCTURE;

///
/// Platform Interrupt Sources Structure Definition
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT16 Flags;
  UINT8 InterruptType;
  UINT8 ProcessorId;
  UINT8 ProcessorEid;
  UINT8 IoSapicVector;
  UINT32 GlobalSystemInterrupt;
  UINT32 PlatformInterruptSourceFlags;
  UINT8 CpeiProcessorOverride;
  UINT8 Reserved[31];
} EFI_ACPI_5_0_PLATFORM_INTERRUPT_APIC_STRUCTURE;

//
// MPS INTI flags.
// All other bits are reserved and must be set to 0.
//
#define EFI_ACPI_5_0_POLARITY (3 << 0)
#define EFI_ACPI_5_0_TRIGGER_MODE (3 << 2)

///
/// Non-Maskable Interrupt Source Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT16 Flags;
  UINT32 GlobalSystemInterrupt;
} EFI_ACPI_5_0_NON_MASKABLE_INTERRUPT_SOURCE_STRUCTURE;

///
/// Local APIC NMI Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT8 AcpiProcessorId;
  UINT16 Flags;
  UINT8 LocalApicLint;
} EFI_ACPI_5_0_LOCAL_APIC_NMI_STRUCTURE;

///
/// Local APIC Address Override Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT16 Reserved;
  UINT64 LocalApicAddress;
} EFI_ACPI_5_0_LOCAL_APIC_ADDRESS_OVERRIDE_STRUCTURE;

///
/// IO SAPIC Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT8 IoApicId;
  UINT8 Reserved;
  UINT32 GlobalSystemInterruptBase;
  UINT64 IoSapicAddress;
} EFI_ACPI_5_0_IO_SAPIC_STRUCTURE;

///
/// Local SAPIC Structure
/// This struct followed by a null-terminated ASCII string - ACPI Processor UID
/// String
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT8 AcpiProcessorId;
  UINT8 LocalSapicId;
  UINT8 LocalSapicEid;
  UINT8 Reserved[3];
  UINT32 Flags;
  UINT32 ACPIProcessorUIDValue;
} EFI_ACPI_5_0_PROCESSOR_LOCAL_SAPIC_STRUCTURE;

///
/// Platform Interrupt Sources Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT16 Flags;
  UINT8 InterruptType;
  UINT8 ProcessorId;
  UINT8 ProcessorEid;
  UINT8 IoSapicVector;
  UINT32 GlobalSystemInterrupt;
  UINT32 PlatformInterruptSourceFlags;
} EFI_ACPI_5_0_PLATFORM_INTERRUPT_SOURCES_STRUCTURE;

///
/// Platform Interrupt Source Flags.
/// All other bits are reserved and must be set to 0.
///
#define EFI_ACPI_5_0_CPEI_PROCESSOR_OVERRIDE BIT0

///
/// Processor Local x2APIC Structure Definition
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT8 Reserved[2];
  UINT32 X2ApicId;
  UINT32 Flags;
  UINT32 AcpiProcessorUid;
} EFI_ACPI_5_0_PROCESSOR_LOCAL_X2APIC_STRUCTURE;

///
/// Local x2APIC NMI Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT16 Flags;
  UINT32 AcpiProcessorUid;
  UINT8 LocalX2ApicLint;
  UINT8 Reserved[3];
} EFI_ACPI_5_0_LOCAL_X2APIC_NMI_STRUCTURE;

///
/// GIC Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT16 Reserved;
  UINT32 GicId;
  UINT32 AcpiProcessorUid;
  UINT32 Flags;
  UINT32 ParkingProtocolVersion;
  UINT32 PerformanceInterruptGsiv;
  UINT64 ParkedAddress;
  UINT64 PhysicalBaseAddress;
} EFI_ACPI_5_0_GIC_STRUCTURE;

///
/// GIC Flags.  All other bits are reserved and must be 0.
///
#define EFI_ACPI_5_0_GIC_ENABLED BIT0
#define EFI_ACPI_5_0_PERFORMANCE_INTERRUPT_MODEL BIT1

///
/// GIC Distributor Structure
///
typedef struct {
  UINT8 Type;
  UINT8 Length;
  UINT16 Reserved1;
  UINT32 GicId;
  UINT64 PhysicalBaseAddress;
  UINT32 SystemVectorBase;
  UINT32 Reserved2;
} EFI_ACPI_5_0_GIC_DISTRIBUTOR_STRUCTURE;

#pragma pack()

//
// Known table signatures
//

///
/// "RSD PTR " Root System Description Pointer
///
#define EFI_ACPI_5_0_ROOT_SYSTEM_DESCRIPTION_POINTER_SIGNATURE                 \
  SIGNATURE_64('R', 'S', 'D', ' ', 'P', 'T', 'R', ' ')

///
/// "APIC" Multiple APIC Description Table
///
#define EFI_ACPI_5_0_MULTIPLE_APIC_DESCRIPTION_TABLE_SIGNATURE                 \
  SIGNATURE_32('A', 'P', 'I', 'C')

///
/// "DSDT" Differentiated System Description Table
///
#define EFI_ACPI_5_0_DIFFERENTIATED_SYSTEM_DESCRIPTION_TABLE_SIGNATURE         \
  SIGNATURE_32('D', 'S', 'D', 'T')

///
/// "FACP" Fixed ACPI Description Table
///
#define EFI_ACPI_5_0_FIXED_ACPI_DESCRIPTION_TABLE_SIGNATURE                    \
  SIGNATURE_32('F', 'A', 'C', 'P')

///
/// "FACS" Firmware ACPI Control Structure
///
#define EFI_ACPI_5_0_FIRMWARE_ACPI_CONTROL_STRUCTURE_SIGNATURE                 \
  SIGNATURE_32('F', 'A', 'C', 'S')

///
/// "HPET" IA-PC High Precision Event Timer Table
///
#define EFI_ACPI_5_0_HIGH_PRECISION_EVENT_TIMER_TABLE_SIGNATURE                \
  SIGNATURE_32('H', 'P', 'E', 'T')

///
/// "MCFG" PCI Express Memory Mapped Configuration Space Base Address
/// Description Table
///
#define EFI_ACPI_5_0_PCI_EXPRESS_MEMORY_MAPPED_CONFIGURATION_SPACE_BASE_ADDRESS_DESCRIPTION_TABLE_SIGNATURE \
  SIGNATURE_32('M', 'C', 'F', 'G')

///
/// "XSDT" Extended System Description Table
///
#define EFI_ACPI_5_0_EXTENDED_SYSTEM_DESCRIPTION_TABLE_SIGNATURE               \
  SIGNATURE_32('X', 'S', 'D', 'T')

///
/// "TPM2" Trusted Computing Platform 1 Table
///
#define EFI_ACPI_5_0_TRUSTED_COMPUTING_PLATFORM_2_TABLE_SIGNATURE              \
  SIGNATURE_32('T', 'P', 'M', '2')

///
/// Non-HDA Link Table
///
#define EFI_ACPI_5_0_NON_HIGH_DEFINITION_AUDIO_LINK_TABLE_SIGNATURE            \
  SIGNATURE_32('N', 'H', 'L', 'T')

///
/// "SSDT" Secondary System Description Table
///
#define EFI_ACPI_5_0_SECONDARY_SYSTEM_DESCRIPTION_TABLE_SIGNATURE              \
  SIGNATURE_32('S', 'S', 'D', 'T')

#endif /* ACPI_5_0_H_ */
