/** @file
  Copyright (C) 2016 - 2017, The HermitCrabs Lab. All rights reserved.

  All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
**/

#include <Uefi.h>

#include <IndustryStandard/Pci.h>

#include <Protocol/PciIo.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/UefiBootServicesTableLib.h>

#define XHC_HCCPARAMS_OFFSET      0x10
#define XHC_NEXT_CAPABILITY_MASK    0xFF00
#define XHC_CAPABILITY_ID_MASK    0xFF
#define XHC_USBCMD_OFFSET         0x00    ///< USB Command Register Offset
#define XHC_USBSTS_OFFSET         0x04    ///< USB Status Register Offset
#define XHC_POLL_DELAY            1000

#define EHC_BAR_INDEX             0x00
#define EHC_HCCPARAMS_OFFSET      0x08
#define EHC_USBCMD_OFFSET         0x00    ///< USB Command Register Offset
#define EHC_USBSTS_OFFSET         0x04    ///< USB Status Register Offset
#define EHC_USBINT_OFFSET         0x08    ///< USB Interrupt Enable Register

#define UHC_LEGACY_REGISTER       0xC0

#define CONTROLLED_BY_BIOS        BIT16
#define CONTROLLED_BY_OS          BIT24

/**
  Disable USB Legacy Emulation on XHCI USB controller

  @param[in]  PciIo  PCI I/O protocol for the device
**/
STATIC
VOID
XhciTurnOffUsbEmulation (
  IN  EFI_PCI_IO_PROTOCOL   *PciIo
  )
{
  EFI_STATUS  Status;

  UINT32  HcCapParams;
  UINT32  ExtendCap;
  UINT32  Value;
  INT32   TimeOut;

  //
  // Start search for USB Legacy Support Capability Register
  //

  Status = PciIo->Mem.Read (
    PciIo,
    EfiPciIoWidthUint32,
    XHC_USBCMD_OFFSET,
    XHC_HCCPARAMS_OFFSET,
    1,
    &HcCapParams
    );

  ExtendCap = EFI_ERROR (Status) ? 0 : ((HcCapParams >> 14U) & 0x03FFFCU);

  while (ExtendCap) {
    Status = PciIo->Mem.Read (
      PciIo,
      EfiPciIoWidthUint32,
      XHC_USBCMD_OFFSET,
      ExtendCap,
      1,
      &Value
      );

    if (EFI_ERROR (Status)) {
      break;
    }

    if ((Value & XHC_CAPABILITY_ID_MASK) == 1) {
      // USBLEGSUP register found
      // Check current state
      // Do nothing if there is no legacy emulation on device
      //
      if (!(Value & CONTROLLED_BY_BIOS)) {
        break;
      }

      Value |= CONTROLLED_BY_OS;

      PciIo->Mem.Write (
        PciIo,
        EfiPciIoWidthUint32,
        XHC_USBCMD_OFFSET,
        ExtendCap,
        1,
        &Value
        );

      TimeOut = 40;
      while (TimeOut--) {
        gBS->Stall (500);

        Status = PciIo->Mem.Read (
          PciIo,
          EfiPciIoWidthUint32,
          XHC_USBCMD_OFFSET,
          ExtendCap,
          1,
          &Value
          );

        if (EFI_ERROR(Status) || !(Value & CONTROLLED_BY_BIOS)) {
          break;
        }
      }

      //
      // Dismiss all interrupts in USBLEGCTLSTS
      //
      Status = PciIo->Mem.Read (
        PciIo,
        EfiPciIoWidthUint32,
        XHC_USBCMD_OFFSET,
        ExtendCap + 4,
        1,
        &Value
        );

      if (EFI_ERROR(Status)) {
        break;
      }

      Value &= 0x001F1FEEU;
      Value |= 0xE0000000U;

      PciIo->Mem.Write (
        PciIo,
        EfiPciIoWidthUint32,
        XHC_USBCMD_OFFSET,
        ExtendCap + 4,
        1,
        &Value
        );

      //
      // XXX: Clear all ownership
      //
      Status = PciIo->Mem.Read (
        PciIo,
        EfiPciIoWidthUint32,
        XHC_USBCMD_OFFSET,
        ExtendCap,
        1,
        &Value
        );

      if (EFI_ERROR(Status)) {
        break;
      }

      Value &= ~(CONTROLLED_BY_BIOS | CONTROLLED_BY_OS);
      PciIo->Mem.Write (
        PciIo,
        EfiPciIoWidthUint32,
        XHC_USBCMD_OFFSET,
        ExtendCap,
        1,
        &Value
        );

      break;
    }

    if (!(Value & XHC_NEXT_CAPABILITY_MASK)) {
      break;
    }

    ExtendCap += ((Value >> 6U) & 0x03FCU);
  }
}

/**
  Disable USB Legacy Emulation on EHCI USB controller

  @param[in]  PciIo  PCI I/O protocol for the device
 **/
STATIC
VOID
EhciTurnOffUsbEmulation (
  IN  EFI_PCI_IO_PROTOCOL   *PciIo
  )
{
  EFI_STATUS       Status;
  UINT32           Value;
  UINT32           Base;
  UINT32           OpAddr;
  UINT32           ExtendCap;
  UINT32           UsbCmd;
  UINT32           UsbLegSup;
  UINT32           UsbLegCtlSts;
  UINTN            IsOsOwned;
  UINTN            IsBiosOwned;
  BOOLEAN          IsOwnershipConflict;
  UINT32           HcCapParams;
  INT32            TimeOut;

  // Hit hardware with sledgehammer

  Value = 0x0002;

  PciIo->Pci.Write (
    PciIo,
    EfiPciIoWidthUint16,
    0x04,
    1,
    &Value
    );

  Base = 0;

  Status = PciIo->Pci.Read (
    PciIo,
    EfiPciIoWidthUint32,
    0x10,
    1,
    &Base
    );

  if (MmioRead8 (Base) < 0x0C) {
    //
    // Config space too small: no legacy support.
    //
    return;
  }

  //
  // Operational Registers = capaddr + offset (8bit CAPLENGTH in Capability Registers + offset 0).
  //
  OpAddr = Base + MmioRead8 (Base);

  Status = PciIo->Mem.Read (
    PciIo,
    EfiPciIoWidthUint32,
    EHC_BAR_INDEX,
    EHC_HCCPARAMS_OFFSET,
    1,
    &HcCapParams
    );

  ExtendCap = (HcCapParams >> 8U) & 0xFFU;

  //
  // Read PCI Config 32bit USBLEGSUP (eecp+0).
  //
  Status = PciIo->Pci.Read (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap,
    1,
    &UsbLegSup
    );

  IsBiosOwned = (UsbLegSup & CONTROLLED_BY_BIOS) != 0;
  if (!IsBiosOwned) {
    //
    // No legacy emulation on device
    //
    return;
  }

  //
  // Read PCI Config 32bit USBLEGCTLSTS (eecp+4).
  //
  PciIo->Pci.Read (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap + 0x4,
    1,
    &UsbLegCtlSts
    );

  //
  // Disable the SMI in USBLEGCTLSTS firstly.
  //
  UsbLegCtlSts &= 0xFFFF0000U;
  PciIo->Pci.Write (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap + 0x4,
    1,
    &UsbLegCtlSts
    );

  UsbCmd  = MmioRead32 (OpAddr + EHC_USBCMD_OFFSET);

  //
  // Set registers to default.
  //
  UsbCmd = UsbCmd & 0xFFFFFF00U;
  MmioWrite32 (OpAddr + EHC_USBCMD_OFFSET, UsbCmd);
  MmioWrite32 (OpAddr + EHC_USBINT_OFFSET, 0);
  MmioWrite32 (OpAddr + EHC_USBSTS_OFFSET, 0x1000);

  Value = 1;
  PciIo->Pci.Write (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap,
    1,
    &Value
    );

  //
  // Read 32bit USBLEGSUP (eecp+0).
  //
  PciIo->Pci.Read (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap,
    1,
    &UsbLegSup
    );

  IsBiosOwned = (UsbLegSup & CONTROLLED_BY_BIOS) != 0;
  IsOsOwned   = (UsbLegSup & CONTROLLED_BY_OS) != 0;

  //
  // Read 32bit USBLEGCTLSTS (eecp+4).
  //
  PciIo->Pci.Read (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap + 0x4,
    1,
    &UsbLegCtlSts
    );

  //
  // Get current emulation state
  //
  PciIo->Pci.Read (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap,
    1,
    &UsbLegSup
    );

  IsOwnershipConflict = IsBiosOwned && IsOsOwned;

  if (IsOwnershipConflict) {
    //
    // Device control conflict - attempting soft reset.
    //
    Value = 0;
    PciIo->Pci.Write (
      PciIo,
      EfiPciIoWidthUint8,
      ExtendCap + 3,
      1,
      &Value
      );

    TimeOut = 40;
    while (TimeOut--) {
      gBS->Stall (500);

      PciIo->Pci.Read (
        PciIo,
        EfiPciIoWidthUint32,
        ExtendCap,
        1,
        &Value
        );

      if ((Value & CONTROLLED_BY_OS) == 0) {
        break;
      }
    }
  }

  PciIo->Pci.Read (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap,
    1,
    &Value
    );

  Value |= CONTROLLED_BY_OS;
  PciIo->Pci.Write (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCap,
    1,
    &Value
    );

  TimeOut = 40;
  while (TimeOut--) {
    gBS->Stall (500);

    PciIo->Pci.Read (
      PciIo,
      EfiPciIoWidthUint32,
      ExtendCap,
      1,
      &Value
      );

    if ((Value & CONTROLLED_BY_BIOS) == 0x0) {
      break;
    }
  }

  IsOwnershipConflict = (Value & CONTROLLED_BY_BIOS) != 0x0;
  if (IsOwnershipConflict) {
    //
    // Soft reset has failed. Assume SMI being ignored and do hard reset.
    //
    Value = 0;
    PciIo->Pci.Write (
      PciIo,
      EfiPciIoWidthUint8,
      ExtendCap + 2,
      1,
      &Value
      );

    TimeOut = 40;
    while (TimeOut--) {
      gBS->Stall (500);

      PciIo->Pci.Read (
        PciIo,
        EfiPciIoWidthUint32,
        ExtendCap,
        1,
        &Value
        );

      if ((Value & CONTROLLED_BY_BIOS) == 0x0) {
        break;
      }
    }

    //
    // Disable further SMI events.
    //
    PciIo->Pci.Read (
      PciIo,
      EfiPciIoWidthUint32,
      ExtendCap + 0x4,
      1,
      &UsbLegCtlSts
      );

    UsbLegCtlSts &= 0xFFFF0000U;
    PciIo->Pci.Write (
      PciIo,
      EfiPciIoWidthUint32,
      ExtendCap + 0x4,
      1,
      &UsbLegCtlSts
      );
  }
}

/**
  Disable USB Legacy Emulation on UHCI USB controller

  @param[in]  PciIo  PCI I/O protocol for the device
 **/

STATIC
VOID
UhciTurnOffUsbEmulation (
  IN  EFI_PCI_IO_PROTOCOL   *PciIo
  )
{
  UINT16      Command;

  Command = 0;

  (VOID) PciIo->Pci.Write (
    PciIo,
    EfiPciIoWidthUint16,
    UHC_LEGACY_REGISTER,
    1,
    &Command
    );
}

VOID
TurnOffUsbEmulation (
  VOID
  )
{
  EFI_STATUS            Status;
  EFI_HANDLE            *HandleArray;
  UINTN                 HandleArrayCount;
  UINTN                 Index;
  EFI_PCI_IO_PROTOCOL   *PciIo;
  PCI_TYPE00            Pci;

  Status = gBS->LocateHandleBuffer (
                    ByProtocol,
                    &gEfiPciIoProtocolGuid,
                    NULL,
                    &HandleArrayCount,
                    &HandleArray
                    );

  if (EFI_ERROR (Status)) {
    return;
  }

  for (Index = 0; Index < HandleArrayCount; ++Index) {
    Status = gBS->HandleProtocol (
      HandleArray[Index],
      &gEfiPciIoProtocolGuid,
      (VOID **) &PciIo
      );

    if (EFI_ERROR (Status)) {
      continue;
    }

    Status = PciIo->Pci.Read (
      PciIo,
      EfiPciIoWidthUint32,
      0,
      sizeof (Pci) / sizeof (UINT32),
      &Pci
      );

    if (EFI_ERROR (Status) || !IS_PCI_USB (&Pci)) {
      continue;
    }

    switch (Pci.Hdr.ClassCode[0]) {
    case PCI_IF_EHCI:
      EhciTurnOffUsbEmulation (PciIo);
      break;
    case PCI_IF_UHCI:
      UhciTurnOffUsbEmulation (PciIo);
      break;
    case PCI_IF_XHCI:
      XhciTurnOffUsbEmulation (PciIo);
      break;
    default:
      break;
    }
  }

  gBS->FreePool (HandleArray);
}
