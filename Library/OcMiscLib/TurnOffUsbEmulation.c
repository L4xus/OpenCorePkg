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
#define XHC_USBCMD_OFFSET         0x00    ///< USB Command Register Offset
#define XHC_NEXT_CAPABILITY_MASK  0xFF00
#define XHC_CAPABILITY_ID_MASK    0xFF

#define EHC_BAR_INDEX             0x00
#define EHC_HCCPARAMS_OFFSET      0x08

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
  UINT32  ExtendCapOffset;
  UINT32  Value;

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

  if (EFI_ERROR (Status)) {
    return;
  }

  ExtendCapOffset = EFI_ERROR (Status) ? 0 : ((HcCapParams >> 16) << 2); // spec table 7-1

  while (ExtendCapOffset) {
    Status = PciIo->Mem.Read (
      PciIo,
      EfiPciIoWidthUint32,
      XHC_USBCMD_OFFSET,
      ExtendCapOffset,
      1,
      &Value
      );

    if (EFI_ERROR (Status)) {
      break;
    }

    if ((Value & XHC_CAPABILITY_ID_MASK) == 1) {
      //
      // USBLEGSUP register present
      // Check current state
      // Do nothing if there is no legacy emulation on device
      //
      if (!(Value & CONTROLLED_BY_BIOS)) {
        break;
      }

      Value &= ~CONTROLLED_BY_BIOS;
      Value |= CONTROLLED_BY_OS;

      (VOID) PciIo->Mem.Write (
        PciIo,
        EfiPciIoWidthUint32,
        XHC_USBCMD_OFFSET,
        ExtendCapOffset,
        1,
        &Value
        );
    }
    ExtendCapOffset += ((Value & XHC_NEXT_CAPABILITY_MASK) >> 8) << 2; // spec table 7-1
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
  UINT32           ExtendCapOffset;
  UINT32           UsbLegSup;
  UINT32           HcCapParams;

  Status = PciIo->Mem.Read (
    PciIo,
    EfiPciIoWidthUint32,
    EHC_BAR_INDEX,
    EHC_HCCPARAMS_OFFSET,
    1,
    &HcCapParams
    );

  if (EFI_ERROR (Status)) {
    return;
  }

  ExtendCapOffset = (HcCapParams >> 8U) & 0xFFU;

  Status = PciIo->Pci.Read (
    PciIo,
    EfiPciIoWidthUint32,
    ExtendCapOffset,
    1,
    &UsbLegSup
    );

  if (EFI_ERROR (Status) || !(UsbLegSup & CONTROLLED_BY_BIOS)) {
    return;
  }

  UsbLegSup &= ~CONTROLLED_BY_BIOS;
  UsbLegSup |= CONTROLLED_BY_OS;

  (VOID) PciIo->Mem.Write (
    PciIo,
    EfiPciIoWidthUint32,
    XHC_USBCMD_OFFSET,
    ExtendCapOffset,
    1,
    &UsbLegSup
    );
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
