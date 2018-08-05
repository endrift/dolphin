// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/HW/HSP/HSP_DeviceGBPlayer.h"

#include "Common/ChunkFile.h"
#include "Common/Logging/Log.h"

#include "Core/HW/DSP.h"
#include "Core/HW/HSP/HSP.h"
#include "Core/HW/ProcessorInterface.h"

#include <cstring>

namespace HSP
{
enum GBPRegister
{
  GBP_TEST = 0x10,
  GBP_VIDEO = 0x11,
  GBP_CONTROL = 0x14,
  GBP_SIO_CONTROL = 0x15,
  GBP_AUDIO = 0x18,
  GBP_SIO = 0x19,
  GBP_KEYPAD = 0x1C,
  GBP_IRQ = 0x1D
};

enum
{
  IRQ_ASSERTED = 0x8000
};

enum
{
  CONTROL_CART_DETECTED = 0x01,
  CONTROL_CART_INSERTED = 0x02,
  CONTROL_3V = 0x04,
  CONTROL_5V = 0x08,
  CONTROL_MASK_IRQ = 0x10,
  CONTROL_SLEEP = 0x20,
  CONTROL_LINK_CABLE = 0x40,
  CONTROL_LINK_ENABLE = 0x80
};

enum class CHSPDevice_GBPlayer::IRQ : int
{
  LINK = 0,
  GAME_PAK = 1,
  SLEEP = 2,
  SERIAL = 3,
  VIDEO = 4,
  AUDIO = 5
};

CHSPDevice_GBPlayer::CHSPDevice_GBPlayer(HSPDevices device)
    : IHSPDevice(device), m_test{}, m_control(0xCC), m_irq(0x0000)
{
  memset(m_scanlines, 0xFF, sizeof(m_scanlines));
}

u64 CHSPDevice_GBPlayer::Read(u32 address)
{
  u64 value = 0;
  switch (address >> 20)
  {
  case GBP_TEST:
    value = *(u64*)&m_test[address & 0x1F];
    break;
  case GBP_CONTROL:
    value = m_control * 0x0101010101010101ULL;
    break;
  case GBP_IRQ:
    value = m_irq * 0x0000000100000001ULL;
    value |= (m_irq & 0xFF00) * 0x0001010000010100ULL;
    break;
  case GBP_VIDEO:
    value = (u64)m_scanlines[(address & 0xFFF) / sizeof(u64)] << 32;
    value |= m_scanlines[(address & 0xFFF) / sizeof(u64) + 1];
    break;
  default:
    NOTICE_LOG(HSP, "Unknown GBP read from 0x%08x", address);
    break;
  }
  return value;
}

void CHSPDevice_GBPlayer::Write(u32 address, u64 value)
{
  switch (address >> 20)
  {
  case GBP_TEST:
    *(u64*)&m_test[address & 0x1F] = value ^ 0xFFFFFFFFFFFFFFFFULL;
    break;
  case GBP_CONTROL:
    if ((address & 0x1F) != 0x18)
      break;
    m_control = value & 0xFF;
    if (m_control & CONTROL_MASK_IRQ)
      ProcessorInterface::SetInterrupt(ProcessorInterface::INT_CAUSE_HSP, false);
    else if (m_irq & IRQ_ASSERTED)
      ProcessorInterface::SetInterrupt(ProcessorInterface::INT_CAUSE_HSP, true);
    NOTICE_LOG(HSP, "GBP control write: 0x%02x", m_control);
    break;
  case GBP_IRQ:
    if ((address & 0x1F) != 0x18)
      break;
    m_irq &= ~value & 0xFFFF;
    if (!m_irq)
      ProcessorInterface::SetInterrupt(ProcessorInterface::INT_CAUSE_HSP, false);
    break;
  default:
    NOTICE_LOG(HSP, "Unknown GBP write to 0x%08x: 0x%016lx", address, value);
    break;
  }
}

void CHSPDevice_GBPlayer::DoState(PointerWrap& p)
{
  p.Do(m_test);
  p.Do(m_control);
  p.Do(m_irq);
  p.Do(m_scanlines);
}

void CHSPDevice_GBPlayer::AssertIRQ(IRQ irq)
{
  m_irq |= 1 << (2 * static_cast<int>(irq));
  if (!(m_control & CONTROL_MASK_IRQ))
    ProcessorInterface::SetInterrupt(ProcessorInterface::INT_CAUSE_HSP, true);
}

void CHSPDevice_GBPlayer::PostScanlineBuffer(const u32* scanlines)
{
  memcpy(m_scanlines, scanlines, 0xF00);
}
}  // namespace HSP
