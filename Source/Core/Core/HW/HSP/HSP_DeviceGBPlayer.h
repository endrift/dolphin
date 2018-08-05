// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <memory>
#include "Common/CommonTypes.h"
#include "Core/HW/HSP/HSP_Device.h"

namespace HSP
{
class CHSPDevice_GBPlayer;

class IGBPlayer
{
public:
  IGBPlayer(CHSPDevice_GBPlayer*);
  virtual ~IGBPlayer() = default;

  virtual void Init() = 0;
  virtual void Shutdown();

  virtual bool IsLoaded() const = 0;
  virtual bool IsGBA() const = 0;
};

class CHSPDevice_GBPlayer : public IHSPDevice
{
public:
  enum class IRQ : int;

  CHSPDevice_GBPlayer(HSPDevices device);

  virtual void Write(u32 address, u64 value);
  virtual u64 Read(u32 address);

  virtual void DoState(PointerWrap& p);

  void AssertIRQ(IRQ);
  void PostScanlineBuffer(const u32* scanlines);

private:
  u8 m_test[0x20];
  u8 m_control;
  u16 m_irq;
  u32 m_scanlines[0x400];

  std::unique_ptr<IGBPlayer> m_gbp;
};
}  // namespace HSP
