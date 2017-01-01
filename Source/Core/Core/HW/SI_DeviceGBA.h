// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <SFML/Network.hpp>
#include <memory>

#include "Common/CommonTypes.h"
#include "Core/HW/SI_Device.h"

#include <mgba/core/timing.h>
#include <mgba/gba/interface.h>
// GameBoy Advance "Link Cable"

u8 GetNumConnected();
void GBAConnectionWaiter_Shutdown();

class mGBACore;
struct mGBAJoybusShim
{
  GBASIODriver d;
  mGBACore* core;
};

struct mCore;
class mGBACore
{
public:
  mGBACore(int _iDeviceNumber);
  ~mGBACore();

  void Disconnect();

  void ClockSync();
  void ClocksPassed(s32 clocks);

  void Send(const u8* si_buffer);
  int Receive(u8* si_buffer);

  void ProcessCommand(uint32_t cycles_late);

private:
  std::unique_ptr<sf::TcpSocket> client;

  u8 send_data[5];
  u8 recv_data[5];
  int num_received;
  bool need_process;

  u64 last_time_slice;
  u8 device_number;

  s32 clocks_pending;
  std::unique_ptr<color_t[]> video_buffer;

  mCore* core;
  mGBAJoybusShim shim;
  mTimingEvent joy_event;
};

class CSIDevice_GBA : public ISIDevice, private mGBACore
{
public:
  CSIDevice_GBA(SIDevices device, int _iDeviceNumber);
  ~CSIDevice_GBA();

  int RunBuffer(u8* _pBuffer, int _iLength) override;
  int TransferInterval() override;

  bool GetData(u32& _Hi, u32& _Low) override { return false; }
  void SendCommand(u32 _Cmd, u8 _Poll) override {}
private:
  u8 cmd;
  int num_bits_sent;
  int num_bits_received;
  int num_data_received;
  u64 timestamp_sent;
  bool waiting_for_response;
};
