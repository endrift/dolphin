// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <netinet/in.h>

#include "Common/CommonTypes.h"
#include "Common/Flag.h"
#include "Common/Logging/Log.h"
#include "Common/Thread.h"
#include "Core/CoreTiming.h"
#include "Core/HW/SI_Device.h"
#include "Core/HW/SI_DeviceGBA.h"
#include "Core/HW/SystemTimers.h"

#include "SFML/Network.hpp"

static std::thread connectionThread;
static std::queue<std::unique_ptr<sf::TcpSocket>> waiting_socks;
static std::mutex cs_gba;

#include "mgba/src/core/core.h"
#include "gba/core.h"
#include "gba/gba.h"
#include "gba/io.h"
#include "util/vfs.h"

static u8 num_connected;

namespace
{
Common::Flag server_running;
}

// The signal is self-clocking, so there's a lot of variance
const u64 GC_BITS_PER_SECOND = 200000;
const u64 GBA_BITS_PER_SECOND = 262144;

static uint16_t _joyWriteRegister(struct GBASIODriver* driver, uint32_t address, uint16_t value)
{
  switch (address)
  {
    case REG_JOYCNT:
      return (value & 0x0040) | (driver->p->p->memory.io[REG_JOYCNT >> 1] & ~(value & 0x7) & ~0x0040);
    case REG_JOYSTAT:
      return (value & 0x0030) | (driver->p->p->memory.io[REG_JOYSTAT >> 1] & ~0x30);
    break;
  }
  return value;
}

static void _joyProcessEvents(struct mTiming* timing, void* user, uint32_t cycles_late)
{
  mGBAJoybusShim* shim = static_cast<mGBAJoybusShim*>(user);
  shim->core->ProcessCommand(cycles_late);
}

u8 GetNumConnected()
{
  int num_ports_connected = num_connected;
  if (num_ports_connected == 0)
    num_ports_connected = 1;

  return num_ports_connected;
}

// --- GameBoy Advance "Link Cable" ---

static int GetSendBits(u8 cmd)
{
  u64 bits_transferred = 8 + 1;

  switch (cmd)
  {
  case JOY_CMD_RECV:
    bits_transferred += 32;
    break;
  case JOY_CMD_RESET:
  case JOY_CMD_POLL:
  case JOY_CMD_TRANS:
  default:
    break;
  }
  return bits_transferred;
}

static int GetRecvBits(u8 cmd)
{
  u64 bits_received = 1;

  switch (cmd)
  {
  case JOY_CMD_RESET:
  case JOY_CMD_POLL:
    bits_received += 24;
    break;
  case JOY_CMD_TRANS:
    bits_received += 40;
    break;
  case JOY_CMD_RECV:
    bits_received += 8;
  default:
    break;
  }
  return bits_received;
}

static int GetSendTransferTime(int bits_transferred)
{
  return (int)(bits_transferred * (u64)SystemTimers::GetTicksPerSecond() / GC_BITS_PER_SECOND);
}

static int GetRecvTransferTime(int bits_transferred)
{
  return (int)(bits_transferred * (u64)SystemTimers::GetTicksPerSecond() / GBA_BITS_PER_SECOND);
}

static void GBAConnectionWaiter()
{
  server_running.Set();

  Common::SetCurrentThreadName("GBA Connection Waiter");

  sf::TcpListener server;

  // "dolphin gba"
  if (server.listen(13721) != sf::Socket::Done)
    return;

  server.setBlocking(false);

  auto new_client = std::make_unique<sf::TcpSocket>();
  while (server_running.IsSet())
  {
    if (server.accept(*new_client) == sf::Socket::Done)
    {
      std::lock_guard<std::mutex> lk(cs_gba);
      waiting_socks.push(std::move(new_client));

      new_client = std::make_unique<sf::TcpSocket>();
    }

    Common::SleepCurrentThread(1);
  }
}

void GBAConnectionWaiter_Shutdown()
{
  server_running.Clear();
  if (connectionThread.joinable())
    connectionThread.join();
}

static bool GetAvailableSock(std::unique_ptr<sf::TcpSocket>& sock_to_fill)
{
  bool sock_filled = false;

  std::lock_guard<std::mutex> lk(cs_gba);

  if (!waiting_socks.empty())
  {
    sock_to_fill = std::move(waiting_socks.front());
    waiting_socks.pop();
    sock_filled = true;
  }

  return sock_filled;
}

static CoreTiming::EventType* et_GBAPoll = nullptr;
static mGBACore* active_gbas[4] = { nullptr, nullptr, nullptr, nullptr };

static void GBAPoll(u64 userdata, s64 cyclesLate) {
  CoreTiming::RemoveEvent(et_GBAPoll);
  CoreTiming::ScheduleEvent(SystemTimers::GetTicksPerSecond() / 2048, et_GBAPoll);

  if (active_gbas[0])
    active_gbas[0]->ClockSync();
  if (active_gbas[1])
    active_gbas[1]->ClockSync();
  if (active_gbas[2])
    active_gbas[2]->ClockSync();
  if (active_gbas[3])
    active_gbas[3]->ClockSync();
}

mGBACore::mGBACore(int _iDeviceNumber)
  : shim{ {
    nullptr,
    nullptr, nullptr, nullptr, nullptr,
    _joyWriteRegister
  }, this }
{
  joy_event.context = &shim;
  joy_event.name = "GBA Dolphin JOY Bus";
  joy_event.callback = _joyProcessEvents;
  joy_event.priority = 0x80;

  if (!connectionThread.joinable())
    connectionThread = std::thread(GBAConnectionWaiter);

  if (!et_GBAPoll)
    et_GBAPoll = CoreTiming::RegisterEvent("GBAPoll", GBAPoll);

  num_connected = 0;
  last_time_slice = 0;
  device_number = _iDeviceNumber;

  clocks_pending = 0;

  core = GBACoreCreate();
  core->init(core);

  video_buffer = std::make_unique<color_t[]>(240 * 160);
  core->setVideoBuffer(core, video_buffer.get(), 240);

  GBA* gba = static_cast<GBA*>(core->board);
  GBASIOSetDriver(&gba->sio, &shim.d, SIO_JOYBUS);

  struct VFile* bios = VFileOpen("gba_bios.bin", O_RDONLY);
  if (bios) {
    core->loadBIOS(core, bios, 0);
  }
}

mGBACore::~mGBACore()
{
  active_gbas[device_number] = nullptr;
  Disconnect();
}

void mGBACore::Disconnect()
{
  if (client)
  {
    num_connected--;
    client->disconnect();
    client = nullptr;
  }
  last_time_slice = 0;
}

void mGBACore::ClockSync()
{
  if (!client)
  {
    if (!GetAvailableSock(client))
      return;

    core->reset(core);

    u32 width = htonl(240);
    u32 height = htonl(160);
    u32 depth = htonl(4);
    client->send(&width, sizeof(width));
    client->send(&height, sizeof(height));
    client->send(&depth, sizeof(depth));

    active_gbas[device_number] = this;
    CoreTiming::RemoveEvent(et_GBAPoll);
    CoreTiming::ScheduleEvent(SystemTimers::GetTicksPerSecond() / 2048, et_GBAPoll);
  }
  u64 time_slice = 0;

  if (last_time_slice == 0)
  {
    num_connected++;
    time_slice = SystemTimers::GetTicksPerSecond() / 60;
  }
  else
  {
    time_slice = CoreTiming::GetTicks() - last_time_slice;
  }

  time_slice = (u64)time_slice * 16777216 / SystemTimers::GetTicksPerSecond();
  last_time_slice = CoreTiming::GetTicks();

  clocks_pending += time_slice;

  if (clocks_pending <= 0)
    WARN_LOG(SERIALINTERFACE, "Negative clocks pending: %d", clocks_pending);

  GBA* gba = static_cast<GBA*>(shim.core->core->board);
  mTimingDeschedule(&gba->timing, &joy_event);
  mTimingSchedule(&gba->timing, &joy_event, clocks_pending);
  int current_frame = core->frameCounter(core);
  do {
    core->runLoop(core);
  } while (clocks_pending > 0);
  if (core->frameCounter(core) != current_frame)
  {
    client->setBlocking(true);
    client->send(video_buffer.get(), 240 * 160 * 4);
    u16 input;
    size_t num_client_received;
    client->setBlocking(false);
    client->receive(&input, sizeof(input), num_client_received);
    if (!num_client_received) {
      input = 0;
    }
    core->setKeys(core, ntohs(input));
  }
}

void mGBACore::Send(const u8* si_buffer)
{
  for (int i = 0; i < 5; i++)
    send_data[i] = si_buffer[i ^ 3];

#ifdef _DEBUG
  NOTICE_LOG(SERIALINTERFACE, "%01d cmd %02x [> %02x%02x%02x%02x]", device_number, (u8)send_data[0],
             (u8)send_data[1], (u8)send_data[2], (u8)send_data[3], (u8)send_data[4]);
#endif

  num_received = 0;
  need_process = true;
}

int mGBACore::Receive(u8* si_buffer)
{
  if (!client || shim.d.p->mode != SIO_JOYBUS)
  {
    need_process = false;
    return 0;
  }

  if (num_received > 0)
  {
#ifdef _DEBUG
    WARN_LOG(SERIALINTERFACE, "%01d                              [< %02x%02x%02x%02x%02x] (%d)",
      device_number,
      (u8)recv_data[0], (u8)recv_data[1], (u8)recv_data[2],
      (u8)recv_data[3], (u8)recv_data[4],
      num_received);
#endif

    for (int i = 0; i < 5; i++)
      si_buffer[i ^ 3] = recv_data[i];
  }

  return num_received;
}

void mGBACore::ProcessCommand(uint32_t cycles_late)
{
  GBA* gba = static_cast<GBA*>(core->board);
  clocks_pending = -cycles_late;
  gba->earlyExit = true;
  if (shim.d.p->mode != SIO_JOYBUS)
    need_process = false;
  if (!need_process)
    return;

  switch ((u8) send_data[0])
  {
  case JOY_CMD_RESET:
    gba->memory.io[REG_JOYCNT >> 1] |= 1;
    if (gba->memory.io[REG_JOYCNT >> 1] & 0x40)
      GBARaiseIRQ(gba, IRQ_SIO);
    // Fall through
  case JOY_CMD_POLL:
    recv_data[0] = 0x00;
    recv_data[1] = 0x04;
    recv_data[2] = gba->memory.io[REG_JOYSTAT >> 1];
    num_received = 3;
    break;
  case JOY_CMD_RECV:
    if (gba->memory.io[REG_JOYSTAT >> 1] & JOYSTAT_RECV_BIT) {
      // XXX: Process this data later once that bit has been cleared
      return;
    }
    gba->memory.io[REG_JOYCNT >> 1] |= 2;
    gba->memory.io[REG_JOYSTAT >> 1] |= JOYSTAT_RECV_BIT;
    gba->memory.io[REG_JOY_RECV_LO >> 1] = send_data[1] | (send_data[2] << 8);
    gba->memory.io[REG_JOY_RECV_HI >> 1] = send_data[3] | (send_data[4] << 8);
    recv_data[0] = gba->memory.io[REG_JOYSTAT >> 1];
    num_received = 1;
    if (gba->memory.io[REG_JOYCNT >> 1] & 0x40)
      GBARaiseIRQ(gba, IRQ_SIO);
    break;
  case JOY_CMD_TRANS:
    if (!(gba->memory.io[REG_JOYSTAT >> 1] & JOYSTAT_TRANS_BIT)) {
      // The GBA timed out when replying
      need_process = false;
      return;
    }
    gba->memory.io[REG_JOYCNT >> 1] |= 4;
    recv_data[0] = gba->memory.io[REG_JOY_TRANS_LO >> 1];
    recv_data[1] = gba->memory.io[REG_JOY_TRANS_LO >> 1] >> 8;
    recv_data[2] = gba->memory.io[REG_JOY_TRANS_HI >> 1];
    recv_data[3] = gba->memory.io[REG_JOY_TRANS_HI >> 1] >> 8;
    recv_data[4] = gba->memory.io[REG_JOYSTAT >> 1];
    gba->memory.io[REG_JOYSTAT >> 1] &= ~JOYSTAT_TRANS_BIT;
    num_received = 5;
    if (gba->memory.io[REG_JOYCNT >> 1] & 0x40)
      GBARaiseIRQ(gba, IRQ_SIO);
    break;
  }
  need_process = false;
}

CSIDevice_GBA::CSIDevice_GBA(SIDevices _device, int _iDeviceNumber)
  : ISIDevice(_device, _iDeviceNumber)
  , mGBACore(_iDeviceNumber)
{
  waiting_for_response = false;
}

CSIDevice_GBA::~CSIDevice_GBA()
{
  mGBACore::Disconnect();
}

int CSIDevice_GBA::RunBuffer(u8* _pBuffer, int _iLength)
{
  // For debug logging only
  ISIDevice::RunBuffer(_pBuffer, _iLength);

  if (!waiting_for_response)
  {
    num_data_received = 0;
    cmd = _pBuffer[3];
    num_bits_sent = GetSendBits(cmd);
    num_bits_received = GetRecvBits(cmd);
    Send(_pBuffer);
    CoreTiming::RemoveEvent(et_GBAPoll);
    CoreTiming::ScheduleEvent(GetSendTransferTime(num_bits_sent), et_GBAPoll);
    timestamp_sent = CoreTiming::GetTicks();
    waiting_for_response = true;
  }

  ClockSync();

  // Use an invalid command to get the time it takes to send the command packet
  if (waiting_for_response && num_data_received == 0 && CoreTiming::GetTicks() - timestamp_sent >= (u64)GetSendTransferTime(num_bits_sent) + (u64)GetRecvTransferTime(num_bits_received))
  {
    num_data_received = Receive(_pBuffer);

    NOTICE_LOG(SERIALINTERFACE, "Received %i bytes from GBA %i", num_data_received, GetDeviceNumber());
  }

  if (waiting_for_response && CoreTiming::GetTicks() - timestamp_sent >= (u64)GetSendTransferTime(num_bits_sent) + (u64)GetRecvTransferTime(num_bits_received))
  {
    NOTICE_LOG(SERIALINTERFACE, "JOY transfer finished on GBA %i", GetDeviceNumber());

    waiting_for_response = false;
    if (num_data_received > 0)
      return num_data_received;
  } else
    return 0;

  reinterpret_cast<u32*>(_pBuffer)[0] = SI_ERROR_NO_RESPONSE;
  return 4;
}

int CSIDevice_GBA::TransferInterval()
{
  return GetSendTransferTime(num_bits_sent) + GetRecvTransferTime(num_bits_received);
}
