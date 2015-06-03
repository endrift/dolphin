// Copyright 2014 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <hidapi.h>
#include <mutex>

#include "Common/Flag.h"
#include "Common/Thread.h"
#include "Core/ConfigManager.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/HW/SI.h"
#include "Core/HW/SI_GCAdapter.h"
#include "Core/HW/SystemTimers.h"
#include "InputCommon/GCPadStatus.h"

namespace SI_GCAdapter
{
enum ControllerTypes
{
	CONTROLLER_NONE = 0,
	CONTROLLER_WIRED = 1,
	CONTROLLER_WIRELESS = 2
};

static void AddGCAdapter(hid_device_info* device);

static bool s_detected = false;
static hid_device* s_handle = nullptr;
static u8 s_controller_type[MAX_SI_CHANNELS] = { CONTROLLER_NONE, CONTROLLER_NONE, CONTROLLER_NONE, CONTROLLER_NONE };
static u8 s_controller_rumble[4];

static std::mutex s_mutex;
static u8 s_controller_payload[37];
static u8 s_controller_payload_swap[37];

static int s_controller_payload_size = 0;

static std::thread s_adapter_thread;
static Common::Flag s_adapter_thread_running;

static std::thread s_adapter_detect_thread;
static Common::Flag s_adapter_detect_thread_running;

static std::function<void(void)> s_detect_callback;

static u64 s_last_init = 0;

static void Read()
{
	while (s_adapter_thread_running.IsSet())
	{
		s_controller_payload_size = hid_read_timeout(s_handle, s_controller_payload_swap, sizeof(s_controller_payload_swap), 16);

		{
		std::lock_guard<std::mutex> lk(s_mutex);
		std::swap(s_controller_payload_swap, s_controller_payload);
		}

		Common::YieldCPU();
	}
}

static void ScanThreadFunc()
{
	Common::SetCurrentThreadName("GC Adapter Scanning Thread");
	NOTICE_LOG(SERIALINTERFACE, "GC Adapter scanning thread started");

	while (s_adapter_detect_thread_running.IsSet())
	{
		if (s_handle == nullptr)
		{
			Setup();
			if (s_detected && s_detect_callback != nullptr)
				s_detect_callback();
		}
		Common::SleepCurrentThread(500);
	}
	NOTICE_LOG(SERIALINTERFACE, "GC Adapter scanning thread stopped");
}

void SetAdapterCallback(std::function<void(void)> func)
{
	s_detect_callback = func;
}

void Init()
{
	if (s_handle != nullptr)
		return;

	if (Core::GetState() != Core::CORE_UNINITIALIZED)
	{
		if ((CoreTiming::GetTicks() - s_last_init) < SystemTimers::GetTicksPerSecond())
			return;

		s_last_init = CoreTiming::GetTicks();
	}

	int ret = hid_init();

	if (ret)
	{
		ERROR_LOG(SERIALINTERFACE, "libusb_init failed with error: %d", ret);
		Shutdown();
	}
	else
	{
		StartScanThread();
	}
}

void StartScanThread()
{
	s_adapter_detect_thread_running.Set(true);
	s_adapter_detect_thread = std::thread(ScanThreadFunc);
}

void StopScanThread()
{
	if (s_adapter_detect_thread_running.TestAndClear())
	{
		s_adapter_detect_thread.join();
	}
}

void Setup()
{
	hid_device_info* list = hid_enumerate(0x057e, 0x0337);

	for (int i = 0; i < MAX_SI_CHANNELS; i++)
	{
		s_controller_type[i] = CONTROLLER_NONE;
		s_controller_rumble[i] = 0;
	}

	for (hid_device_info* device = list; device; device = device->next)
	{
		AddGCAdapter(device);
	}

	hid_free_enumeration(list);
}

static void AddGCAdapter(hid_device_info* device)
{
	s_handle = hid_open(0x057e, 0x0337, device->serial_number);
	if (!s_handle)
		return;

	unsigned char payload = 0x13;
	hid_write(s_handle, &payload, sizeof(payload));

	s_adapter_thread_running.Set(true);
	s_adapter_thread = std::thread(Read);

	s_detected = true;
	if (s_detect_callback != nullptr)
		s_detect_callback();
}

void Shutdown()
{
	StopScanThread();
	Reset();

	hid_exit();
}

void Reset()
{
	if (!s_detected)
		return;

	if (s_adapter_thread_running.TestAndClear())
	{
		s_adapter_thread.join();
	}

	for (int i = 0; i < MAX_SI_CHANNELS; i++)
		s_controller_type[i] = CONTROLLER_NONE;

	s_detected = false;

	if (s_handle)
	{
		hid_close(s_handle);
		s_handle = nullptr;
	}
	if (s_detect_callback != nullptr)
		s_detect_callback();
	NOTICE_LOG(SERIALINTERFACE, "GC Adapter detached");
}

void Input(int chan, GCPadStatus* pad)
{
	if (!SConfig::GetInstance().m_GameCubeAdapter)
		return;

	if (s_handle == nullptr || !s_detected)
		return;

	u8 controller_payload_copy[37];

	{
	std::lock_guard<std::mutex> lk(s_mutex);
	std::copy(std::begin(s_controller_payload), std::end(s_controller_payload), std::begin(controller_payload_copy));
	}

	if (s_controller_payload_size != sizeof(controller_payload_copy))
	{
		INFO_LOG(SERIALINTERFACE, "error reading payload (size: %d, type: %02x)", s_controller_payload_size, controller_payload_copy[0]);
		Reset();
	}
	else
	{
		u8 type = controller_payload_copy[1 + (9 * chan)] >> 4;
		if (type != CONTROLLER_NONE && s_controller_type[chan] == CONTROLLER_NONE)
			NOTICE_LOG(SERIALINTERFACE, "New device connected to Port %d of Type: %02x", chan + 1, controller_payload_copy[1 + (9 * chan)]);

		s_controller_type[chan] = type;

		if (s_controller_type[chan] != CONTROLLER_NONE)
		{
			memset(pad, 0, sizeof(*pad));
			u8 b1 = controller_payload_copy[1 + (9 * chan) + 1];
			u8 b2 = controller_payload_copy[1 + (9 * chan) + 2];

			if (b1 & (1 << 0)) pad->button |= PAD_BUTTON_A;
			if (b1 & (1 << 1)) pad->button |= PAD_BUTTON_B;
			if (b1 & (1 << 2)) pad->button |= PAD_BUTTON_X;
			if (b1 & (1 << 3)) pad->button |= PAD_BUTTON_Y;

			if (b1 & (1 << 4)) pad->button |= PAD_BUTTON_LEFT;
			if (b1 & (1 << 5)) pad->button |= PAD_BUTTON_RIGHT;
			if (b1 & (1 << 6)) pad->button |= PAD_BUTTON_DOWN;
			if (b1 & (1 << 7)) pad->button |= PAD_BUTTON_UP;

			if (b2 & (1 << 0)) pad->button |= PAD_BUTTON_START;
			if (b2 & (1 << 1)) pad->button |= PAD_TRIGGER_Z;
			if (b2 & (1 << 2)) pad->button |= PAD_TRIGGER_R;
			if (b2 & (1 << 3)) pad->button |= PAD_TRIGGER_L;

			pad->stickX = controller_payload_copy[1 + (9 * chan) + 3];
			pad->stickY = controller_payload_copy[1 + (9 * chan) + 4];
			pad->substickX = controller_payload_copy[1 + (9 * chan) + 5];
			pad->substickY = controller_payload_copy[1 + (9 * chan) + 6];
			pad->triggerLeft = controller_payload_copy[1 + (9 * chan) + 7];
			pad->triggerRight = controller_payload_copy[1 + (9 * chan) + 8];
		}
	}
}

void Output(int chan, u8 rumble_command)
{
	if (s_handle == nullptr || !SConfig::GetInstance().m_GameCubeAdapter || !SConfig::GetInstance().m_AdapterRumble)
		return;

	// Skip over rumble commands if it has not changed or the controller is wireless
	if (rumble_command != s_controller_rumble[chan] && s_controller_type[chan] != CONTROLLER_WIRELESS)
	{
		s_controller_rumble[chan] = rumble_command;

		unsigned char rumble[5] = { 0x11, s_controller_rumble[0], s_controller_rumble[1], s_controller_rumble[2], s_controller_rumble[3] };
		int size = hid_write(s_handle, rumble, sizeof(rumble));
		// Netplay sends invalid data which results in size = 0x00.  Ignore it.
		if (size != 0x05 && size != 0x00)
		{
			INFO_LOG(SERIALINTERFACE, "error writing rumble (size: %d)", size);
			Reset();
		}
	}
}

bool IsDetected()
{
	return s_detected;
}

bool IsDriverDetected()
{
	return true;
}

} // end of namespace SI_GCAdapter
