////////////////////////////////////////////////////////////////////////////////
// Scara Robot Control
//
//		 https://docs.python.org/ja/3/library/ctypes.html
//
//	Relase
//		Rev 1.0:2023.08.19
//		Rev	1.1:2023.08.25
//
//
#ifndef	_SCARA_LIB_H_
#define	_SCARA_LIB_H_

typedef	unsigned char BYTE;
typedef void* HID_UART_DEVICE;

enum pos_type {	POS_X		= 0,
				POS_Y		= 1,
				POS_Z		= 2,
				POS_YAW		= 3,
				POS_GRIP	= 4
			};

enum dir_type {	DIR_CW	= 0,
				DIR_CCW	= 1
			};

////////////////////////////////////////
//
extern "C" __declspec(dllexport) void SetCoordinate(int coordinate);
extern "C" __declspec(dllexport) void SetDebugMode(bool bDebug);
extern "C" __declspec(dllexport) int GetEnumDevices();
extern "C" __declspec(dllexport) HID_UART_DEVICE ScaraOpen(int nDeviceNum);
extern "C" __declspec(dllexport) bool ScaraClose(HID_UART_DEVICE hDevice);
extern "C" __declspec(dllexport) bool Initialize(HID_UART_DEVICE hDevice);
extern "C" __declspec(dllexport) bool SetDir(dir_type type);
extern "C" __declspec(dllexport) bool FixYaw(bool bFix);
extern "C" __declspec(dllexport) bool SetPos(pos_type type, double pos);
extern "C" __declspec(dllexport) double GetPos(pos_type type);
extern "C" __declspec(dllexport) bool Move(HID_UART_DEVICE hDevice, int ms);
extern "C" __declspec(dllexport) bool MotorTorque(HID_UART_DEVICE hDevice, bool bOn, int nFrom, int nTo);

////////////////////////////////////////////////////////////////////////////////
#endif	// _SCARA_LIB_H_
