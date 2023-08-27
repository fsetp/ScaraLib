////////////////////////////////////////////////////////////////////////////////
//
#ifndef	_SCARA_LIB_H_
#define	_SCARA_LIB_H_

typedef	unsigned char BYTE;
typedef void* HID_UART_DEVICE;

enum pos_type {	POS_X,
				POS_Y,
				POS_Z,
				POS_YAW,
				POS_GRIP
			};

enum dir_type {	DIR_CW,
				DIR_CCW
			};

////////////////////////////////////////
//
extern "C" __declspec(dllexport) void SetDebugMode(bool bDebug);
extern "C" __declspec(dllexport) int GetEnumDevices();
extern "C" __declspec(dllexport) HID_UART_DEVICE ScaraOpen(int nDeviceNum);
extern "C" __declspec(dllexport) bool ScaraClose(HID_UART_DEVICE hDevice);
extern "C" __declspec(dllexport) bool Initialize(HID_UART_DEVICE hDevice);
extern "C" __declspec(dllexport) bool SetPos(pos_type type, double pos);
extern "C" __declspec(dllexport) bool SetDir(dir_type type);
extern "C" __declspec(dllexport) bool Move(HID_UART_DEVICE hDevice, int ms);

extern "C" __declspec(dllexport) bool MotorTorque(HID_UART_DEVICE hDevice, bool bOn, int nFrom, int nTo);
extern "C" __declspec(dllexport) bool MoveXY(HID_UART_DEVICE hDevice, double x, double y, int ms);
extern "C" __declspec(dllexport) bool MoveZ(HID_UART_DEVICE hDevice, double z, int ms);
extern "C" __declspec(dllexport) bool MoveXYZ(HID_UART_DEVICE hDevice, double x, double y, double z, int ms);
extern "C" __declspec(dllexport) bool MoveYawGrip(HID_UART_DEVICE hDevice, double yaw, double grip, int ms);
extern "C" __declspec(dllexport) bool MoveYaw(HID_UART_DEVICE hDevice, double yaw, int ms);
extern "C" __declspec(dllexport) bool MoveGrip(HID_UART_DEVICE hDevice, double grip, int ms);
extern "C" __declspec(dllexport) bool MoveXYZYaw(HID_UART_DEVICE hDevice, double x, double y, double z, double yaw, int ms);
extern "C" __declspec(dllexport) bool MoveXYZYawGrip(HID_UART_DEVICE hDevice, double x, double y, double z, double yaw, double grip, int ms);
extern "C" __declspec(dllexport) double GetPos(HID_UART_DEVICE hDevice, int axis);
#endif	// _SCARA_LIB_H_
