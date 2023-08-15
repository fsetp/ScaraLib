////////////////////////////////////////////////////////////////////////////////
//
#ifndef	_SCARA_LIB_H_
#define	_SCARA_LIB_H_

typedef	unsigned char BYTE;
typedef void* HID_UART_DEVICE;

////////////////////////////////////////
//
extern "C" __declspec(dllexport) int GetEnumDevices();
extern "C" __declspec(dllexport) HID_UART_DEVICE DeviceOpen(int nDeviceNum);
extern "C" __declspec(dllexport) bool DeviceClose(HID_UART_DEVICE hDevice);
extern "C" __declspec(dllexport) bool MotorTorque(HID_UART_DEVICE hDevice, bool bOn, int nFrom, int nTo);
extern "C" __declspec(dllexport) bool MoveXY(HID_UART_DEVICE hDevice, double x, double y, int ms);
extern "C" __declspec(dllexport) bool MoveXYZ(HID_UART_DEVICE hDevice, double x, double y, double z, int ms);
extern "C" __declspec(dllexport) bool MoveYawOC(HID_UART_DEVICE hDevice, double yaw, double oc, int ms);
extern "C" __declspec(dllexport) bool MoveYaw(HID_UART_DEVICE hDevice, double yaw, int ms);
extern "C" __declspec(dllexport) bool MoveOC(HID_UART_DEVICE hDevice, double oc, int ms);
extern "C" __declspec(dllexport) bool MoveXYZYaw(HID_UART_DEVICE hDevice, double x, double y, double z, double yaw, int ms);
extern "C" __declspec(dllexport) bool MoveXYZYawOC(HID_UART_DEVICE hDevice, double x, double y, double z, double yaw, double oc, int ms);

#endif	// _SCARA_LIB_H_
