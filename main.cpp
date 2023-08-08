#pragma warning(disable:4996)

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_LED_RED                65
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Data Byte Length
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         11                   // Dynamixel#1 ID: 11
#define DXL2_ID                         12                   // Dynamixel#2 ID: 12
#define DXL3_ID                         13                   // Dynamixel#3 ID: 13
#define DXL4_ID                         14                   // Dynamixel#4 ID: 14
#define DXL5_ID                         15                   // Dynamixel#5 ID: 15
#define BAUDRATE                        1000000
#define DEVICENAME                      "COM3"              // Check which port is being used on your controller
							    // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
#elif defined(_WIN32) || defined(_WIN64)
	return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
#elif defined(_WIN32) || defined(_WIN64)
	return _kbhit();
#endif
}

int main() {
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Initialize GroupBulkWrite instance
	dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

	// Initialize GroupBulkRead instance
	dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

	int index = 0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	bool dxl_addparam_result = false;               // addParam result
	bool dxl_getdata_result = false;                // GetParam result
	int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

	uint8_t dxl_error = 0;                          // Dynamixel error
	uint8_t dxl_led_value[2] = { 0x00, 0xFF };        // Dynamixel LED value for write
	uint8_t param_goal_position[4];
	int32_t dxl1_present_position = 0;               // Present position
	int32_t dxl2_present_position = 0;               // Present position
	int32_t dxl3_present_position = 0;               // Present position
	int32_t dxl4_present_position = 0;               // Present position
	int32_t dxl5_present_position = 0;               // Present position

	// Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Enable Dynamixel#1 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
	}

	// Enable Dynamixel#2 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
	}

	// Enable Dynamixel#3 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel#%d has been successfully connected \n", DXL3_ID);
	}

	// Enable Dynamixel#4 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel#%d has been successfully connected \n", DXL4_ID);
	}

	// Enable Dynamixel#5 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel#%d has been successfully connected \n", DXL5_ID);
	}

	while (1)
	{
		printf("Press any key to continue! (or press ESC to quit!)\n");
		char temp = getch();
		if (temp == ESC_ASCII_VALUE)
			break;

		for (int i = 1; i <= 5; i++) {
			// Read present position
			if (i == 1) {
				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl1_present_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
					printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}
				else if (dxl_error != 0)
				{
					printf("%s\n", packetHandler->getRxPacketError(dxl_error));
				}

				printf("[ID:%03d] PresPos:%03d\n", DXL1_ID, dxl1_present_position);
			}
			else if (i == 2) {
				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl2_present_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
					printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}
				else if (dxl_error != 0)
				{
					printf("%s\n", packetHandler->getRxPacketError(dxl_error));
				}

				printf("[ID:%03d] PresPos:%03d\n", DXL2_ID, dxl2_present_position);
			}
			else if (i == 3) {
				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl3_present_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
					printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}
				else if (dxl_error != 0)
				{
					printf("%s\n", packetHandler->getRxPacketError(dxl_error));
				}

				printf("[ID:%03d] PresPos:%03d\n", DXL3_ID, dxl3_present_position);
			}
			else if (i == 4) {
				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl4_present_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
					printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}
				else if (dxl_error != 0)
				{
					printf("%s\n", packetHandler->getRxPacketError(dxl_error));
				}

				printf("[ID:%03d] PresPos:%03d\n", DXL4_ID, dxl4_present_position);
			}
			else if (i == 5) {
				dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl5_present_position, &dxl_error);
				if (dxl_comm_result != COMM_SUCCESS)
				{
					printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
				}
				else if (dxl_error != 0)
				{
					printf("%s\n", packetHandler->getRxPacketError(dxl_error));
				}

				printf("[ID:%03d] PresPos:%03d\n", DXL5_ID, dxl5_present_position);
			}
		}

		if (temp == 'u') {
			// Write goal position
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_present_position + 50, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == 'j') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_present_position - 50, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == 'i') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_present_position + 50, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == 'k') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_present_position - 50, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == 'o') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_present_position + 50, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == 'l') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_GOAL_POSITION, dxl3_present_position - 50, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == 'p') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_present_position + 50, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == ';') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_GOAL_POSITION, dxl4_present_position - 50, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == '[') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, dxl5_present_position + 100, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
		else if (temp == '\'') {
			dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_GOAL_POSITION, dxl5_present_position - 100, &dxl_error);
			if (dxl_comm_result != COMM_SUCCESS)
			{
				printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			}
			else if (dxl_error != 0)
			{
				printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			}
		}
	}

	// Disable Dynamixel#1 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	// Disable Dynamixel#2 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	// Disable Dynamixel#3 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	// Disable Dynamixel#4 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	// Disable Dynamixel#5 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	// Close port
	portHandler->closePort();

	return 0;
}
