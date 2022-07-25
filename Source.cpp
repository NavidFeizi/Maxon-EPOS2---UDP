/*
	This code is a UDP server on local host (127.0.0.1)
	integarted with maxon controller on Velocity mode.
	The program can be used for communication between
	QUARC (Real-Time Control Software - Quanser) and
	Maxon EPOS2 controller.

	-> set SDL check setting under C/C++ General to => No (/sdl-)
	-> add library files to Resource Files
	-> add "Definitions.h" to additional include directories
*/

#include <stdio.h>
#include <winsock2.h>
#include <Mstcpip.h>
#include <iostream>
#include "Definitions.h"
#include <string>
#include <windows.h>

#pragma comment(lib, "ws2_32.lib") //Winsock Library
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)

using namespace std;

BOOL bNewBehavior = FALSE;
DWORD dwBytesReturned = 0;
SOCKET s;						// socket

static HANDLE mHandle = NULL;	// Maxon controller handle
int    NodeID = 1;				// Maxon controller node ID
DWORD  ErrCode;					// Maxon controller erro code


#define BUFLEN 512				//Max length of buffer
#define PORT 8888				//The port on which to listen for incoming data

void Char2Float(char inchar[4], float* outfloat);
void Float2Char(float infloat, char* outStr);
float Char2Float2(char inchar[4]);
void exiting();

/* Disable device on termin */
BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
		// Handle the CTRL-C signal.
	case CTRL_C_EVENT:
		printf("Ctrl-C event\n\n");
		exiting();
		return TRUE;
		// CTRL-CLOSE: confirm that the user wants to exit.
	case CTRL_CLOSE_EVENT:
		printf("Ctrl-Close event\n\n");
		exiting();
		return TRUE;
		// Pass other signals to the next handler.
	case CTRL_BREAK_EVENT:
		printf("Ctrl-Break event\n\n");
		exiting();
		return FALSE;
	default:
		return FALSE;
	}
}

void exiting() {
	std::cout << "Exiting";
	/* Disable device */
	VCS_SetDisableState(mHandle, NodeID, &ErrCode);
}

void main()
{
	/* UDP parameters */
	struct sockaddr_in server, si_other;
	int slen, recv_len;
	char buf[BUFLEN];
	WSADATA wsa;
	char Str2send[6];
	slen = sizeof(si_other);
	const char serverAddress[] = "127.0.0.1";
	float inputData;			// input data from UDP
	long outputData;			// input data to UDP

	/* maxon controller parameters */
	BOOL	Fault = FALSE;
	char	ErrorInfo[255];
	char	pDeviceName[] = "EPOS2";
	char	pProtocolStackName[] = "MAXON SERIAL V2";
	char	pInterfaceName[] = "USB";
	char	pPortName[] = "USB0";

	if (SetConsoleCtrlHandler(CtrlHandler, TRUE))
	{
		/* Initialise winsock */
		printf("\nInitialising Winsock => ");
		if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		{
			printf("Failed. Error Code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}
		printf("Initialised => ");

		/* Create a socket */
		if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
		{
			printf("Could not Create Socket : %d", WSAGetLastError());
			return;
		}
		printf("Socket Created => ");
		WSAIoctl(s, SIO_UDP_CONNRESET, &bNewBehavior, sizeof bNewBehavior, NULL, 0, &dwBytesReturned, NULL, NULL);		// disable 10054 error

		/* Prepare the sockaddr_in structure */
		server.sin_family = AF_INET;
		server.sin_addr.s_addr = inet_addr(serverAddress);
		server.sin_port = htons(PORT);

		/* Bind */
		if (bind(s, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
		{
			printf("Bind Failed with Error Code : %d", WSAGetLastError());
			exit(EXIT_FAILURE);
		}
		printf("Bind Done! => Server Address: %s\n", serverAddress);

		/* Open usb port */
		mHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &ErrCode);
		if (ErrCode) {
			VCS_GetErrorInfo(ErrCode, ErrorInfo, 255);
			printf("Error Opening Device! Error: %s \n", ErrorInfo);
		}
		else {
			printf("USB Port Opened => ");
		}

		/* Reseting Error if in error state */
		if (!VCS_GetFaultState(mHandle, NodeID, &Fault, &ErrCode)) {
			printf("Error Checking Fault State \n");
			return;
		}
		if (Fault) {
			if (!VCS_ClearFault(mHandle, NodeID, &ErrCode)) {
				printf("Error Clearing Fault State \n");
				return;
			}
		}

		/* Enabling controller */
		if (!VCS_SetEnableState(mHandle, NodeID, &ErrCode)) {
			printf("Error Enabling Device \n");
			return;
		}
		else {
			printf("Device Enabled!! \n");
		}

		/* Setting operation mode (velocity control)*/
		if (!VCS_SetOperationMode(mHandle, NodeID, OMD_VELOCITY_MODE, &ErrCode)) {
			printf("Error Setting Operation Mode \n");
			return;
		}

		/* keep listening for data */
		while (1)
		{
			try {
				//printf("Waiting for data...");
				fflush(stdout);

				/* clear the buffer by filling null, it might have previously received data */
				memset(buf, '\0', BUFLEN);

				/* try to receive some data, this is a blocking call */
				if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr*)&si_other, &slen)) == SOCKET_ERROR)
				{
					printf("recvfrom() failed with error code : %d", WSAGetLastError());
					exit(EXIT_FAILURE);
				}

				/* print received chars to 32 bit float */
				printf("Packet from %s:%d =>   ", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
				Char2Float(buf, &inputData);
				printf("Traget Velocity: %0.1f   ", inputData);

				//
				VCS_SetVelocityMust(mHandle, NodeID, inputData, &ErrCode);	// command controller
				VCS_GetVelocityIs(mHandle, NodeID, &outputData, &ErrCode);	// get current velocity
				printf("Current Velocity: %d\n", outputData);

				/* convert actual velocity to characters and send it */
				Float2Char(outputData, Str2send);
				//printf("Str received: %s    -> Str sent : %s \n", buf, Str2send);
				if (sendto(s, Str2send, 4, 0, (struct sockaddr*)&si_other, slen) == SOCKET_ERROR)
				{
					printf("sendto() failed with error code : %d", WSAGetLastError());
					exit(EXIT_FAILURE);
				}
			}
			catch (...) {
				exiting();
				closesocket(s);
				WSACleanup();
			}
		}
	}
	else {
		exiting();
		closesocket(s);
		WSACleanup();
	}
	return;
}

void Char2Float(char inchar[4], float* outfloat) {
	/* this function convers first 4 bits of received data(from QUARC) to a 32 bit float */
	char b[] = { inchar[0], inchar[1], inchar[2], inchar[3] };
	memcpy(*&outfloat, &b, 4);
}

void Float2Char(float infloat, char* outStr) {
	/* this function convers a 32 bit float to 4 bits of characters for sending to QUARC */
	memcpy(*&outStr, &infloat, 4);
}

float Char2Float2(char inchar[4]) {
	float outfloat;
	char b[] = { inchar[0], inchar[1], inchar[2], inchar[3] };
	memcpy(&outfloat, &b, sizeof(outfloat));
	return outfloat;
}