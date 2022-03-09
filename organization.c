//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Interrupt Demo Application
// Application Overview - The objective of this application is to showcase 
//                        interrupt preemption and tail-chaining capabilities. 
//                        Nested interrupts are synthesized when the interrupts 
//                        have the same priority, increasing priorities and 
//                        decreasing priorities. With increasing priorities, 
//                        preemption will occur; in the other two cases tail-
//                        chaining will occur.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup InterruptsReferenceApp
//! @{
//
//*****************************************************************************

// Standard includes
#include <stdlib.h>
#include <string.h>

// Driverlib includes
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_timer.h"
#include "uart.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "timer.h"
#include "interrupt.h"
#include "gpio.h"
// Common interface includes
#include "uart_if.h"

//SPI
#include "spi.h"
#include "Adafruit_SSD1351.h"
#include "Adafruit_GFX.h"

#include "pin_mux_config.h"
#include "timer_if.h"
#include "gpio_if.h"

//Webservice Communication
#include "common.h"
#include "simplelink.h"

//I2C
#include "i2c_if.h"

//#define CONSOLE1               UARTA1_BASE
//#define CONSOLE_PERIPH1        PRCM_UARTA1

#define UartGetChar()        MAP_UARTCharGet(CONSOLE)
#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);

volatile unsigned long SW2_intcount;
volatile unsigned long SW3_intcount;
volatile unsigned char SW2_intflag;
volatile unsigned char SW3_intflag;

unsigned long UART_STATUS;
unsigned long UARTDataFlag;
char UARTDataBuffer[1024];
char UARTData;
unsigned long UARTDataCount = 0;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End (Interrupts)


//************************Accelerometer****************************************



//**********************End Accelerometer****************************************

//************************For Webservice Communication**************************

#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1

#define APPLICATION_NAME        "SSL"
//#define APPLICATION_VERSION     "1.1.1.EEC.Spring2018"
#define SERVER_NAME             "a1mp98z0a9h8t5-ats.iot.us-west-2.amazonaws.com"
#define GOOGLE_DST_PORT         8443

#define SL_SSL_CA_CERT "/cert/rootCA.der"
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"


//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                7     // Current Date
#define MONTH               3     // Month 1-12
#define YEAR                2022  // Current year
#define HOUR                9     // Time - hours
#define MINUTE              45    // Time - minutes
#define SECOND              30    // Time - seconds


// What to transmit via SPI
#define POSTHEADER "POST /things/CC3200_Thing/shadow HTTP/1.1\n\r"
#define HOSTHEADER "Host: a1mp98z0a9h8t5-ats.iot.us-west-2.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"

#define DATA1 "{\"state\": {\r\n\"desired\" : {\r\n\"var\" : "

// Strings to transmit depending on how many times switch has been pressed
#define DATALOSS    "\"You Lost :(\""
#define DATAFIRST   "\"You are in level 1, Goodluck!\""
#define DATASECOND  "\"Level 2\""
#define DATATHIRD   "\"Level 3\""
#define DATAFOURTH  "\"Level 4\""
#define DATAFIFTH   "\"You Won!\""

#define DATA3 "\r\n}}}\r\n\r\n"

// Application specific status/error codes
typedef enum{

    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

// Global variables to determine which message to print
int length;
int DATAmessage = 0;

typedef struct
{
   // time
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   // date
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;
unsigned long  g_ulPingPacketsRecv = 0;
unsigned long  g_ulGatewayIP = 0;
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1];
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX];
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End: df
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post(int);

/* Variables for OLED Graphics */
/* Variables to store the position of the square on the screen */
int x = 64;
int y = 64;
int dy;
int dx;
int size = 1;

/* Initialize barrier as moving up or down*/
int barrierup = 0;
int barrierup2 = 1;
int barrierup3 = 0;
int barrierup4 = 1;

int level = 1;
int roundsSurvived = 0;
char fontColor[10];

char fontText[30] = "Level: ";
char fontPrint[30];

char character[2];
int powerUp = 0;

/* Initialize coordinates of each barrier */
int barrierY = 80;
int barrierX = 125;

int barrier2Y = 40;
int barrier2X = 12;

int barrier3Y = 50;
int barrier3X = 20;

int barrier4Y = 20;
int barrier4X = 80;

int height1 = 10;
int height2 = 10;
int height3 = 10;
int height4 = 10;

/* For I2C acceleration */
int i;
int j;

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in] pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in] pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End breadcrumb: s18_df
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);



    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}

static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(5000000);
        _SlNonOsMainLoopTask();
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(5000000);
    }

    return SUCCESS;

}




long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}


//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP;

    unsigned int uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256;

    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }




// START: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION
    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
// END: COMMENT THIS OUT IF DISABLING SERVER VERIFICATION



    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    // connect to the peer device - Google server
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal >= 0) {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal == SL_ESECSNOVERIFY) {
        UART_PRINT("Device has connected to the website (UNVERIFIED):");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }
    else if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



int connectToAccessPoint() {
    long lRetVal = -1;
    GPIO_IF_LedConfigure(LED1|LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);


    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();


    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    UART_PRINT("Opening sl_start\n\r");
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");


    // Connecting to WLAN AP
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

//*****************************************************************************
//
//! Main
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

// Obtain the length of the string that will be sent via email
void getStringLength(int number) {

    switch(number) {
    case -1:
        length = strlen(DATALOSS);   //"you lost"
        break;
    case 1:
        length = strlen(DATAFIRST);  //"First"
        break;
    case 2:
        length = strlen(DATASECOND); //"Second"
        break;
    case 3:
        length = strlen(DATATHIRD);  //"Third"
        break;
    case 4:
        length = strlen(DATAFOURTH); //"Fourth"
        break;
    case 5:
        length = strlen(DATAFIFTH);  //"You won"
        break;
    default:
        break;

    }
}

//************************For Webservice Communication**************************

//***************************************I2C************************************


#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
//#define APPLICATION_VERSION     "1.4.0"
#define APP_NAME                "I2C Demo"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************

//*****************************************************************************
//
//! Display a prompt for the user to enter command
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
DisplayPrompt()
{
    UART_PRINT("\n\rcmd#");
}

/*void delay(unsigned long ulCount){
    int i;

  do{
    ulCount--;
        for (i=0; i< 65535; i++) ;
    }while(ulCount);
}*/

//*****************************************************************************
//
//! Display the usage of the I2C commands supported
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
DisplayUsage()
{
    UART_PRINT("Command Usage \n\r");
    UART_PRINT("------------- \n\r");
    UART_PRINT("write <dev_addr> <wrlen> <<byte0> [<byte1> ... ]> <stop>\n\r");
    UART_PRINT("\t - Write data to the specified i2c device\n\r");
    UART_PRINT("read  <dev_addr> <rdlen> \n\r\t - Read data frpm the specified "
                "i2c device\n\r");
    UART_PRINT("writereg <dev_addr> <reg_offset> <wrlen> <<byte0> [<byte1> ... "
                "]> \n\r");
    UART_PRINT("\t - Write data to the specified register of the i2c device\n\r");
    UART_PRINT("readreg <dev_addr> <reg_offset> <rdlen> \n\r");
    UART_PRINT("\t - Read data from the specified register of the i2c device\n\r");
    UART_PRINT("\n\r");
    UART_PRINT("Parameters \n\r");
    UART_PRINT("---------- \n\r");
    UART_PRINT("dev_addr - slave address of the i2c device, a hex value "
                "preceeded by '0x'\n\r");
    UART_PRINT("reg_offset - register address in the i2c device, a hex value "
                "preceeded by '0x'\n\r");
    UART_PRINT("wrlen - number of bytes to be written, a decimal value \n\r");
    UART_PRINT("rdlen - number of bytes to be read, a decimal value \n\r");
    UART_PRINT("bytex - value of the data to be written, a hex value preceeded "
                "by '0x'\n\r");
    UART_PRINT("stop - number of stop bits, 0 or 1\n\r");
    UART_PRINT("--------------------------------------------------------------"
                "--------------- \n\r\n\r");

}

//*****************************************************************************
//
//! Display the buffer contents over I2C
//!
//! \param  pucDataBuf is the pointer to the data store to be displayed
//! \param  ucLen is the length of the data to be displayed
//!
//! \return none
//!
//*****************************************************************************
void
DisplayBuffer(unsigned char *pucDataBuf, unsigned char ucLen)
{
    unsigned char ucBufIndx = 0;
    UART_PRINT("Read contents");
    UART_PRINT("\n\r");
    while(ucBufIndx < ucLen)
    {
        UART_PRINT(" 0x%x, ", pucDataBuf[ucBufIndx]);
        ucBufIndx++;
        if((ucBufIndx % 8) == 0)
        {
            UART_PRINT("\n\r");
        }
    }
    UART_PRINT("\n\r");
}

/*void
DisplayBuffer2(int pucDataBuf, unsigned char ucLen)
{
    unsigned char ucBufIndx = 0;
    UART_PRINT("Read contents");
    UART_PRINT("\n\r");
//    while(ucBufIndx < ucLen)
//   {
        UART_PRINT(" 0x%x, ", pucDataBuf);
//        ucBufIndx++;
//        if((ucBufIndx % 8) == 0)
//        {
//            UART_PRINT("\n\r");
//        }
//    }
    UART_PRINT("\n\r");
}*/

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t      CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

//****************************************************************************
//
//! Parses the read command parameters and invokes the I2C APIs
//!
//! \param pcInpString pointer to the user command parameters
//!
//! This function
//!    1. Parses the read command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessReadCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iRetVal;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    //
    // Get the length of data to be read
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    //
    // Read the specified length of data
    //
    iRetVal = I2C_IF_Read(ucDevAddr, aucDataBuf, ucLen);

    if(iRetVal == SUCCESS)
    {
        UART_PRINT("I2C Read complete\n\r");

        //
        // Display the buffer over UART on successful write
        //
        DisplayBuffer(aucDataBuf, ucLen);
    }
    else
    {
        UART_PRINT("I2C Read failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the readreg command parameters and invokes the I2C APIs
//! i2c readreg 0x<dev_addr> 0x<reg_offset> <rdlen>
//!
//! \param pcInpString pointer to the readreg command parameters
//!
//! This function
//!    1. Parses the readreg command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessReadRegCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucRegOffset, ucRdLen;
    unsigned char aucRdDataBuf[256];
    char *pcErrPtr;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    //
    // Get the register offset address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRegOffset = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    //
    // Get the length of data to be read
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRdLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //                    bma ad      reg offset

    while (1) {
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&ucRegOffset,1,0));

    //
    // Read the specified length of data
    //
    RET_IF_ERR(I2C_IF_Read(ucDevAddr, &aucRdDataBuf[0], ucRdLen));

    UART_PRINT("I2C Read From address complete\n\r");

    //
    // Display the buffer over UART on successful readreg
    //
    DisplayBuffer(aucRdDataBuf, ucRdLen);
    delay(50);
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the writereg command parameters and invokes the I2C APIs
//! i2c writereg 0x<dev_addr> 0x<reg_offset> <wrlen> <0x<byte0> [0x<byte1> ...]>
//!
//! \param pcInpString pointer to the readreg command parameters
//!
//! This function
//!    1. Parses the writereg command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessWriteRegCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucRegOffset, ucWrLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iLoopCnt = 0;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    //
    // Get the register offset to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucRegOffset = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    aucDataBuf[iLoopCnt] = ucRegOffset;
    iLoopCnt++;

    //
    // Get the length of data to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucWrLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucWrLen > sizeof(aucDataBuf));

    //
    // Get the bytes to be written
    //
    for(; iLoopCnt < ucWrLen + 1; iLoopCnt++)
    {
        //
        // Store the data to be written
        //
        pcInpString = strtok(NULL, " ");
        RETERR_IF_TRUE(pcInpString == NULL);
        aucDataBuf[iLoopCnt] =
                (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    }
    //
    // Write the data values.
    //
    RET_IF_ERR(I2C_IF_Write(ucDevAddr,&aucDataBuf[0],ucWrLen+1,1));

    UART_PRINT("I2C Write To address complete\n\r");

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the write command parameters and invokes the I2C APIs
//!
//! \param pcInpString pointer to the write command parameters
//!
//! This function
//!    1. Parses the write command parameters.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ProcessWriteCommand(char *pcInpString)
{
    unsigned char ucDevAddr, ucStopBit, ucLen;
    unsigned char aucDataBuf[256];
    char *pcErrPtr;
    int iRetVal, iLoopCnt;

    //
    // Get the device address
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    //
    // Get the length of data to be written
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);
    //RETERR_IF_TRUE(ucLen > sizeof(aucDataBuf));

    for(iLoopCnt = 0; iLoopCnt < ucLen; iLoopCnt++)
    {
        //
        // Store the data to be written
        //
        pcInpString = strtok(NULL, " ");
        RETERR_IF_TRUE(pcInpString == NULL);
        aucDataBuf[iLoopCnt] =
                (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
    }

    //
    // Get the stop bit
    //
    pcInpString = strtok(NULL, " ");
    RETERR_IF_TRUE(pcInpString == NULL);
    ucStopBit = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);

    //
    // Write the data to the specified address
    //
    iRetVal = I2C_IF_Write(ucDevAddr, aucDataBuf, ucLen, ucStopBit);
    if(iRetVal == SUCCESS)
    {
        UART_PRINT("I2C Write complete\n\r");
    }
    else
    {
        UART_PRINT("I2C Write failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Parses the user input command and invokes the I2C APIs
//!
//! \param pcCmdBuffer pointer to the user command
//!
//! This function
//!    1. Parses the user command.
//!    2. Invokes the corresponding I2C APIs
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ParseNProcessCmd(char *pcCmdBuffer)
{
    char *pcInpString;
    int iRetVal = FAILURE;

    pcInpString = strtok(pcCmdBuffer, " \n\r");
    if(pcInpString != NULL)

    {

        if(!strcmp(pcInpString, "read"))
        {
            //
            // Invoke the read command handler
            //
            iRetVal = ProcessReadCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "readreg"))
        {
            //
            // Invoke the readreg command handler
            //
            iRetVal = ProcessReadRegCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "writereg"))
        {
            //
            // Invoke the writereg command handler
            //
            iRetVal = ProcessWriteRegCommand(pcInpString);
        }
        else if(!strcmp(pcInpString, "write"))
        {
            //
            // Invoke the write command handler
            //
            iRetVal = ProcessWriteCommand(pcInpString);
        }
        else
        {
            UART_PRINT("Unsupported command\n\r");
            return FAILURE;
        }
    }

    return iRetVal;
}

//***************************************I2C************************************


//                              FOR TIMER
//*****************************************************************************
#define APPLICATION_VERSION        "1.4.0"
#define FOREVER                    1

//*****************************************************************************
//                      Global Variables for Vector Table
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;

/* Timer */
unsigned long clock;

/* Decoding */
int bits[10000];
int index;
int bit = 1;

int matchingChars0 = 0;
int matchingChars1 = 0;
int matchingChars2 = 0;
int matchingChars3 = 0;
int matchingChars4 = 0;
int matchingChars5 = 0;
int matchingChars6 = 0;
int matchingChars7 = 0;
int matchingChars8 = 0;
int matchingChars9 = 0;
int matchingCharsMUTE = 0;
int matchingCharsLAST = 0;
int flag = 0;
int pauseTime;
int repeatedCharTime = 0;
char stringToPrint[50];
int stringFinalIndex = 0;
int B2Index = 0;
int B3Index = 0;
int B4Index = 0;
int B5Index = 0;
int B6Index = 0;
int B7Index = 0;
int B8Index = 0;
int B9Index = 0;
char B2Str[5] = {'a', 'b', 'c'};
char B3Str[5] = {'d', 'e', 'f'};
char B4Str[5] = {'g', 'h', 'i'};
char B5Str[5] = {'j', 'k', 'l'};
char B6Str[5] = {'m', 'n', 'o'};
char B7Str[5] = {'p', 'q', 'r', 's'};
char B8Str[5] = {'t', 'u', 'v'};
char B9Str[5] = {'w', 'x', 'y', 'z'};
char alphabet[30][2] = {"a", "d", "g", "j", "m", "p", "t", "w"};
int previousButton = 0;
int colorIndex;
int colorUpdated = 0;
int entered = 0;
char cCharacter;
char colorArray[100];
int colorArrayIndex = -1;
int stringUpdated = 0;
char charTyped = 'z';


int button0[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 1, 1, 1, 1, 2, 1, 1, 1, 2, 2, 2, 2, 1, 2, 2, 2};
int button1[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 2, 1, 1, 1, 1, 2, 2, 2, 1, 2, 2, 2};
int button2[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 1, 2, 1, 1, 2, 1, 1, 1, 2, 1, 2, 2, 1, 2, 2, 2};
int button3[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2};
int button4[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 1, 1, 2, 1, 2, 1, 1, 1, 2, 2, 1, 2, 1, 2, 2, 2};
int button5[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1, 2, 1, 2, 1, 1, 1, 1, 2, 1, 2, 1, 2, 2, 2};
int button6[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 1, 2, 2, 1, 2, 1, 1, 1, 2, 1, 1, 2, 1, 2, 2, 2};
int button7[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 1, 1, 1, 1, 2, 1, 2, 2, 2};
int button8[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 1, 1, 1, 2, 2, 1, 1, 1, 2, 2, 2, 1, 1, 2, 2, 2};
int button9[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1, 1, 2, 2, 1, 1, 1, 1, 2, 2, 1, 1, 2, 2, 2};
int buttonMUTE[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2};
int buttonLAST[50] = {1, 1, 2, 1, 1, 1, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 1, 2, 1, 2, 2, 1, 1, 1, 2, 1, 2, 1, 1, 2, 2, 2};

/* Variables used by I2C */
int iRetVal;
char acCmdStore[512];

/* To store strings */
char* pcInpString;

/* Commands to access registers */
char test[20] = "0x18 0x3 1";
char test2[20] = "0x5";

/* Other strings necessary for eventually translating the register commands to integer values */
unsigned char ucDevAddr, ucRegOffsetx, ucRegOffsety, ucRdLen;
unsigned char aucRdDataBufx[256];
unsigned char aucRdDataBufy[256];
char *pcErrPtr;
char xhex[20];
char yhex[20];

//*****************************************************************************
//
//! The interrupt handler for the first timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************

/* Timer */
void
TimerBaseIntHandler(void)
{
    Timer_IF_InterruptClear(g_ulBase);
    clock++;
}

//*****************************************************************************
//
//! The interrupt handler for the second timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************

//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);


//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

/*
 * Detects each Rising Edge of TV remote signal
 * Used to decode TV remote signals
 *
 */
static void GPIOA1IntHandler(void) {
    unsigned long ulStatus;
    int num_cycles;

    ulStatus = MAP_GPIOIntStatus (GPIOA0_BASE, true);
    MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);

    num_cycles = clock / 10;
    bits[index] = num_cycles;

    if (num_cycles > 200 && num_cycles < 1000) {
        repeatedCharTime = 1;
    }
    else if (num_cycles >= 1000) {
        repeatedCharTime = 0;
    }

    if (stringFinalIndex == 1) {
        strcpy(stringToPrint, "");
        stringFinalIndex = 0;
        stringUpdated = 1;
    }
    /* Button 1 for changing font color */
    if(num_cycles == button1[matchingChars1]) {
        matchingChars1++;
    }
    else {
        matchingChars1 = 0;
    }
    if(matchingChars1 >= 32){

        if (colorIndex == 4) colorIndex = 0;
        else colorIndex++;

        colorArrayIndex++;
        colorArray[colorArrayIndex] = (char)colorIndex;

        colorUpdated = 1;
        previousButton = 1;
        matchingChars1 = 0;
    }

    /* Button 0 for Spaces */
    if(num_cycles == button0[matchingChars0]) {
        matchingChars0++;
    }
    else {
        matchingChars0 = 0;
    }
    if(matchingChars0 >= 32){
        stringUpdated = 1;
        stringToPrint[stringFinalIndex] = ' ';
        stringFinalIndex++;

        previousButton = 0;
        matchingChars0 = 0;
    }

    /* button 2 */
    if(num_cycles == button2[matchingChars2]) {
        matchingChars2++;
    }
    else {
        matchingChars2 = 0;
    }
    /*
     * If we get 32 matching characters in a row, we concluded that
     * the button corresponding to the bit sequence must have been
     * pressed.
     */
    if(matchingChars2 >= 32){
        stringUpdated = 1;
        if (previousButton == 2 && repeatedCharTime == 1) {
            if (B2Index == 2) B2Index = 0;
            else B2Index++;
            stringFinalIndex--;
            stringToPrint[stringFinalIndex] = B2Str[B2Index];
            //charTyped = B2Str[B2Index];  //CHAR TYPED
            //UART_PRINT("charTyped: %c, ", B2Str[B2Index]);
        }
        else {
            B2Index = 0;
            stringToPrint[stringFinalIndex] = B2Str[B2Index];
        }
        stringFinalIndex++;
        previousButton = 2;
        matchingChars2 = 0;
    }

    /* button 3 */
    if(num_cycles == button3[matchingChars3]) {
        matchingChars3++;
    }
    else {
        matchingChars3 = 0;
    }
    if(matchingChars3 >= 32){
        stringUpdated = 1;
        if (previousButton == 3 && repeatedCharTime == 1) {
            if (B3Index == 2) B3Index = 0;
            else B3Index++;
            stringFinalIndex--;
            stringToPrint[stringFinalIndex] = B3Str[B3Index];
        }
        else {
            B3Index = 0;
            stringToPrint[stringFinalIndex] = B3Str[B3Index];
        }
        stringFinalIndex++;
        previousButton = 3;
        matchingChars3= 0;
    }

    /* button 4 */
    if(num_cycles == button4[matchingChars4]) {
        matchingChars4++;
    }
    else {
        matchingChars4 = 0;
    }
    if(matchingChars4 >= 32){
        stringUpdated = 1;
        if (previousButton == 4 && repeatedCharTime == 1) {
            if (B4Index == 2) B4Index = 0;
            else B4Index++;
            stringFinalIndex--;
            stringToPrint[stringFinalIndex] = B4Str[B4Index];
        }
        else {
            B4Index = 0;
            stringToPrint[stringFinalIndex] = B4Str[B4Index];
        }
        stringFinalIndex++;
        previousButton = 4;
        matchingChars4 = 0;
    }
    /* button 5 */
    if(num_cycles == button5[matchingChars5]) {
        matchingChars5++;
    }
    else {
        matchingChars5 = 0;
    }
    if(matchingChars5 >= 32){
        stringUpdated = 1;
        if (previousButton == 5 && repeatedCharTime == 1) {
            if (B5Index == 2) B5Index = 0;
            else B5Index++;
            stringFinalIndex--;
            stringToPrint[stringFinalIndex] = B5Str[B5Index];
        }
        else {
            B5Index = 0;
            stringToPrint[stringFinalIndex] = B5Str[B5Index];
        }
        stringFinalIndex++;
        previousButton = 5;
        matchingChars5 = 0;
    }
    /* button 6 */
    if(num_cycles == button6[matchingChars6]) {
        matchingChars6++;
    }
    else {
        matchingChars6 = 0;
    }
    if(matchingChars6 >= 32){
        stringUpdated = 1;
        if (previousButton == 6 && repeatedCharTime == 1) {
            if (B6Index == 2) B6Index = 0;
            else B6Index++;
            stringFinalIndex--;
            stringToPrint[stringFinalIndex] = B6Str[B6Index];
        }
        else {
            B6Index = 0;
            stringToPrint[stringFinalIndex] = B6Str[B6Index];
        }
        stringFinalIndex++;
        previousButton = 6;
        matchingChars6 = 0;
    }
    /* button 7 */
    if(num_cycles == button7[matchingChars7]) {
        matchingChars7++;
    }
    else {
        matchingChars7 = 0;
    }
    if(matchingChars7 >= 32){
        stringUpdated = 1;
        if (previousButton == 7 && repeatedCharTime == 1) {
            if (B7Index == 3) B7Index = 0;
            else B7Index++;
            stringFinalIndex--;
            stringToPrint[stringFinalIndex] = B7Str[B7Index];
        }
        else {
            B7Index = 0;
            stringToPrint[stringFinalIndex] = B7Str[B7Index];
        }
        stringFinalIndex++;
        previousButton = 7;
        matchingChars7 = 0;
    }
    /* button 8 */
    if(num_cycles == button8[matchingChars8]) {
        matchingChars8++;
    }
    else {
        matchingChars8 = 0;
    }
    if(matchingChars8 >= 32){
        stringUpdated = 1;
        if (previousButton == 8 && repeatedCharTime == 1) {
            if (B8Index == 2) B8Index = 0;
            else B8Index++;
            stringFinalIndex--;
            stringToPrint[stringFinalIndex] = B8Str[B8Index];
        }
        else {
            B8Index = 0;
            stringToPrint[stringFinalIndex] = B8Str[B8Index];
        }
        stringFinalIndex++;
        previousButton = 8;
        matchingChars8 = 0;
    }
    /* button 9 */
    if(num_cycles == button9[matchingChars9]) {
        matchingChars9++;
    }
    else {
        matchingChars9 = 0;
    }
    if(matchingChars9 >= 32){
        stringUpdated = 1;
        if (previousButton == 9 && repeatedCharTime == 1) {
            if (B9Index == 3) B9Index = 0;
            else B9Index++;
            stringFinalIndex--;
            stringToPrint[stringFinalIndex] = B9Str[B9Index];
        }
        else {
            B9Index = 0;
            stringToPrint[stringFinalIndex] = B9Str[B9Index];
        }
        stringFinalIndex++;
        previousButton = 9;
        matchingChars9 = 0;
    }
    /* button MUTE */
    if(num_cycles == buttonMUTE[matchingCharsMUTE]) {
        matchingCharsMUTE++;
    }
    else {
        matchingCharsMUTE = 0;
    }
    if(matchingCharsMUTE >= 32){
        previousButton = 10;
        entered = 1;
        matchingCharsMUTE = 0;
    }
    /* button LAST */
    if(num_cycles == buttonLAST[matchingCharsLAST]) {
        matchingCharsLAST++;
    }
    else {
        matchingCharsLAST = 0;
    }
    if(matchingCharsLAST >= 32){
        stringUpdated = 1;
        if (stringFinalIndex > 0) stringFinalIndex--;
        stringToPrint[stringFinalIndex] = ' ';
        previousButton = 11;
        matchingCharsLAST = 0;
    }
    index++;

    clock = 0;
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

    /* In case of TI-RTOS vector table is initialize by OS itself */
    #ifndef USE_TIRTOS

    #if defined(ewarm)
        MAP_IntVTableBaseSet((unsigned long)&__vector_table);
    #endif

    #endif
    
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

int initAWS() {

    long lRetVal = -1;
    UART_PRINT("My terminal works!\n\r");

    // Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();

    // Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }

    // Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }
    return lRetVal;
}

void IRHandler() {
    /* Base address for first timer */
        unsigned long ulStatus;
        g_ulBase = TIMERA0_BASE;


        /* Configuring the timers */
        Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

        /* Setup the interrupts for the timer timeouts */
        Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);

        /* Turn on the timers feeding values in mSec */
        Timer_IF_Start(g_ulBase, TIMER_A, 100);

        MAP_GPIOIntRegister(GPIOA0_BASE, GPIOA1IntHandler);

        /* Interrupt on rising edge */
        MAP_GPIOIntTypeSet(GPIOA0_BASE, 0x80, GPIO_RISING_EDGE);

        /* Clear interrupts on base A3*/
        ulStatus = MAP_GPIOIntStatus (GPIOA0_BASE, false);
        MAP_GPIOIntClear(GPIOA0_BASE, ulStatus);

        MAP_GPIOIntEnable(GPIOA0_BASE, 0x80);

}

void initOLED() {

    /*Enable the SPI module clock */
     MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
     MAP_PRCMPeripheralReset(PRCM_GSPI);

     /* Reset SPI */
     MAP_SPIReset(GSPI_BASE);

     /* Configure SPI interface */
     MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                      SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                      (SPI_SW_CTRL_CS |
                      SPI_4PIN_MODE |
                      SPI_TURBO_OFF |
                      SPI_CS_ACTIVELOW |
                      SPI_WL_8));

     /* Enable SPI for communication */
     MAP_SPIEnable(GSPI_BASE);

     /* Initialize Adafruit */
     Adafruit_Init();
}

void initI2C() {

    /* I2C Init */
    I2C_IF_Open(I2C_MASTER_MODE_FST);


    /* Get the device address */
    pcInpString = strtok(test, " ");
    ucDevAddr = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    /* Get the register offset address */
    pcInpString = strtok(NULL, " ");
    ucRegOffsetx = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);

    /* Get the length of data to be read */
    pcInpString = strtok(NULL, " ");
    ucRdLen = (unsigned char)strtoul(pcInpString, &pcErrPtr, 10);

    /* Second register offset address */
    pcInpString = strtok(test2, " ");
    ucRegOffsety = (unsigned char)strtoul(pcInpString+2, &pcErrPtr, 16);
}

void powerUpChar() {

    powerUp = 200;
    strcpy(stringToPrint, "");
    stringFinalIndex = 0;
    stringUpdated = 1;
    setTextColor(BLACK, BLACK);
    setCursor(110, 8);
    Outstr(character);
    setTextColor(GREEN, BLACK);
    setCursor(110, 8);
    strcpy(character, alphabet[rand() % 7]);
    Outstr(character);
}

int nextLevel(long lRetVal) {
    level++;

    setTextColor(BLACK, BLACK);
    setCursor(1, 8);
    Outstr(fontPrint);
    setTextColor(GREEN, BLACK);

    // Get the length of the string that will be printed
    DATAmessage = level;
    getStringLength(DATAmessage);

    /* Send level or winning message via email */
    http_post(lRetVal);

    if (level == 5) {

        setCursor(1, 8);
        Outstr("You Win!!");
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
        return 0;
    }

    sprintf(fontColor, "%d", level);
    strcpy(fontPrint, fontText);
    strcat(fontPrint, fontColor);

    setCursor(1, 8);
    Outstr(fontPrint);

    roundsSurvived = 0;
    return 1;
}

void getAccelVals() {

        I2C_IF_Write(ucDevAddr,&ucRegOffsetx,1,0);

        // Read the specified length of data
        I2C_IF_Read(ucDevAddr, &aucRdDataBufx[0], ucRdLen);
        //DisplayBuffer(aucRdDataBufx, ucRdLen);

        I2C_IF_Write(ucDevAddr,&ucRegOffsety,1,0);

        I2C_IF_Read(ucDevAddr, &aucRdDataBufy[0], ucRdLen);
        //DisplayBuffer(aucRdDataBufy, ucRdLen);

        //Convert hex values to integers
        sprintf(xhex, "%x", aucRdDataBufx[0]);
        i = (int)strtol((char*)xhex, NULL, 16);

        sprintf(yhex, "%x", aucRdDataBufy[0]);
        j = (int)strtol((char*)yhex, NULL, 16);

}

void updateSquarePosition() {
    //Handle tilting top of screen down
    if (i > 80) {
           dy = (255 - i)/14;
           y = y - dy;

           if(y < 17) {
               y = 17;
           }

      }
     //Handle tilting bottom of screen down
      else {
           dy = i/14;
           y = y + dy;

           //If ball goes off screen
           if(y > 121) {
               y = 121;
           }
    }

    //Handle tilting left side of screen down
    if (j > 80) {
           dx = (255 - j)/14;
           x = x - dx;

           if(x < 0) {
               x = 0;
           }
      }

     //Handle tilting right side of screen down
     else {
           dx = j/14;
           x = x + dx;

           if(x > 121) {
               x = 121;
           }
     }

    //Change the size of the circle depending on the acceleration
    if(dx > dy && dx > 2) size = dx;
    else if (dy > dx && dy > 2) size = dy;
    else size = 2;

    if (powerUp > 0) {
         GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
         drawRect(x, y, size*3, size*3, GREEN);
    }
    else drawRect(x, y, size*3, size*3, WHITE);
}

void updateBarrierPositions() {

    drawFastVLine(barrierX, barrierY, height1, BLACK);
    if (barrierX == 0) {
        barrierX = 125;
        height1 = (rand() % 20) + 15;
        barrierY = 17 + (rand() % (108 - height1));
        roundsSurvived++;
    }
    else barrierX--;

    if (level > 1) {
        if (barrierup == 1) {
            if (barrierY > 17) {
                barrierY--;
            }
            else {
                barrierup = 0;
            }
        }
        else {
            if (barrierY + height1 - 5 < 122) {
                barrierY++;
            }
            else {
                barrierup = 1;
            }

        }
    }

    drawFastVLine(barrierX, barrierY, height1 , WHITE);

    if (x + 6 >= barrierX && x <= barrierX && powerUp < 1) {
        if (y <= barrierY + height1 && y >= barrierY - 5) {
            //delay(500);
            level = -2;
        }
    }

    drawFastVLine(barrier2X, barrier2Y, height2, BLACK);
    if (barrier2X == 0) {
        barrier2X = 125;
        height2 = (rand() % 20) + 15;
        barrier2Y = 50 + (rand() % (50 - height2));
        roundsSurvived++;
    }
    else barrier2X--;

    /* For barrier moving up and down */
    if (level > 2) {
        if (barrierup2 == 1) {
            if (barrier2Y > 17) {
                barrier2Y--;
            }
            else {
                barrierup2 = 0;
            }
        }
        else {
            if (barrier2Y + height2 - 5 < 122) {
                barrier2Y++;
            }
            else {
                barrierup2 = 1;
            }

        }
    }

    drawFastVLine(barrier2X, barrier2Y, height2 , WHITE);

    if(x + 6 >= barrier2X && x <= barrier2X && powerUp < 1) {
        if (y <= barrier2Y + height2 && y >= barrier2Y - 5) {
            //delay(500);
            level = -2;
        }
    }

    drawFastVLine(barrier3X, barrier3Y, height3, BLACK);
    drawFastVLine(barrier3X + 60, barrier3Y, height3, BLACK);
    if (barrier3X == 0) {
        barrier3X = 125;
        height3 = (rand() % 20) + 15;
        barrier3Y = 17 + (rand() % (50 - height3));
        roundsSurvived++;
    }
    else barrier3X--;

    if (level > 2) {
        if (barrierup3 == 1) {
            if (barrier3Y > 17) {
                barrier3Y--;
            }
            else {
                barrierup3 = 0;
            }
        }
        else {
            if (barrier3Y + height3 - 5 < 122) {
                barrier3Y++;
            }
            else {
                barrierup3 = 1;
            }

        }
    }

    drawFastVLine(barrier3X, barrier3Y, height3 , WHITE);
    drawFastVLine(barrier3X + 60, barrier3Y, height3, WHITE);

    if(((x + 6 >= barrier3X && x <= barrier3X) | (x + 6 >= barrier3X + 60 && x <= barrier3X + 60)) && powerUp < 1) {
        if (y <= barrier3Y + height3 && y >= barrier3Y - 5) {
            //delay(500);
            level = -2;
        }
    }

    drawFastVLine(barrier4X, barrier4Y, height4, BLACK);
    if (barrier4X == 0) {
        barrier4X = 125;
        height4 = (rand() % 20) + 15;
        barrier4Y = 17 + (rand() % (108 - height4));
        roundsSurvived++;
    }
    else barrier4X--;

    if (level > 3) {
        if (barrierup4 == 1) {
            if (barrier4Y > 17) {
                barrier4Y--;
            }
            else {
                barrierup4 = 0;
            }
        }
        else {
            if (barrier4Y + height4 - 5 < 122) {
                barrier4Y++;
            }
            else {
                barrierup4 = 1;
            }

        }
    }

    drawFastVLine(barrier4X, barrier4Y, height4 , WHITE);

    if(x + 6 >= barrier4X && x <= barrier4X && powerUp < 1) {
        if (y <= barrier4Y + height4 && y >= barrier4Y - 5) {
            //delay(500);
            level = -2;
        }
    }
}

void gameOver(long lRetVal) {

    setTextColor(BLACK, BLACK);
    setCursor(1, 8);
    Outstr(fontPrint);
    setTextColor(GREEN, BLACK);

    // Get the length of the string that will be printed
    DATAmessage = -1;
    getStringLength(DATAmessage);

    /* Send losing message via email */
    http_post(lRetVal);
    setCursor(1, 8);
    Outstr("You lost :(");
}

void printInitialText(long lRetVal) {

    sprintf(fontColor, "%d", level);
    /* Set the initial font color to green */
    setTextColor(GREEN, BLACK);
    fillScreen(BLACK);
    strcpy(fontPrint, fontText);
    strcat(fontPrint, fontColor);

    /* Print initial power up character */
    setCursor(1, 8);

    strcpy(character, alphabet[rand() % 7]);
    Outstr(fontPrint);
    setCursor(110, 8);
    Outstr(character);

    setCursor(70, 8);
    Outstr("Power:");
    setCursor(110, 8);
    Outstr(character);

    DATAmessage = level;
    getStringLength(DATAmessage);
    http_post(lRetVal);
}

//****************************************************************************
//
//! Main function
//!
//! \param none
//!
//!
//! \return None.
//
//****************************************************************************
int main() {

    BoardInit();
    PinMuxConfig();

    MAP_PRCMPeripheralReset(CONSOLE_PERIPH);
    InitTerm();

    ClearTerm();

    /* Wifi connection and AWS Set up */
    long lRetVal = initAWS();

    /* I2C Setup */
    initI2C();

    /* SPI Set up */
    initOLED();

    /* IR handler set up*/
    IRHandler();

    printInitialText(lRetVal);

    /* Main driving while loop */
    while (1) {

        /* If we typed the specified character */
        if (!strcmp(stringToPrint, character)) powerUpChar();

        /* Decrement Power up variable with each iteration */
        if (powerUp > 0) powerUp--;

        /* Move on to next level if we survive 15 more barriers passing */
        if (roundsSurvived > 15) {
            if (!nextLevel(lRetVal)) break;
        }

        if (stringUpdated == 1) {
            setCursor(1, 20);
            Outstr(stringToPrint);
            stringUpdated = 0;
        }

        /* Freeze the frame if switch 3 pressed */
        while(GPIOPinRead(GPIOA1_BASE, 0x20)) {}

        /* Get current acceleration values from accelerometer */
        getAccelVals();

        /*Only delete previous circle if SW3 not pushed. This allows us to draw */
        if (!GPIOPinRead(GPIOA1_BASE, 0x20)) drawRect(x, y, size*3, size*3, BLACK);

        updateSquarePosition();
        updateBarrierPositions();

        /* If we hit a barrier and game is over */
        if (level == -2) {
            gameOver(lRetVal);
            break;
        }

    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


static int http_post(int iTLSSockID){
    char acSendBuff[512];
    char GetBuff[512];
    char* GetBuffs;
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    GetBuffs = GetBuff;

    strcpy(GetBuffs, HOSTHEADER);
    GetBuffs += strlen(HOSTHEADER);
    strcpy(GetBuffs, CHEADER);
    GetBuffs += strlen(CHEADER);
    strcpy(GetBuffs, "\r\n\r\n");

    int dataLength = strlen(DATA1) + length + strlen(DATA3);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);

    strcpy(pcBufHeaders, CLHEADER1);
    pcBufHeaders += strlen(CLHEADER1);

    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);


    strcpy(pcBufHeaders, DATA1);
    pcBufHeaders += strlen(DATA1);

    //strcpy(pcBufHeaders, DATAFIRST);

    // Insert the message corresponding to the value of DATAmessage into the total string
    switch(DATAmessage) {
    case -1:
        strcpy(pcBufHeaders, DATALOSS);
        break;
    case 1:
        strcpy(pcBufHeaders, DATAFIRST);
        break;
    case 2:
        strcpy(pcBufHeaders, DATASECOND);
        break;
    case 3:
        strcpy(pcBufHeaders, DATATHIRD);
        break;
    case 4:
        strcpy(pcBufHeaders, DATAFOURTH);
        break;
    case 5:
        strcpy(pcBufHeaders, DATAFIFTH);
        break;
    default:
        break;

    }
    pcBufHeaders += length;

    strcpy(pcBufHeaders, DATA3);
    pcBufHeaders += strlen(DATA3);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);



    // Send the packet to the server
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);

    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

