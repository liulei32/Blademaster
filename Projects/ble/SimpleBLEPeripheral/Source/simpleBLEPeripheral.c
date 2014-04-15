/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_archerled.h"
#include "hal_archerwatch.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_i2c.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "BMSensor.h"
#include "cma3000d.h"
#include "archeraccelerometer.h"
#include "archerled.h"
#include "archerbatt.h"
#include "archergeneral.h"

/*********************************************************************
 * COMPILE OPTIONS
 */
//#define BLADEMASTER_DEBUG
//#define TEST_LED
#define POWER_ON_MEASURE
//#define CONNECT_MEASURE
//#define SI7015


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to check battery voltage (in ms)
#define BATTERY_CHECK_PERIOD          1000

// How often to perform periodic event
//#define SBP_PERIODIC_EVT_PERIOD                   5000
#define SBP_PERIODIC_EVT_PERIOD                   500

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

// How often (in ms) to read the accelerometer
#define ACCEL_READ_PERIOD             50
#define PM_READ_PERIOD                500

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( GENERAL_SERVICE_UUID ),
  HI_UINT16( GENERAL_SERVICE_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Blade Master        ";

// Accelerometer Profile Parameters
static uint8 accelEnabler = FALSE;
static uint32 accelSamplePeriod = ACCEL_READ_PERIOD;      // Default=50ms
static uint8 pmEnabler = FALSE;
static uint32 pmSamplePeriod = PM_READ_PERIOD;          // Default=500ms

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void rssiReadCB( int8 value );
static void performPeriodicTask( void );
static void generalProfileChangeCB( uint8 paramID );
static void accelProfileChangeCB( uint8 paramID);
static void accelRead( void );
static void ledProfileChangeCB(void);

#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined (TEST_LED)
static uint8 periodicR = 0;
static uint8 periodicG = 85;
static uint8 periodicB = 171;
#endif

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  rssiReadCB                      // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// General Profile Callbacks
static generalCBs_t keyFob_GeneralCBs = 
{
  generalProfileChangeCB,  
};

// Accelerometer Profile Callbacks
static accelCBs_t keyFob_AccelCBs =
{
  accelProfileChangeCB,    // Called when Enabler or Sample Period changes
};

// Led Profile Callbacks
static ledCBs_t keyFob_LedCBs =
{
  ledProfileChangeCB,
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
    // For other hardware platforms, device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    
    // RSSI
    uint16 rssi_read_rate = 1000;
    GAPRole_SetParameter( GAPROLE_RSSI_READ_RATE, sizeof( uint16 ), &rssi_read_rate );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    //uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE; // Disable bonding in developing
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif
  General_AddService (GATT_ALL_SERVICES);     // General Profile
  Accel_AddService( GATT_ALL_SERVICES );      // Accelerometer Profile
  Led_AddService( GATT_ALL_SERVICES );        // Led Profile
  Batt_AddService();

#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO
  

  P0DIR = 0x86; // O P0.7 smkON, PM2.5 sensor enable
                // I P0.6 smkSENSOR, PM2.5 sensor output (Analog)
                // I P0.5 V_Measure, Battery voltage measurement (Analog)
                // I P0.4 batPGn, USB charger (BQ25040) power good
                // I P0.3 batCHGn, USB charger (BQ25040) charge done
                // O P0.2 batEN/SET, USB charger (BQ25040) enable/mode select
                // O P0.1 mtCSn, Temperature and moisture sensor (Si7015) chip select
                // I P0.0 TOUCH, Touch sensor (PCF8883) output
  P1DIR = 0xFF; // O P1.7 accINT (CMA3000) Initialize later
                // O P1.6 Unused
                // O P1.5 accMOSI (CMA3000) Initialize later
                // O P1.4 accMISO (CMA3000) Initialize later
                // O P1.3 accCLK (CMA3000) Initialize later
                // O P1.2 accCBS (CMA3000) Initialize later
                // O P1.1 P1_1, LED1
                // O P1.0 P1_0, LED0
  P2DIR = 0x1F; // O P2.4 Unused
                // O P2.3 Unused
                // O P2.2 DC
                // O P2.1 DD
                // O P2.0 Unused

  P0 = 0x7B; // All pins on port 0 to low except for inputs, P0.1
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

  APCFG |= 0x60; // Set P0.6 smkSENSOR, P0.5 V_Measure to analog input
  
  
#endif // #if defined( CC2540_MINIDK )

  // Initialize the ADC for battery reads
  HalAdcInit();
  
  // Initialize the LEDs
  BMLedInit();
  
#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
  
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    
    // Set timer for first battery read event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_BATTERY_CHECK_EVT, BATTERY_CHECK_PERIOD );
    
    // Start the General Profile
    VOID General_RegisterAppCBs ( &keyFob_GeneralCBs );
    
    // Start the Accelerometer Profile
    VOID Accel_RegisterAppCBs( &keyFob_AccelCBs );
    
    // Start the Led Profile
    VOID Led_RegisterAppCBs( &keyFob_LedCBs);
#ifdef POWER_ON_MEASURE
    pmEnabler = 0x01;
    Accel_SetParameter( PM_ENABLER, 1, &pmEnabler );
    osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_PM_READ_EVT, pmSamplePeriod );
#endif
      
    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();
    
    return (events ^ SBP_PERIODIC_EVT);
  }
  
  if ( events & KFD_BATTERY_CHECK_EVT )
  {
    // Restart timer
    if ( BATTERY_CHECK_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_BATTERY_CHECK_EVT, BATTERY_CHECK_PERIOD );
    }

    // perform battery level check
    Batt_MeasLevel( );

    return (events ^ KFD_BATTERY_CHECK_EVT);
  }
  
  if ( events & KFD_ACCEL_READ_EVT )
  {
    bStatus_t status = Accel_GetParameter( ACCEL_ENABLER, &accelEnabler );

    if (status == SUCCESS)
    {
      if ( accelEnabler )
      {
        // Restart timer
        if ( accelSamplePeriod )
        {
          osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_ACCEL_READ_EVT, accelSamplePeriod );
        }

        // Read accelerometer data
        accelRead();
      }
      else
      {
        // Stop the acceleromter
        osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_ACCEL_READ_EVT);
      }
    }
    else
    {
        //??
    }
    return (events ^ KFD_ACCEL_READ_EVT);
  }
  
  if ( events & KFD_PM_READ_EVT )
  {
    if ( pmEnabler )
    {
      // Restart timer
      if ( pmSamplePeriod )
      {
        osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_PM_READ_EVT, pmSamplePeriod );
      }

      // Read PM data
      uint8 pmRaw = pmRead();
      uint8 color[3];
      GetPMRGB(pmRaw, color);
      BMShowColor(color[0], color[1], color[2], 0x08);
      Accel_SetParameter(PM_RAW, sizeof ( uint8 ), &pmRaw);
      
      // Read temp data
#if defined (SI7015)
      int16 temperature = tempRead();
      int8 tempToShow = (int8)(temperature/32-396);
      BMShowDigit(tempToShow);  
      Accel_SetParameter(TEMP_VALUE, sizeof ( int16 ), &temperature);
#endif
      
      // Read humid data
#if defined (SI7015)
      uint8 humility = humRead();
      BMShowHum(humility/16+1);  
      Accel_SetParameter(HUMID_VALUE, sizeof ( uint8 ), &humility);
#endif
    }
    else
    {
        // Stop the PM reader
      osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_PM_READ_EVT);
    }
    return (events ^ KFD_PM_READ_EVT);
  }  
  
#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }

  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
          
        #if defined BLADEMASTER_DEBUG
          BMShowHum(0);
        #endif
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if defined BLADEMASTER_DEBUG
          BMShowHum(1);
        #endif
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if defined BLADEMASTER_DEBUG
          BMShowHum(2);
        #endif
        
#ifdef CONNECT_MEASURE
        pmEnabler = 0x01;
        Accel_SetParameter( PM_ENABLER, 1, &pmEnabler );
        osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_PM_READ_EVT, pmSamplePeriod );
#endif  
        uint8 appConnect;
        General_GetParameter( APP_CONNECT , &appConnect );
        // Not connected before this
        if (appConnect == NOT_CONNECTED)
        {
          // This is a system auto connection
          uint8 newAppConnect = SYS_CONNECTED;
          General_SetParameter( APP_CONNECT , sizeof(uint8), &newAppConnect );
        }
        // Broadcast NON-Connectable ADV if no app connection
        if (appConnect != APP_CONNECTED)
        {
          uint8 current_adv_enabled_status = TRUE;
          GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &current_adv_enabled_status );
        }
      }
      break;
    case GAPROLE_CONNECTED_ADV:
      {
        #if defined BLADEMASTER_DEBUG
          BMShowHum(3);
        #endif
        HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK );
        HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
      }
      break;
    case GAPROLE_WAITING:
      {
#ifdef CONNECT_MEASURE
        pmEnabler = 0x00;
        Accel_SetParameter( PM_ENABLER, 1, &pmEnabler );
        osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_PM_READ_EVT);
        // Initialize the LEDs
        BMLedInit();
#endif
        HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
        uint8 writeValue = FALSE;
        General_SetParameter( ANCS_STATE, sizeof(uint8), &writeValue );
        
        // Broadcast Connectable ADV
        uint8 appConnect = NOT_CONNECTED;
        General_SetParameter( APP_CONNECT , sizeof(uint8), &appConnect );
        uint8 current_adv_enabled_status = TRUE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &current_adv_enabled_status );
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
#ifdef CONNECT_MEASURE
        pmEnabler = 0x00;
        Accel_SetParameter( PM_ENABLER, 1, &pmEnabler );
        osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_PM_READ_EVT);
        // Initialize the LEDs
        BMLedInit();
#endif
        HalLedSet(HAL_LED_2, HAL_LED_MODE_BLINK );
        uint8 writeValue = FALSE;
        General_SetParameter( ANCS_STATE, sizeof(uint8), &writeValue );
        
        // Broadcast Connectable ADV
        uint8 appConnect = NOT_CONNECTED;
        General_SetParameter( APP_CONNECT , sizeof(uint8), &appConnect );
        uint8 current_adv_enabled_status = TRUE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &current_adv_enabled_status );
      }
      break;

    case GAPROLE_ERROR:
      {
        #if defined BLADEMASTER_DEBUG
          BMShowHum(4);
        #endif
      }
      break;

    default:
      {
        ;
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      rssiReadCB
 *
 * @brief   The function gets called every time the RSSI has been read.
 *
 * @param   value - RSSI value
 *
 * @return  none
 */
static void rssiReadCB( int8 value )
{
   General_SetParameter( RSSI_VALUE, sizeof(int8), &value );
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
  uint8 valueToCopy;
  uint8 stat;

  // Call to retrieve the value of the third characteristic in the profile

  if( stat == SUCCESS )
  {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
  }
  
  //int16 temperature = tempRead();
  BMShowDigit(battRead( ));
#ifdef TEST_LED
  BMShowDigit(periodicR);
  BMShowColor(periodicR, periodicG, periodicB, 0x08);
  BMShowHum(periodicR%7);
  
  periodicR = periodicR + 8;
  periodicG = periodicG + 19;
  periodicB = periodicB + 23;
#endif
}

/*********************************************************************
 * @fn      accelProfileChangeCB
 *
 * @brief   Called by the Accelerometer Profile when the Enabler or Sample Period Attribute
 *          is changed.
 *
 * @param   none
 *
 * @return  none
 */
static void accelProfileChangeCB( uint8 paramID )
{
  switch (paramID)
  {
  case ACCEL_ENABLER:
    Accel_GetParameter( ACCEL_ENABLER, &accelEnabler );
    if (accelEnabler)
    {
      // Initialize accelerometer
      accInit();
      // Setup timer for accelerometer task
      osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_ACCEL_READ_EVT, accelSamplePeriod );
    } else
    {
      // Stop the acceleromter
      accStop();
      osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_ACCEL_READ_EVT);
    }
    break;
  case ACCEL_SAMPLEPERIOD:
    Accel_GetParameter( ACCEL_SAMPLEPERIOD, &accelSamplePeriod );
    break;
  case PM_ENABLER:
    Accel_GetParameter( PM_ENABLER, &pmEnabler );
    if (pmEnabler)
    {
      // Setup timer for PM measurement task
      osal_start_timerEx( simpleBLEPeripheral_TaskID, KFD_PM_READ_EVT, pmSamplePeriod );
    } else
    {
      // Stop the PM measurement
      osal_stop_timerEx( simpleBLEPeripheral_TaskID, KFD_PM_READ_EVT);
    }
    break;
  case PM_SAMPLEPERIOD:
    Accel_GetParameter( PM_SAMPLEPERIOD, &pmSamplePeriod );
    break;
  default:
    // Should not reach here!
    break;
  }
}

static void generalProfileChangeCB( uint8 paramID )
{
  if (paramID == APP_CONNECT)
  {
    // Turn off all Adv
    uint8 appConnect;
    General_GetParameter( APP_CONNECT , &appConnect );
    if (appConnect == APP_CONNECTED)
    {
      uint8 current_adv_enabled_status = FALSE;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof(uint8), &current_adv_enabled_status );
    }
  }
}

static void ledProfileChangeCB(void)
{
  ;
}

/*********************************************************************
 * @fn      accelRead
 *
 * @brief   Called by the application to read accelerometer data
 *          and put data in accelerometer profile
 *
 * @param   none
 *
 * @return  none
 */
static void accelRead( void )
{

  static int8 x, y, z;
  int8 new_x, new_y, new_z;

  // Read data for each axis of the accelerometer
  accReadAcc(&new_x, &new_y, &new_z);
  
  // Save all samples
  /*
  if (sample_idx < 512)
  {
    accx[sample_idx] = new_x;
    accy[sample_idx] = new_y;
    accz[sample_idx] = new_z;
    sample_idx = sample_idx + 1;
  }
  else
  {
    sample_idx = 0;
  }
*/
  
  // Check if x-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (x < (new_x-ACCEL_CHANGE_THRESHOLD)) || (x > (new_x+ACCEL_CHANGE_THRESHOLD)) )
  {
    x = new_x;
    Accel_SetParameter(ACCEL_X_ATTR, sizeof ( int8 ), &x);
  }

  // Check if y-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (y < (new_y-ACCEL_CHANGE_THRESHOLD)) || (y > (new_y+ACCEL_CHANGE_THRESHOLD)) )
  {
    y = new_y;
    Accel_SetParameter(ACCEL_Y_ATTR, sizeof ( int8 ), &y);
  }

  // Check if z-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (z < (new_z-ACCEL_CHANGE_THRESHOLD)) || (z > (new_z+ACCEL_CHANGE_THRESHOLD)) )
  {
    z = new_z;
    Accel_SetParameter(ACCEL_Z_ATTR, sizeof ( int8 ), &z);
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

/*********************************************************************
*********************************************************************/
