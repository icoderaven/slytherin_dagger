//supervisor.h
// header file for the supervisor
//

/*****************************************************
  return values for subroutines
*****************************************************/

// Bit definitions for checking timers
#define CHECK_PACKET_ERROR_RATE 1
#define CHECK_PACKET_TIMEOUT    2
#define CHECK_JOYSTICK_TIMEOUT  4




// Timer related variables and constants
// Number of timers
#define N_TIMERS   3
//initial time interval when starting software. Set to 1000 ms to deal with init
#define INITIAL_INTERVAL   1000
// Index into vector of timer objects
#define PACKET_ERROR_CHECK   0
#define PACKET_TIMEOUT       1
#define JOYSTICK_TIMEOUT     2
// values used for each type of timer, units of milliseconds
#define TVAL_CRC        1000
#define TVAL_PACKET     10000
#define TVAL_JOYST       200

DWORD TimeoutVal[] = {TVAL_CRC, TVAL_PACKET, TVAL_JOYST,0};

/*************************************************************
   timer structure
  The variable CheckValue stores a value whose significance
  depends on what type event we are timing.
  for PACKET_ERROR_CHECK it keeps the error count from previous timeout
  PACKET_TIMEOUT it keeps the previous packet sequence number
  JOYSTIC_TIMEOUT it keeps the previous packet sequence number
****************************************************************/


typedef struct CTimersStruct{
        DWORD  Valid;        //timer in use
        DWORD  FinalValue;   //time when expires, relative to start of system (ms)
} CTimer;

/***************************************
   Message Queue
****************************************/
#define NUM_MESSAGES     30
#define STRING_LENGTH    100
typedef struct CMessageQueueStruct{
        int Valid;
        int Type;
        char  Msg[STRING_LENGTH];
} CMessageQueue;

CMessageQueue CMessQ[NUM_MESSAGES];

// routines
int CheckTimers();
int GetEstopStatus();
int ErrorStateMachine();
int QueueMessage(unsigned short source, ProcErr *pErr);
int InitTimers();
int OutPutMessages();
int RelayTest(int reset_test);

