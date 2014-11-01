// CardioError.h
// list of errors that can be detected on each processor in CardioARM  System.


//******************************************************************
// Bit definitions for data packet error words
// BP - Base Processor
// OP - Outer Processor
/*****************************************************************
******************************************************
 ID for processors
******************************************************/

#define M_DCU   1
#define M_BP    2
#define M_OP    3
#define M_INFO  4
#define M_MOT_B1 5
#define M_MOT_B2 6
#define M_MOT_B3 7
#define M_MOT_O1 8
#define M_MOT_O2 9
#define M_MOT_O3 10

// data structure to hold warnings and errors

typedef struct ProcErrStruct{
        unsigned int DcuDword;
        unsigned int BaseDword;
        unsigned int OuterDword;
        unsigned int InfoDword;
        unsigned int MotorDword[6];
} ProcErr;


//  ACCEPTABLE_ERROR_RATE    for bad packets (per second)

#define ACCEPTABLE_ERROR_RATE    5

#define MAX_POS_INNER_SLIDE   300.0 // was 20
#define MAX_POS_OUTER_SLIDE   300.0 // was 20

// max amount feeder seq number can be behind by
#define MAX_SEQ_DIFF 50

#define DCU_ERROR_BIT    0x1
#define BP_ERROR_BIT     (DCU_ERROR_BIT <<1)
#define OP_ERROR_BIT     (BP_ERROR_BIT <<1)
#define CLEAR_BIT        (OP_ERROR_BIT << 1)
#define ERROR_MASK       0x7
#define RESET_BIT        (CLEAR_BIT << 1)

// Bit definitions for DCU Warning and error codes
//DCU warnings
#define DCU_WRN_BAD_PACKETS    0x1       //excessive number of bad packets from feeder
#define DCU_WRN_LOG_FILE_INIT  (0x1 << 1)      // Cannot open the system log file
#define DCU_WRN_LOG_WRITE      (0x1 << 2)      //unable to write to the log file
#define DCU_WRN_BEEP_ERROR     (0x1 << 3)         // the beeper returns an error
#define DCU_WRN_UPTIME         (0x1 << 4)      // DCU has gone long enough w/o a reboot to generate a warning
// DCU errors
#define DCU_ERR_INCOMPAT_FW    (0x100 >> 1)   // Incompatible firmware version
#define DCU_ERR_BAD_CONFIG      0x100                          //corrupt data in config file
#define DCU_ERR_OUT_OF_RANGE   (0x100 << 1)    // Out of range value in confog file
#define DCU_ERR_IMAGE_CRC      (0x100 << 2)                    // DCU executable image is corrupt
#define DCU_ERR_CONFIG_CRC     (0x100 << 3)   // CRC error in configuration global structure

#define DCU_ERR_TIMEOUT_FDR    (0x100 << 4)     //timeout waiting for packet data from feeder
#define DCU_ERR_TIMEOUT_JOY    (0x100 << 5)  //timeout waiting for data from joystick
#define DCU_ERR_JOYSTICK_CRC   (0x100 << 6)     //CRC error in joystick data (USB handles internally)
// Formerlly, (0x100 << 7) was joystick motion limit violation. This error was
// removed on 10/12/2009

#define DCU_ERR_DIST_MOTOR     (0x100 << 8)     //Motor command out of range
#define DCU_ERR_INDEX_MOTOR    (0x100 << 9)    //Illegal motor index
#define DCU_ERR_INDEX_SNAKE    (0x100 << 10)    //Illegal snake index
#define DCU_ERR_FORCE_CMD      (0x100 << 11)    // Force command out of range

#define DCU_ERR_INIT_SERIAL    (0x100 << 12)    //error initializing serial port
#define DCU_ERR_STATE_TRANS    (0x100 << 13)    //illegal state transition
#define DCU_ERR_FORCE_LOCK     (0x100 << 14)   //Position goal achieved without reaching force limit
#define DCU_ERR_READ_SERIAL    (0x100 << 15)   //error reading serial port

#define DCU_ERR_E_STOP         (0x100 << 16)   //E stop circuit opened
#define DCU_INNER_SNAKE_BIND   (0x100 << 17)         //inner snake binding
#define DCU_ERR_ILLEGAL_SEQ    (0x100 << 18)   //Illegal sequence number detected
#define DCU_ERR_KEITHLEY       (0x100 << 19)   //Keithley DIO error
#define DCU_ERR_UPTIME         (0X100 << 20)   //DCU has exeeded maximum allowed time without a reboot
#define DCU_ERR_E_STOP_PRESSED (0x100 << 21)   //Estop button has been pressed
#define DCU_ERR_NO_JOY         (0x100 << 22)   //Lost joystick
#define DCU_ERR_RELAY_FAIL     (0x100 << 23)   //Relay test failure

#define DCU_WRN_MASK           0xFF
#define DCU_ERROR_MASK         0xFFFFFF80
#define DCU_NO_RECOVER         (DCU_ERR_INCOMPAT_FW | DCU_ERR_INDEX_MOTOR | \
								DCU_ERR_INDEX_SNAKE | DCU_ERR_STATE_TRANS | \
								DCU_ERR_ILLEGAL_SEQ | DCU_ERR_UPTIME | \
								DCU_ERR_KEITHLEY) //Errors we can't recover from


#define RET_TO_RUN       0x1         //return to run state



