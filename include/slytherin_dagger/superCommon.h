//superCommon.h

#define OKAY 1
#define SERR 0

#define M_DCU   1
#define M_BP    2
#define M_OP    3
#define M_INFO  4

// define states for error state machine.

#define RUN_STATE        1     //Normal operation no errors detected
#define ERROR_STATE      2     // An error has been detected by at least one processor
#define SAFE_STATE       3     // All processors have acknowledged a detected error
#define SAFE_STATE_NO_RECOVER 4 // Safe state, but unable to recover
#define RECOVERY_STATE   5     // Transition state from SAFE_STATE to RUN_STATE    

// actions for supervisor
#define INIT_SUPERVISOR  1
#define RUN_SUPERVISOR   2

int RunSupervisor(unsigned int Action);
int SetErrorBit1(unsigned short source, unsigned int mask);

#define SetErrorBit(source, mask) \
	do { \
	printf("%s:%d SetErrorBit(%d, 0x%x)\n", __FILE__, __LINE__, source, mask); \
	SetErrorBit1(source, mask); \
	} while (0)

