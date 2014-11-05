// ErrorData.h


// data structure to hold warnings and errors
#include <stdbool.h>

extern ProcErr ErrP;
extern bool RecoverFlag;
extern float    CurrentJoyX,CurrentJoyY;
extern unsigned int DcuErrorFlags, BPErrorFlags, OPErrorFlags;
extern unsigned long PresentJoyStickPacketNumber;
extern float CurrentJoyX,CurrentJoyY;
extern unsigned int ErrorState;
