//ErrMessages.h

//DCU Error Messages. Same order as CardioError.h

char *DCU_Err_Messages[] = {
     "WRN_DCU: Excessive CRC errors in data packets from feeder unit",	//0
     "WRN_DCU: Unable to open log file",								//1
     "WRN_DCU: Unable to write to log file",							//2
     "WRN_DCU: The beeper returns an error",							//3
	 "WRN_DCU: DCU should be rebooted so maximum uptime is not exceeded",//4
     "DCU: Undefined error code 5",										//5
     "DCU: Undefined error code 6",										//6
	 "ERR_DCU: Incompatible firmware version",							//7
     "ERR_DCU: CRC error in configuration file",						//8
     "ERR_DCU: Out of range variable in configuration file",			//9
     "ERR_DCU: Corrupt DCU executable image",							//10
     "ERR_DCU: CRC error in global configuration data structure",		//11
     "ERR_DCU: Timeout waiting for feeder data packet",					//12
     "ERR_DCU: Timeout waiting for data from joystick",					//13
     "ERR_DCU: CRC error in joystick data",								//14
     "ERR_DCU: Undefined error code 15", // Used to be joy motion lim   //15
     "ERR_DCU: Motor command out of range",								//16
     "ERR_DCU: Illegal motor index",									//17
     "ERR_DCU: Illegal snake index",									//18
     "ERR_DCU: Force command out of range",								//19
     "ERR_DCU: Unable to initialize serial link",						//20
     "ERR_DCU: Illegal state transition",								//21
     "ERR_DCU: During lock position goal achieved without reaching force limit",										//22
     "ERR_DCU: Error reading serial port",								//23
     "ERR_DCU: E-stop circuit opened",									//24
     "ERR_DCU: Inner snake binding",									//25
	 "ERR_DCU: Illegal sequence number",								//26
	 "ERR_DCU: Keithley DIO error",                                     //27
	 "ERR_DCU: DCU exceeded maximum time allowed without reboot",       //28
	 "ERR_DCU: E-stop button pressed",                                  //29
	 "ERR_DCU: Joystick unplugged",										//30
	 "ERR_DCU: Relay test failure"										//31
};

//Base processor Error Messages. Same order as in CardioError.h
char *BP_Err_Messages[] = {
     "BP: Undefined Error Code 0",                                      // 0
     "BP: Undefined Error Code 1",                                      // 1
     "BP: R2 Error Detected",                                           // 2
     "BP: Undefined Error Code 3",                                      // 3
     "WRN_BP: Excessive CRC errors in data packets from Outer Proc",    // 4
     "ERR_BP: Timeout waiting for packet from Outer Proc.",             // 5
     "WRN_BP: Excessive CRC errors in data packets from DCU",           // 6
     "ERR_BP: Timeout waiting for data packet from DCU",                // 7
     "ERR_BP: Watchdog timeout, base processor reset",                  // 8
     "ERR_BP: Voltage supply out of range",                             // 9
     "ERR_BP: Safety Circuit opened",                                   //10
	 "ERR_BP: Up-time limit exceeded"                                   //11
};

//Outer processor Error Messages. Same order as in CardioError.h
char *OP_Err_Messages[] = {
     "OP: Undefined Error Code 0",                                      // 0
     "OP: Undefined Error Code 1",                                      // 1
     "OP: R2 Error Detected",                                           // 2
     "OP: Undefined Error Code 3",                                      // 3
     "WRN_OP: Excessive CRC errors in data packets from Base Proc.",    // 4
     "ERR_OP: Timeout waiting for data packet from Base Proc.",         // 5
     "OP: Undefined Error Code 6",                                      // 6
     "OP: Undefined Error Code 7",                                      // 7
     "ERR_OP: Watchdog timeout, outer processor reset",                 // 8
     "ERR_OP: Voltage supply out of range",                             // 9
     "ERR_OP: Safety circuit opened",                                   //10
	 "ERR_OP: Up-time limit exceeded"                                   //11
};

char *MOT_Err_Messages[]={
     "ERR_MOT: Current sensor mismatch",                                // 0
     "ERR_MOT: Enocder mismatch, power must be recycled",               // 1
     "ERR_MOT: Error in trajectory generator",                          // 2
     "ERR_MOT: PID tracking error",                                     // 3
     "ERR_MOT: Force tracking error",                                   // 4
     "ERR_MOT: Rate of change of current exceeds limit",                // 5
	 "ERR_MOT: Maximum motor current exceeded"                          // 6
};
         
// Informational messages

char *Infor_Messages[]={
     "INFO: Return to run state"
     };
     
     
