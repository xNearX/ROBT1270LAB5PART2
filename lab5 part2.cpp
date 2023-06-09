/**********************************************************************************************************************
Course: ROBT1270 - C Programming

Program: Lab5: SCARA Robot Simulator Intermediate Contol

Purpose: To demonstrate basic control over the SCARA Robot Simulator using forward and inverse kinematic
         functions.  Remote commands are sent to the simulator using formatted command strings.

         Programming methods: formatted I/O, conditional statements, pointers, functions, strings,
                              arrays, structures, file I/O, dynamic memory allocation.

Author: ????

Declaration: I/We, ????, declare that the following program was written by me/us.

Date Created: April 07 2023

**********************************************************************************************************************/

//-------------------------- Standard library prototypes --------------------------------------------------------------
#include <stdlib.h>  // standard functions and constant
#include <stdio.h>   // i/o functions
#include <math.h>    // math functions
#include <string.h>  // string functions
#include <ctype.h>   // character functions
#include <stdbool.h> // bool definitions
#include "robot.h"   // robot functions

//---------------------------- Program Constants ----------------------------------------------------------------------
const double PI = 3.14159265358979323846;    // the one and only
const double L1 = 350.0;                     // length of the inner arm
const double L2 = 250.0;                     // length of the outer arm
const double ABS_THETA1_DEG_MAX = 150.0;     // maximum magnitude of shoulder angle in degrees
const double ABS_THETA2_DEG_MAX = 170.0;     // maximum magnitude of elbow angle in degrees
const double LMAX = L1 + L2;                 // max L -> maximum reach of robot
const double LMIN = sqrt(L1 * L1 + L2 * L2 - 2.0 * L1 * L2 * cos(PI - ABS_THETA2_DEG_MAX * PI / 180.0)); // min L

const unsigned char HL = 196;                // for console (code page 437)
const unsigned char FHL = 151;               // for file (code page 1252)
const unsigned char PLUSMINUS_SYMBOL = 241;  // the plus/minus ascii symbol
const unsigned char DEGREE_SYMBOL = 248;     // the degree symbol

const double ERROR_VALUE = DBL_MAX;  // value for angles when robot can't reach

const char* seps = "\t,\n ;:";       // for tokenizing the line string

// number of points on path for every 500 units of arc length
const int LOW_RESOLUTION_POINTS_PER_500_UNITS = 11;
const int MEDIUM_RESOLUTION_POINTS_PER_500_UNITS = 31;
const int HIGH_RESOLUTION_POINTS_PER_500_UNITS = 51;

const int PRECISION = 2;      // for printing values to console
const int FIELD_WIDTH = 8;    // for printing values to console

const int COMMAND_INDEX_NOT_FOUND = -1;   // used when command index not found
const int BLANK_LINE = -2;                // used to signal a blank line in the input file


#define COMMAND_STRING_ARRAY_SIZE 502  // size of array to store commands written by sprintf_s for robot. 
// NOTE: 2 elements must be reserved for trailing '\n' and '\0'

#define MAX_LINE_SIZE 1002             // size of array to store a line from a file. 
                                       // NOTE: 2 elements must be reserved for trailing '\n' and '\0'


enum ARM { LEFT, RIGHT };                                                  // left arm or right arm configuration
enum MOTOR_SPEED { MOTOR_SPEED_LOW, MOTOR_SPEED_MEDIUM, MOTOR_SPEED_HIGH }; // motor speed
enum RESOLUTION { RESOLUTION_LOW, RESOLUTION_MEDIUM, RESOLUTION_HIGH };     // motor speed
enum CURRENT_ANGLES { GET_CURRENT_ANGLES, UPDATE_CURRENT_ANGLES };         // used to get/update current SCARA angles

enum COMMAND_INDEX  // list of all command indexes
{
   ROTATE_JOINT, MOTOR_SPEED, PEN_UP, PEN_DOWN, CYCLE_PEN_COLORS, PEN_COLOR, CLEAR_TRACE,
   CLEAR_REMOTE_COMMAND_LOG, CLEAR_POSITION_LOG, SHUTDOWN_SIMULATION, END, HOME, LINE, ARC, MOVE_TO, NUM_COMMANDS
};

//---------------------------- Structure Definitions ------------------------------------------------------------------

// structure to map command keyword string to a command index
typedef struct COMMAND
{
   const int index;
   const char* strCommand;
}
COMMAND;

// RGB color
typedef struct RGB
{
   int r, g, b;  // ranges are 0-255
}
RGB;

// SCARA tooltip coordinates
typedef struct TOOL_POSITION
{
   double x, y;
}
TOOL_POSITION;

// SCARA joint angles (degrees)
typedef struct JOINT_ANGLES
{
   double theta1Deg, theta2Deg;
}
JOINT_ANGLES;

// pen state
typedef struct PEN_STATE
{
   RGB penColor;
   int penPos;
}
PEN_STATE;

// forward kinematics solution data 
typedef struct FORWARD_SOLUTION
{
   TOOL_POSITION toolPos;  // tool tip coordinates
   bool bCanReach;         // true if robot can reach, false if not
}
FORWARD_SOLUTION;

// inverse kinematics solution data 
typedef struct INVERSE_SOLUTION
{
   JOINT_ANGLES jointAngles[2];  // joint angles (in degrees).  Left and Right arm solutions
   bool bCanReach[2];            // true if robot can reach, false if not.  Left and right arm configurations
}
INVERSE_SOLUTION;

typedef struct PATH_CHECK
{
   bool bCanDraw[2];    // true if robot can draw, false if not.  Left and right arm configurations
   double dThetaDeg[2]; // total angle changes required to draw path
}
PATH_CHECK;

//----------------------------- Globals -------------------------------------------------------------------------------
// global array of command keyword string to command index associations
// NOTE:  CYCLE_PEN_COLORS must preceed PEN_COLOR
const COMMAND m_Commands[NUM_COMMANDS] = { {ROTATE_JOINT, "ROTATE_JOINT"}, {MOTOR_SPEED, "MOTOR_SPEED"},
                                          {PEN_UP, "PEN_UP"}, {PEN_DOWN, "PEN_DOWN"},
                                          {CYCLE_PEN_COLORS, "CYCLE_PEN_COLORS"}, {PEN_COLOR, "PEN_COLOR"},
                                          {CLEAR_TRACE, "CLEAR_TRACE"},
                                          {CLEAR_REMOTE_COMMAND_LOG, "CLEAR_REMOTE_COMMAND_LOG"},
                                          {CLEAR_POSITION_LOG, "CLEAR_POSITION_LOG"},
                                          {SHUTDOWN_SIMULATION, "SHUTDOWN_SIMULATION"}, {END, "END"}, {HOME, "HOME"},
                                          {LINE, "LINE"}, {ARC, "ARC"}, {MOVE_TO, "MOVE_TO"} };

CRobot robot;        // the global robot Class.  Can be used everywhere
FILE* flog = NULL;   // the global log file

//----------------------------- Function Prototypes -------------------------------------------------------------------
bool flushInputBuffer();               // flushes any characters left in the standard input buffer
void waitForEnterKey();                // waits for the Enter key to be pressed
int nint(double);                      // computes nearest integer to a double value
double degToRad(double);               // returns angle in radians from input angle in degrees
double radToDeg(double);               // returns angle in degrees from input angle in radians
double mapAngle(double);               // make sure inverseKinematic angled are mapped in range robot understands
void pauseRobotThenClear();            // pauses the robot for screen capture, then clears everything
void printHLine(int N);                // prints a solid line to the console
int dsprintf(char const*, ...);       // prints to log file and to console 
void makeStringUpperCase(char*);      // makes an input string all upper case
size_t getNumPathPoints(double, int);  // gets the number of points on a path based on arc length and resolution value
void robotAngles(JOINT_ANGLES*, int); // gets or updates the current SCARA angles

void processFileCommands();            // gets commands out of a file and processes them for robot control
bool setCyclePenColors(char* strLine); // Parses line string to send a CYCLE_PEN_COLORS command to robot
bool setPenColor(char* strLine);
bool setRotateJoint(char* strLine);
bool setMoveTo(char* strLine);
bool setMotorSpeed(char* strLine);
bool setLine(char *strLine);

void processCommand(int commandIndex, char* strCommandLine);   // processes a command string from the file
int getCommandIndex(const char* strLine);                      // gets the command keyword index from a string

INVERSE_SOLUTION inverseKinematics(TOOL_POSITION); // get left/right arm joint angles from x,y pos


//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Program to demonstrate basic control of the SCARA robot simulator
// ARGUMENTS:    none
// RETURN VALUE: an int that tells the O/S how the program ended.  0 = EXIT_SUCCESS = normal termination
int main()
{
   // open connection with robot
   if (!robot.Initialize()) return 0;

   processFileCommands();

   dsprintf("\n\nPress ENTER to end the program...\n");
   waitForEnterKey();
   return EXIT_SUCCESS;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  processes robot commands stored in a file and uses them to control the SCARA robot
// ARGUMENTS:    none
// RETURN VALUE: none
void processFileCommands()
{
   char strFileName[MAX_PATH];                  // stores input file name
   char strLine[MAX_LINE_SIZE];                 // stores one line out of input file
   FILE* fi = NULL;                             // input file handle
   errno_t err;                                 // stores fopen_s error value
   int numChars;                                // used to draw dividing line
   int nLine;                                   // file line number
   int commandIndex = COMMAND_INDEX_NOT_FOUND;  // command index

   // open the log file (mirrors console output to log.txt if dsprintf used instead of printf)
   err = fopen_s(&flog, "log.txt", "w");
   if (err != 0 || flog == NULL)
   {
      dsprintf("Cannot open log.txt for writing!  Press ENTER to end program...");
      waitForEnterKey();
      exit(0);
   }

   // get the input file
   while (true)
   {
      dsprintf("Please enter the name of the commands file: ");
      fgets(strFileName, MAX_PATH, stdin);
      strFileName[strlen(strFileName) - 1] = '\0';  // remove newline character

      err = fopen_s(&fi, strFileName, "r");
      if (err == 0 && fi != NULL) break;

      dsprintf("Failed to open %s!\nError code = %d", strFileName, err);
      if (err == ENOENT)
         dsprintf(" (File not found!  Check name/path)\n");
      else if (err == EACCES)
         dsprintf(" (Permission Denied! Is the file opened in another program?)\n");
      else
         dsprintf("\n");
   }
   numChars = dsprintf("Processing %s\n", strFileName);
   printHLine(numChars - 1);

   // get each line from the input file and process the command
   nLine = 0;
   while (fgets(strLine, MAX_LINE_SIZE, fi) != NULL)
   {
      if (strstr(strLine, "\n") == NULL) strcat_s(strLine, MAX_LINE_SIZE, "\n"); // needed for last line

      nLine++;
      dsprintf("Line %02d: %s", nLine, strLine);  // echo the line

      //--- get the command index and process it 
      makeStringUpperCase(strLine);  // make line string all upper case (makes commands case-insensitive)

      //**** YOUR CODE FOR getCommandIndex and processCommand GOES HERE ****

      printf("Looking for command\n");
      commandIndex = getCommandIndex(strLine);
      if (commandIndex != COMMAND_INDEX_NOT_FOUND)
      {
         printf("Command was found\n");
         processCommand(commandIndex, strLine);
      }
      else
      {
         printf("Command not found\n");
      }
   }
   fclose(fi);
   fclose(flog);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  processes a command referenced by the commandIndex.  Parses the command string from the file and 
//               packages up the command to be sent to the robot if no errors found.  
// ARGUMENTS:    commandIndex:  index of the command keyword string
//               strCommandLine: command line from the file in the form of a string
// RETURN VALUE: none
void processCommand(int commandIndex, char* strCommandLine)
{
   bool bSuccess = true;
   JOINT_ANGLES homeAngles = { 0.0, 0.0 };

   switch (commandIndex)
   {
   case PEN_UP:
      robot.Send("PEN_UP\n");
      break;
   case PEN_DOWN:
      robot.Send("PEN_DOWN\n");
      break;
   case CLEAR_TRACE:
      robot.Send("CLEAR_TRACE\n");
      break;
   case CLEAR_REMOTE_COMMAND_LOG:
      robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");
      break;
   case CLEAR_POSITION_LOG:
      robot.Send("CLEAR_POSITION_LOG\n");
      break;
   case SHUTDOWN_SIMULATION:
      robot.Send("SHUTDOWN_SIMULATION\n");
      break;
   case END:
      robot.Send("END\n");
      break;
   case HOME:
      robot.Send("HOME\n");
      robotAngles(&homeAngles, UPDATE_CURRENT_ANGLES);
      break;
   case PEN_COLOR:
      bSuccess = setPenColor(strCommandLine);
      break;
   case CYCLE_PEN_COLORS:
      bSuccess = setCyclePenColors(strCommandLine);
      break;
   case ROTATE_JOINT:
      bSuccess = setRotateJoint(strCommandLine);
      break;
   case MOVE_TO:
      bSuccess = setMoveTo(strCommandLine);
      break;
   case MOTOR_SPEED:
      bSuccess = setMotorSpeed(strCommandLine);
      break;
   case LINE:
      bSuccess = setLine(strCommandLine);
      break;
   case ARC:
      //*** ADD CODE ***
      break;
   default:
      dsprintf("unknown command!\n");
   }

   if (bSuccess) dsprintf("Command sent to robot!\n\n");
}
int getCommandIndex(const char* strLine)
{
   char* tok = NULL;
   char* nextTok = NULL;
   char strLineCopy[MAX_LINE_SIZE];

   if (strLine == NULL || strlen(strLine) == 0)
   {
      return BLANK_LINE;
   }

   strcpy_s(strLineCopy, MAX_LINE_SIZE, strLine);
   tok = strtok_s(strLineCopy, seps, &nextTok);

   if (tok == NULL) {
      return COMMAND_INDEX_NOT_FOUND;
   }

   for (int i = 0; i < NUM_COMMANDS; i++) {
      if (strcmp(tok, m_Commands[i].strCommand) == 0) {
         return m_Commands[i].index;
      }
   }
   return COMMAND_INDEX_NOT_FOUND;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  parses a command string that contains CYCLE_PEN_COLORS and sends command to robot if data ok.
// ARGUMENTS:    strLine:  A file line string.
// RETURN VALUE: true if command sent to robot, false if not.
bool setCyclePenColors(char* strLine)
{
   char* tok = NULL, * nextTok = NULL;              // for tokenizing the line string
   char cmd[COMMAND_STRING_ARRAY_SIZE];            // command string for sprintf_s

   tok = strtok_s(strLine, seps, &nextTok); // CYCLE_PEN_COLORS keyword (discarded)

   tok = strtok_s(NULL, seps, &nextTok);  // parameter should be "ON" or "OFF"
   if (tok == NULL)
   {
      dsprintf("Missing CYCLE_PEN_COLORS parameter!\n\n");
      return false;
   }

   // Got token.  Check if token is "ON" or "OFF"
   if (strcmp(tok, "ON") != 0 && strcmp(tok, "OFF") != 0)
   {
      dsprintf("Invalid parameter for CYCLE_PEN_COLORS!  Must be ON or OFF.\n\n");
      return false;
   }

   // all good.  Send command.
   sprintf_s(cmd, COMMAND_STRING_ARRAY_SIZE, "CYCLE_PEN_COLORS %s\n", tok);
   robot.Send(cmd);
   return true;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  get or update current robot shoulder and elbow angles
// ARGUMENTS:    pAngles:  shoulder/joint angles.
//               getOrUpdate:  set to UPDATE_CURRENT_ANGLES to update the current angles
//                             set to GET_CURRENT_ANGLES to retrieve the current angles
// RETURN VALUE: none
void robotAngles(JOINT_ANGLES* pAngles, int getOrUpdate)
{
   static JOINT_ANGLES currentAngles = { 0.0, 0.0 };   // NOTE:  robot must be in home position when program starts!

   if (pAngles == NULL) // safety
   {
      dsprintf("NULL JOINT_ANGLES pointer! (robotAngles)");
      return;
   }

   if (getOrUpdate == UPDATE_CURRENT_ANGLES)
      currentAngles = *pAngles;
   else if (getOrUpdate == GET_CURRENT_ANGLES)
      *pAngles = currentAngles;
   else
      dsprintf("Unknown value for getOrUpdate (robotAngles)");
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Pauses the robot then clears everthing after user presses ENTER
// ARGUMENTS:    none
// RETURN VALUE: none
void pauseRobotThenClear()
{
   waitForEnterKey();
   system("cls");
   robot.Send("HOME\n");
   robot.Send("CLEAR_TRACE\n");
   robot.Send("PEN_COLOR 0 0 255\n");
   robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");
   robot.Send("CLEAR_POSITION_LOG\n");
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Maps an angle in radians into a an equivalent angle understood by the robot (-PI <= ang <= +PI)
// ARGUMENTS:    ang: the angle in radians 
// RETURN VALUE: the mapped angle in radians
double mapAngle(double angRad)
{
   angRad = fmod(angRad, 2.0 * PI);  // put in range -2*PI <= ang <= +2*PI

   // map into range -PI <= ang <= +PI
   if (angRad > PI)
      angRad -= 2.0 * PI;
   else if (angRad < -PI)
      angRad += 2.0 * PI;

   return angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in degrees from input angle in radian
// ARGUMENTS:    angDeg:  angle in degrees
// RETURN VALUE: angle in radians
double degToRad(double angDeg)
{
   return (PI / 180.0) * angDeg;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Returns angle in radians from input angle in degrees
// ARGUMENTS:    angRad:  angle in radians
// RETURN VALUE: angle in degrees
double radToDeg(double angRad)
{
   return (180.0 / PI) * angRad;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  This function flushes the input buffer to avoid scanf issues
// ARGUMENTS:    none
// RETURN VALUE: false if nothing or only '\n' in stdin. true if extra keystrokes precede the '\n'.
//               Good for detecting left over garbage from scanf_s in the input buffer
bool flushInputBuffer()
{
   int ch; // temp character variable
   bool bHasGarbage = false;

   // exit loop when all characters are flushed
   while ((ch = getchar()) != '\n' && ch != EOF)
   {
      if (!bHasGarbage) bHasGarbage = true;
   }
   return bHasGarbage;
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Waits for user to press enter.  flushes stdin if keystrokes precede enter
// ARGUMENTS:    none
// RETURN VALUE: none
void waitForEnterKey()
{
   unsigned char ch;
   if ((ch = (unsigned char)getchar()) != EOF && ch != '\n') flushInputBuffer();
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  computes nearest integer to given double
// ARGUMENTS:    d: double value
// RETURN VALUE: nearest int
int nint(double d)
{
   return (int)floor(d + 0.5);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  print a solid line to the console
// ARGUMENTS:    length of line in characters
// RETURN VALUE: none
void printHLine(int N)
{
   int n;

   for (n = 0; n < N; n++)
   {
      // can't use dsprintf because characters are different because code pages are different
      // console = code page 437, file = code page 1252
      printf("%c", HL);
      fprintf(flog, "%c", FHL);
   }
   dsprintf("\n");
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  makes a string all upper case characters
// ARGUMENTS:    str:  the string memory address
// RETURN VALUE: none
void makeStringUpperCase(char* str)
{
   if (str == NULL) return; // safety!

   for (size_t i = 0; i < strlen(str); i++) str[i] = (char)toupper(str[i]);
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  prints to both a file and to the console
// ARGUMENTS:    f:  the file handle
//               fmt, ...: for variable number of parameters
// RETURN VALUE: the number of characters printed
int dsprintf(char const* fmt, ...)
{
   va_list args;
   int n1 = -1, n2 = -1;

   if (flog != NULL)
   {
      va_start(args, fmt);
      n1 = vfprintf(flog, fmt, args);
      va_end(args);
   }
   va_start(args, fmt);
   n2 = vfprintf(stdout, fmt, args);
   va_end(args);

   if (n2 < n1) n1 = n2;
   return n1;
}
//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION:  Gets the number of points to check/draw a path based on the length of the path and a resolution value
// ARGUMENTS:    len:  The arc length of the path
//               resolution:  a parameter that determines the density of points on a path
// RETURN VALUE: the number of points
size_t getNumPathPoints(double len, int resolution)
{
   size_t NP;  // number of points used to check/draw the path points

   if (resolution == RESOLUTION_LOW)
      NP = (size_t)nint((len / 500.0) * (double)LOW_RESOLUTION_POINTS_PER_500_UNITS);
   else if (resolution == RESOLUTION_MEDIUM)
      NP = (size_t)nint((len / 500.0) * (double)MEDIUM_RESOLUTION_POINTS_PER_500_UNITS);
   else
      NP = (size_t)nint((len / 500.0) * (double)HIGH_RESOLUTION_POINTS_PER_500_UNITS);

   NP = max(NP, 2);  //safety

   return NP;
}

bool setPenColor(char* strLine)
{
   char cmd[COMMAND_STRING_ARRAY_SIZE];

   sprintf_s(cmd, COMMAND_STRING_ARRAY_SIZE, "%s\n", strLine);
   robot.Send(cmd);

   return true;

}

bool setRotateJoint(char* strLine)
{
   char *tok = NULL, *nextTok = NULL;
   char *pGarbage;
   char cmd[COMMAND_STRING_ARRAY_SIZE];

   tok = strtok_s(strLine, seps, &nextTok);
   
   TOOL_POSITION toolPos = {strtod(strtok_s(NULL, seps, &nextTok), &pGarbage), strtod(strtok_s(NULL, seps, &nextTok), &pGarbage)};

   INVERSE_SOLUTION isol = inverseKinematics(toolPos);

   if(isol.bCanReach[LEFT] || isol.bCanReach[RIGHT])
   {
      sprintf_s(cmd, COMMAND_STRING_ARRAY_SIZE, "%s\n", strLine);
      robot.Send(cmd);
      return true;
   }
   else
   {
      return false;
   }

}

bool setMoveTo(char *strLine)
{
   char *tok = NULL, *nextTok = NULL;
   char *pGarbage;
   char cmd[COMMAND_STRING_ARRAY_SIZE];
   tok = strtok_s(strLine, seps, &nextTok);


}

bool setMotorSpeed(char *strLine)
{
   char cmd[COMMAND_STRING_ARRAY_SIZE];

   sprintf_s(cmd, COMMAND_STRING_ARRAY_SIZE, "%s\n", strLine);
   robot.Send(cmd);

   return true;
}

bool setLine(char *strLine)
{



   return true;
}


INVERSE_SOLUTION inverseKinematics(TOOL_POSITION toolPos)
{
   INVERSE_SOLUTION isol = { ERROR_VALUE, ERROR_VALUE, ERROR_VALUE, ERROR_VALUE, false, false };

   double distance = 0.0;
   double beta = 0.0;
   double alpha = 0.0;
   double tanExpression = 0.0;

   double leftTheta1 = 0.0;
   double leftTheta2 = 0.0;
   double leftTheta1Deg = 0.0;
   double leftTheta2Deg = 0.0;

   double rightTheta1 = 0.0;
   double rightTheta2 = 0.0;
   double rightTheta1Deg = 0.0;
   double rightTheta2Deg = 0.0;

   distance = sqrt(toolPos.x * toolPos.x + toolPos.y * toolPos.y);

   // test that distance from robot origin to entered point is within range
   if (distance > LMIN && distance < LMAX)
   {
      beta = atan2(toolPos.y, toolPos.x);
      alpha = acos((L2 * L2 - distance * distance - L1 * L1) / (-2 * distance * L1));

      leftTheta1 = mapAngle(beta + alpha);
      tanExpression = atan2(toolPos.y - L1 * sin(leftTheta1), toolPos.x - L1 * cos(leftTheta1));
      leftTheta2 = mapAngle(tanExpression - leftTheta1);


      leftTheta1Deg = radToDeg(leftTheta1);
      leftTheta2Deg = radToDeg(leftTheta2);

      if (fabs(leftTheta1Deg) < ABS_THETA1_DEG_MAX && fabs(leftTheta2Deg) < ABS_THETA2_DEG_MAX)
      {
         isol.jointAngles[LEFT].theta1Deg = leftTheta1Deg;
         isol.jointAngles[LEFT].theta2Deg = leftTheta2Deg;
         isol.bCanReach[LEFT] = true;

      }

      rightTheta1 = mapAngle(beta - alpha);
      tanExpression = atan2(toolPos.y - L1 * sin(rightTheta1), toolPos.x - L1 * cos(rightTheta1));
      rightTheta2 = mapAngle(tanExpression - rightTheta1);

      rightTheta1Deg = radToDeg(rightTheta1);
      rightTheta2Deg = radToDeg(rightTheta2);

      if (fabs(rightTheta1Deg) < ABS_THETA1_DEG_MAX && fabs(rightTheta2Deg) < ABS_THETA2_DEG_MAX)
      {
         isol.jointAngles[RIGHT].theta1Deg = rightTheta1Deg;
         isol.jointAngles[RIGHT].theta2Deg = rightTheta2Deg;
         isol.bCanReach[RIGHT] = true;

      }
   }
   return isol;

}
