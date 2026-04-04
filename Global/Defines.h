/*****************************************************************************
*	Name: Defines.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Global common functions and definitions
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
/* TODO: AssertReport to save Logfile */

#ifndef __DEFINES_H__
#define __DEFINES_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <termios.h>
#include <vector>
#include <functional>
#include <thread>
#include <string>
#include <iostream>
#include <sys/timeb.h>
#include <math.h>
#include <string>

#ifndef M_PI
#define M_PI        (double)(3.141592653589793)
#endif

#define TWO_M_PI    (double)(6.283185307179586)

#define	TRUE	1
#define	FALSE	0
#define __MAX_PATH__	1024
#define __MAX_MESSAGE__	2048

#define LOG_RST			"\033[0m"
#define LOG_BLACK		"\033[30m"				/* Black */
#define LOG_RED			"\033[31m"				/* Red */
#define LOG_GREEN		"\033[32m"				/* Green */
#define LOG_YELLOW		"\033[33m"				/* Yellow */
#define LOG_BLUE		"\033[34m"				/* Blue */
#define LOG_MAGENTA		"\033[35m"				/* Magenta */
#define LOG_CYAN		"\033[36m"				/* Cyan */
#define LOG_WHITE		"\033[37m"				/* White */
#define LOG_BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define LOG_BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define LOG_BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define LOG_BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define LOG_BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define LOG_BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define LOG_BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define LOG_BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define TOSTRING  std::to_string

#define ZeroMemory(x)   memset(x, 0, sizeof(x))
#define ZeroMemoryE(x, y) memset(x, 0, y)



typedef void* PVOID;
typedef std::thread STD_THREAD, * PSTD_THREAD;
typedef std::function<void(PVOID, PVOID, PVOID, PVOID)> GLOBAL_CALLBACK_FN;
typedef std::string TSTRING;
typedef char TCHAR;

typedef	uint8_t		UINT8;
typedef	uint16_t	UINT16;
typedef	uint32_t	UINT32, UINT;
typedef	uint64_t	UINT64;
typedef	int8_t		INT8;
typedef	int16_t		INT16;
typedef	int32_t		INT32, INT;
typedef	int64_t		INT64;
typedef bool        BOOL;

typedef unsigned char       BYTE;
typedef std::vector<BYTE>   BYTEARRAY;

typedef enum _eLOG_TYPE
{
    eLogTrace = 0,
    eLogWarning,
    eLogError,
    eLogInfo,
    eLogNothing,
} eLOG_TYPE;


inline int getch();
inline int getche();
inline int GetLastError();
inline char* GetOSTime();
inline UINT64 GetCurrTimeInNs();
inline void AssertReport( const char* asOutput );
static inline void OutputString(const char* aszFileName, int anLineNo, eLOG_TYPE aeType, const char* alpszFormat, ...);
static inline void AssertOutputString( const char* aszFileName, int aLineNum, BOOL abUseExtend, BOOL abSave, BOOL abShowTime, const char* alpszFormat, ... );
static inline void OutputDebugString( const char* asOutput, BOOL abIsError=FALSE);


#ifndef __FILENAME__
#define __FILENAME__					(strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif 



#define DBG_LOG_TRACE(fmt, ...)			        OutputString(__FILENAME__, __LINE__, eLogTrace, (char*) fmt, ##__VA_ARGS__)
#define DBG_LOG_ERROR(fmt, ...)			        OutputString(__FILENAME__, __LINE__, eLogError, (char*) fmt,  ##__VA_ARGS__)
#define DBG_LOG_WARN(fmt, ...)				    OutputString(__FILENAME__, __LINE__, eLogWarning, (char*)fmt,  ##__VA_ARGS__)
#define DBG_LOG_INFO(fmt, ...)				    OutputString(__FILENAME__, __LINE__, eLogInfo, (char*)fmt,  ##__VA_ARGS__)
#define DBG_LOG_NOTHING(fmt, ...)				OutputString(__FILENAME__, __LINE__, eLogNothing, (char*)fmt,  ##__VA_ARGS__)

static inline void OutputDebugString( const char* asOutput, BOOL abIsError)
{
    FILE* stream = stderr;
    if (abIsError == FALSE)
        stream = stdout;

    fprintf(stream, "%s\n", asOutput);
}

inline char* GetOSTime()
{
    struct tm stTm;


    struct timeb stTb;
    ftime(&stTb);
    localtime_r(&stTb.time, &stTm);

    /* ftime is deprecated, use clock_gettime instead*/    
    // struct timespec stTspec;
    // clock_gettime(CLOCK_REALTIME, &stTspec); // use time-of-the-day guessed by the OS w.r.t. timezone
    // localtime_r((time_t  *)&stTspec.tv_sec, &stTm);

    char* szDateTime;
    szDateTime = (char*)malloc(sizeof(char)*200);
    
    sprintf(szDateTime, "%d-%02d-%02d_%02d:%02d:%02d.%02d", stTm.tm_year + 1900, stTm.tm_mon + 1, stTm.tm_mday, stTm.tm_hour, stTm.tm_min, stTm.tm_sec, stTb.millitm);
    
    return szDateTime;
}

inline UINT64 GetCurrTimeInNs()
{
    struct timespec	stNow;
    if (clock_gettime(CLOCK_MONOTONIC, &stNow))
    {
        return -1;
    }
    else
    {
        UINT64 ullNanoSeconds = (UINT64)stNow.tv_sec * 1000000000;
        ullNanoSeconds += (UINT64)stNow.tv_nsec;
        return ullNanoSeconds;
    }
}

inline void AssertReport( const char* asOutput )
{

}

template <class A=int>
class CThrowAssert  
{
	const char*	m_pFileName;
	long		m_nLine;
	
public:
	CThrowAssert( const char* apFileName, long anLine ) : m_pFileName( apFileName), m_nLine(anLine) { ; }
	
	inline bool Report()
	{	
		char	szMsg[__MAX_MESSAGE__] = "";
		sprintf( szMsg, "<%s> [ASSERT in file:%s line:%ld]\n", GetOSTime(), m_pFileName, m_nLine );
        OutputDebugString( szMsg );
		// AssertReport( szMsg );
		
		return true;
	}
	
};

static inline void OutputString( const char* aszFileName, int anLineNo, eLOG_TYPE aeType, const char* alpszFormat, ... )
{
    va_list args;
	va_start(args, alpszFormat);

    char	szMsg	[__MAX_MESSAGE__] = "";
	char	szOutput[__MAX_MESSAGE__] = "";
    BOOL    bError = FALSE;

    try
    {
        vsnprintf(szMsg, sizeof(szMsg), alpszFormat, args);
        va_end(args);

        switch (aeType)
        {
        case eLogTrace:
            sprintf(szOutput, LOG_RST "<%-10s>[TRACE] %s [%s:%d]" LOG_RST, GetOSTime(), szMsg, aszFileName, anLineNo);
            break;
        case eLogWarning:
            sprintf(szOutput, LOG_BOLDYELLOW "<%-10s>[WARN] %s [%s:%d]" LOG_RST, GetOSTime(), szMsg, aszFileName, anLineNo);
            break;
        case eLogError:
            sprintf(szOutput, LOG_BOLDRED "<%-10s>[ERROR] %s [%s:%d]" LOG_RST, GetOSTime(), szMsg, aszFileName, anLineNo);
            bError = TRUE;
            break;
        case eLogInfo:
            sprintf(szOutput, LOG_BOLDGREEN "<%-10s>[INFO] %s [%s:%d]" LOG_RST, GetOSTime(), szMsg, aszFileName, anLineNo);
            break;
        case eLogNothing:
            sprintf(szOutput, "<%s> %s", GetOSTime(), szMsg);
            break;
        default:
            break;
        }
    }
    catch ( CThrowAssert <> a )
    {
        a.Report();
        return;
    }
    catch(...)
    {
        DBG_LOG_ERROR("%s - Unknown exception occurred.", "OutputString");
        return;
    }
    OutputDebugString(szOutput, bError);
}

static inline void AssertOutputString( const char* aszFileName, int aLineNum, BOOL abUseExtend, BOOL abSave, BOOL abShowTime, const char* alpszFormat, ... )
{
    va_list args;
    va_start(args, alpszFormat);

    char	szMsg	[__MAX_MESSAGE__] = "";
    char	szOutput[__MAX_MESSAGE__] = "";


    try
    {
        vsnprintf(szMsg, sizeof(szMsg), alpszFormat, args);
        va_end(args);

        if ( strlen( szMsg ) > 0)
        {
            if ( abUseExtend )
                sprintf( szOutput, "<%s> %s [ERROR in file:%s line:%d]\n", GetOSTime(), szMsg, aszFileName, aLineNum );
            else
                sprintf( szOutput, "<%s> %s\n", GetOSTime(), szMsg );
        }
        else
        {
            if ( abUseExtend )
				sprintf( szOutput, "<%s> [ERROR in file:%s line:%d]\n", GetOSTime(), aszFileName, aLineNum );
			else
				sprintf( szOutput, "<%s>\n", GetOSTime() );
        }
    }
    catch ( CThrowAssert <> a )
    {
        a.Report();
        return;
    }
    catch(...)
    {
        DBG_LOG_ERROR("%s - Unknown exception occurred.","AssertOutputString");
        return;
    }
    OutputDebugString( szOutput, TRUE );
}

inline double ConvertRad2Deg(double adRad)
{
    return adRad * 180. / M_PI;
}

inline double ConvertDeg2Rad(double adDeg)
{
    return adDeg * M_PI / 180.;
}

inline double ConvertRpm2Rad(double adRpm)
{
    return adRpm * M_PI / 30.;
}

inline int getch()
{
	struct termios oldt, newt;
	int ch;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~ (ICANON | ECHO);
	tcsetattr (STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

inline int getche()
{
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~ (ICANON);
	tcsetattr (STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
}

inline int 
GetLastError()
{
    return (int)errno;
}

inline TSTRING 
TranslateError(int anErrNo)
{
    return TSTRING(strerror(anErrNo));
}

static INT64 CalcTwosComplement(INT64 anValue)
{
    return ~(-anValue) + 1;
}

/* BIG ENDIAN */
inline BYTEARRAY 
ConvertS64ToByteArrayBE(INT64 anValue)
{
    INT64 nVal = anValue;
    
    if (nVal > 9223372036854775807) nVal = 9223372036854775807;
    if (nVal < -9223372036854775807LL - 1) nVal = -9223372036854775807LL - 1; // avoid long integer warning 

    if (nVal < 0) nVal = CalcTwosComplement(nVal);

    BYTEARRAY vRet{};

    vRet.push_back((BYTE)((nVal >> 56) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 48) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 40) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 32) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 24) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 16) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 8) & 0xFF));
    vRet.push_back((BYTE)(nVal & 0xFF));

    return vRet;
}


inline BYTEARRAY 
ConvertS32ToByteArrayBE(INT32 anValue)
{
    INT32 nVal = anValue;

    if (nVal > 2147483647) nVal = 2147483647;
    if (nVal < -2147483648) nVal = -2147483648;


    if (nVal < 0) nVal = (INT32)CalcTwosComplement((INT64)nVal);


    BYTEARRAY vRet{};

    vRet.push_back((BYTE)((nVal >> 24) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 16) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 8) & 0xFF));
    vRet.push_back((BYTE)(nVal & 0xFF));
    
    return vRet;
}

inline BYTEARRAY 
ConvertS16ToByteArrayBE(INT16 anValue)
{
    INT16 nVal = anValue;

    if (nVal > 32767) nVal = 32767;
    if (nVal < -32768) nVal = -32768;

    if (nVal < 0) nVal = (INT16)CalcTwosComplement((INT64)nVal);

    BYTEARRAY vRet{};

    vRet.push_back((BYTE)((nVal >> 8) & 0xFF));
    vRet.push_back((BYTE)(nVal & 0xFF));

    return vRet;
}

inline BYTEARRAY 
ConvertS8ToByteArrayBE(INT8 anValue)
{
    INT8 nVal = anValue;

    if (nVal > 127) nVal = 127;
    if (nVal < -128) nVal = -128;

    if (nVal < 0) nVal = (INT8)CalcTwosComplement((INT64)nVal);

    BYTEARRAY vRet{};

    vRet.push_back((BYTE)(nVal & 0xFF));

    return vRet;
}

inline BYTEARRAY 
ConvertU64ToByteArrayBE(UINT64 anValue)
{
    UINT64 nVal = anValue;

    if (nVal > 0xFFFFFFFFFFFFFFFF) nVal = 0xFFFFFFFFFFFFFFFF;

    BYTEARRAY vRet{};

    vRet.push_back((BYTE)((nVal >> 56) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 48) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 40) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 32) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 24) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 16) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 8) & 0xFF));
    vRet.push_back((BYTE)(nVal & 0xFF));

    return vRet;
}

inline BYTEARRAY 
ConvertU32ToByteArrayBE(UINT32 anValue)
{
    UINT32 nVal = anValue;

    if (nVal > 0xFFFFFFFF) nVal = 0xFFFFFFFF;

    BYTEARRAY vRet{};

    vRet.push_back((BYTE)((nVal >> 24) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 16) & 0xFF));
    vRet.push_back((BYTE)((nVal >> 8) & 0xFF));
    vRet.push_back((BYTE)(nVal & 0xFF));

    return vRet;
}

inline BYTEARRAY 
ConvertU16ToByteArrayBE(UINT16 anValue)
{
    UINT16 nVal = anValue;

    if (nVal > 0xFFFF) nVal = 0xFFFF;

    BYTEARRAY vRet{};

    vRet.push_back((BYTE)((nVal >> 8) & 0xFF));
    vRet.push_back((BYTE)(nVal & 0xFF));

    return vRet;

}

inline BYTEARRAY 
ConvertU8ToByteArrayBE(UINT8 anValue)
{
    UINT8 nVal = anValue;

    if (nVal > 0xFF) nVal = 0xFF;

    BYTEARRAY vRet{};

    vRet.push_back((BYTE)(nVal & 0xFF));

    return vRet;
}

inline BYTE
ConvertU8ToByte(UINT8 anValue)
{
    UINT8 nVal = anValue;
    if (nVal > 0xFF) nVal = 0xFF;

    return (BYTE)nVal;
}

inline BYTE
ConvertS8ToByte(INT8 anValue)
{
    INT8 nVal = anValue;
    if (nVal > 127) nVal = 127;
    if (nVal < -128) nVal = -128;

    if (nVal < 0) nVal = (INT8)CalcTwosComplement((INT64)nVal);

    return (BYTE)nVal;
}


inline UINT 
ConvertByteArrayToUintBE(BYTEARRAY apData)
{
    UINT nValue = 0;
    for (int nCnt = 0; nCnt < (INT)apData.size(); nCnt++)
    {
        nValue <<= 8;
        nValue |= (INT)apData[nCnt] & 0xFF;
    }
    return nValue;
}

inline INT 
ConvertByteArrayToIntBE(BYTEARRAY apData)
{
    UINT nValue = ConvertByteArrayToUintBE(apData);
    double dMax = (pow(2., 8 * apData.size()));
    UINT nUpperLimit = (UINT)(dMax / 2) - 1;
    INT nRet = (INT)nValue;
    if (nValue > nUpperLimit)
    {
        nRet = (INT)(dMax - nValue) * -1;
    }
    return nRet;
}

inline void 
ExtendByteArray(BYTEARRAY& apOrigin, BYTEARRAY apExtend)
{
    apOrigin.reserve(apOrigin.size() + distance(apExtend.begin(), apExtend.end()));
    apOrigin.insert(apOrigin.end(), apExtend.begin(), apExtend.end());
}


#pragma pack(push, 1)
typedef struct _ST_AXIS_CMD
{
    int nAxisNo;
    int nCmd;
    double dValue;
}ST_AXIS_CMD;
#pragma pack(pop)

typedef std::vector<ST_AXIS_CMD> VEC_AXIS_CMD;


#pragma pack(push, 1)
typedef struct _ST_AXIS_STATE
{
    BYTE btDriveMode;
    BYTE btStatus;
    double dCurPos;
    double dCurVel;
    double dCurAcc;
    double dCurTor;

}ST_AXIS_STATE;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct stRobotMetadata
{
    BOOL	bIsSim;
    BOOL	bWithEcat;
    TSTRING	strName;
    INT32	nAxisNo;
}ST_ROBOT_METADATA;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct stEcatStates
{
    UINT8			btMaster;
    UINT8			btSlave;
    UINT8			btDomain;

    stEcatStates()
    {
        btMaster = 0;
        btSlave = 0;
        btDomain = 0;
    }
} ST_ECAT_STATES;
#pragma pack(pop)

#pragma pack(push, 1)
/* This includes EtherCAT state machine*/
typedef struct stEcatMetadata
{
    ST_ECAT_STATES	stEcatStates;
    UINT16			usSlaveNo;

    stEcatMetadata()
    {
        usSlaveNo = 0;
    }

}ST_ECAT_METADATA;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct stAxisMetadata
{
    TSTRING strName;
    UINT8	btAxisType;
    UINT8	btCommType;
    INT32	nPosLimitU;
    INT32	nPosLimitL;
    INT32	nVelLimitU;
    INT32	nVelLimitL;
    INT32	nAccLimitU;
    INT32	nAccLimitL;
}ST_AXIS_METADATA;
#pragma pack(pop)


#endif // __DEFINES_H__
