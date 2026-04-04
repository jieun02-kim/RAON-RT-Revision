/*****************************************************************************
*	Name: ConfigParser.h
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Header to parse configuration files (.cfg)
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#ifndef __CONFIG_PARSER_H__
#define __CONFIG_PARSER_H__

#include "Defines.h"
#include <sys/stat.h>
#include <sys/types.h>

#include <sys/stat.h>
#include <unistd.h>

extern BOOL IsExistFile(const char*);
extern BOOL IsExistDir(const char*);

extern size_t 	GetPrivateProfileString(const char* section, const char* entry, const char* def, char* buffer, size_t buffer_len, const char* file_name);
extern int 		GetPrivateProfileInt(const char* section, const char* entry, int defval, const char* fname);
extern BOOL 	WritePrivateProfileString(const char* section, const char* entry, const char* def, const char* file_name);

#endif // __CONFIG_PARSER_H__
