/*****************************************************************************
*	Name: ConfigParser.cpp
*	Author: Raimarius Delgado (Post-Doc)
*	Affiliation: Center for Intelligent & Interactive Robotics - KIST AIRI
*	Description: Implementation of Global Functions to process configuration files (.cfg)
*	Copyright: ROBOGRAM LAB (2022)
*****************************************************************************/
#include "ConfigParser.h"

BOOL 
IsExistFile(const char* strName)
{
	struct stat stInfo;
	if (0 == stat(strName, &stInfo))
		return TRUE;
	
	return FALSE;
}

BOOL
IsExistDir(const char* strName)
{
	struct stat stInfo;
	if (0 == stat(strName, &stInfo) && stInfo.st_mode & S_IFDIR)
		return TRUE;
	
	return FALSE;
}

size_t 
GetPrivateProfileString(const char* section, const char* entry, const char* def, char* buffer, size_t buffer_len, const char* file_name)
{
	char tmpbuf[__MAX_PATH__];
	char tmpsec[__MAX_PATH__];
	char* ep = NULL, * ptr = NULL;
	size_t slen = 0;
	size_t elen = strlen(entry);
	FILE* fp = fopen(file_name, "r");

	if (!fp) return 0;

	memset(tmpsec, 0, __MAX_PATH__);
	sprintf(tmpsec, "[%s]", section);    /* Format the section name */
	slen = strlen(tmpsec);

	/*  Move through file 1 line at a time until a section is matched or EOF */
	do {
		memset(tmpbuf, 0, __MAX_PATH__);
		if (!fgets(tmpbuf, __MAX_PATH__ - 1, fp))
		{
			if (fp) fclose(fp);
			strncpy(buffer, def, buffer_len);
			return strlen(buffer);
		}
	} while (strncasecmp(tmpbuf, tmpsec, slen));

	/* Now that the section has been found, find the entry.
	* Stop searching upon leaving the section's area. */
	do {
		memset(tmpbuf, 0, __MAX_PATH__);
		if (!fgets(tmpbuf, __MAX_PATH__ - 1, fp))
		{
			if (fp)
			{
				fclose(fp);
			}
			strncpy(buffer, def, buffer_len);
			return strlen(buffer);
		}
	} while (strncasecmp(tmpbuf, entry, elen));

	if (fp)
	{
		fclose(fp);
	}

	ep = strrchr(tmpbuf, '=');    /* Parse out the equal sign */
	if (ep == NULL)
	{
		strncpy(buffer, def, buffer_len);
		return strlen(buffer);
	}
	ep++;

	/* remove leading spaces*/
	while (*ep && (isspace(*ep) || *ep == 10)) ep++;

	/* remove trailing spaces*/
	ptr = ep;
	while (*ptr) ptr++; // go to the end, point to a NULL

	ptr--;
	while (ptr > ep) // backup and put in nulls if there is a space
	{
		if (isspace(*ptr) || *ep == 10)
		{
			*ptr = 0;
			ptr--;
		}
		else break;
	}

	/* Copy up to buflen chars to strbuf */
	strncpy(buffer, ep, buffer_len - 1);
	buffer[buffer_len] = '\0';

	return strlen(buffer);
}

int 
GetPrivateProfileInt(const char* section, const char* entry, int defval, const char* fname)
{
	char tmpbuf[__MAX_PATH__];
	char tmpsec[__MAX_PATH__];
	char value[12]; /* the maximum digits of the integer is 10 in decimal */
	char* ep = NULL;
	int i = 0;
	size_t elen = strlen(entry);
	FILE* fp = fopen(fname, "r");

	if (!fp) return 0;

	memset(tmpsec, 0, sizeof(tmpsec));
	sprintf(tmpsec, "[%s]", section); /* string of the section name */
	/* get section */
	do {
		if (!fgets(tmpbuf, __MAX_PATH__ - 1, fp))
		{
			fclose(fp);
			return defval;
		}
	} while (strcmp(tmpbuf, tmpsec));
	/* get entry line */
	do {
		if (!fgets(tmpbuf, __MAX_PATH__ - 1, fp) || tmpbuf[0] == '[')
		{
			fclose(fp);
			return defval;
		}
	} while (strncmp(tmpbuf, entry, elen));

	ep = strrchr(tmpbuf, '=');    /* Parse out the equal sign */
	ep++;
	if (!strlen(ep))          /* No setting? */
		return defval;

	memset(value, 0, sizeof(value));
	for (i = 0; isdigit(ep[i]); i++)
		value[i] = ep[i];
	fclose(fp);

	return atoi(value);
}

BOOL 
WritePrivateProfileString(const char* section, const char* entry, const char* def, const char* file_name)
{
	FILE* rfp, * wfp;
	char tname[15] = "PRIVPROFXXXXXX";
	char tmpbuf[__MAX_PATH__];
	char tmpsec[__MAX_PATH__];
	size_t elen = strlen(entry);
	int tmpfd = mkstemp(tname); /* Get a temporary file name to copy to */

	if (tmpfd == -1)
		return FALSE;
	sprintf(tmpsec, "[%s]", section);
	if (!(rfp = fopen(file_name, "r"))) /* open to read Fail, file NOT exist */
	{
		if (!(wfp = fopen(file_name, "w"))) /* File Open to Write Fail */
		{
			return FALSE;
		}
		/* rfp Fail, Create to Write Ok -> write string to the created file */
		fprintf(wfp, "%s\n", tmpsec);
		fprintf(wfp, "%s=%s\n", entry, def);
		fclose(wfp);
		return TRUE;
	}
	if (!(wfp = fdopen(tmpfd, "w"))) /* open temp file to write */
	{
		fclose(rfp);
		return FALSE;
	}

	/* get section */
	do {
		if (!fgets(tmpbuf, __MAX_PATH__ - 1, rfp))
		{
			/* Reach the EOF of rfp with NO Section Found */
			/* then write string to wfp */
			fprintf(wfp, "\n%s\n", tmpsec);
			fprintf(wfp, "%s=%s\n", entry, def);
			fclose(rfp);
			fclose(wfp);
			unlink(file_name);
			rename(tname, file_name);
			return TRUE;
		}
		/* copy lines to wfp until the section name */
		fprintf(wfp, "%s", tmpbuf);
	} while (strncmp(tmpbuf, tmpsec, strlen(tmpsec)));

	/* get entry */
	while (1)
	{
		if (!fgets(tmpbuf, __MAX_PATH__ - 1, rfp))
		{ /* no entry or error */
			fprintf(wfp, "%s=%s\n", entry, def);
			fclose(rfp);
			fclose(wfp);
			unlink(file_name);
			rename(tname, file_name);
			return TRUE;
		}
		if (!strncmp(tmpbuf, entry, elen) || tmpbuf[0] == '\0')
			break;
		fprintf(wfp, "%s", tmpbuf); /* copy line */
	}

	if (tmpbuf[0] == '\0')
	{ /* no entry, blank line (maybe end of section) */
		fprintf(wfp, "%s=%s\n", entry, def);
		do { /* first write the read null data */
			fprintf(wfp, "%s\n", tmpbuf);
		} while (fgets(tmpbuf, __MAX_PATH__ - 1, rfp));
	}
	else
	{ /* found entry */
		fprintf(wfp, "%s=%s\n", entry, def);
		/* this mean replace the found data and copy remains */
		while (fgets(tmpbuf, __MAX_PATH__ - 1, rfp))
		{
			fprintf(wfp, "%s", tmpbuf);
		}
	}
	fclose(wfp);
	fclose(rfp);
	unlink(file_name);
	rename(tname, file_name);
	return TRUE;
}
