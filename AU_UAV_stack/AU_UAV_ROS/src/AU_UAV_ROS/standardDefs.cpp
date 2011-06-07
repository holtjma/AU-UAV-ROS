
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "AU_UAV_ROS/standardDefs.h"

/*
isBlankLine(...)
simple function for parsing to determine is a string is a "blank" line
*/
bool isBlankLine(char str[])
{
	for(int i = 0; i < strlen(str); i++)
	{
		switch(str[i])
		{
			case ' ':
			case '\n':
			case '\t':
			{
				//keep checking
				break;
			}
			default:
			{
				//not a blank line character
				return false;
				break;
			}
		}
	}
	
	//we made it here, must be blank
	return true;
}

/*
isValidYesNo(...)
returns true if the character is a 'y', 'Y', 'n', or 'N'
*/
bool isValidYesNo(char c)
{
	c = tolower(c);
	if(c == 'y' || c == 'n') return true;
	else return false;
}
