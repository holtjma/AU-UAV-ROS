/*
automator
This is a program to automatically startup several tests for AU_UAV_ROS evaluator when the
inputs are enumerated in another .txt file.

Compiled with G++.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <string.h>

//normal time + 30 as buffer time
#define SIMULATION_TIME 600
#define BUFFER_TIME 5
#define SLEEP_TIME (SIMULATION_TIME+16)
#define OUTPUT_ADDITION "_test"
#define LENGTH_OF_EXTENSION 8 //".course" has 7 characters

/*
isBlankLine(...)
simple function for parsing to determine is a string is a "blank" line
NOTE: Copied from elsewhere
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

int main()
{
	//get the filename
	char filename[256];
	printf("Enter the file with all the courses in it:");
	scanf("%s", filename);
	
	//open the file
	FILE *fp;
	fp = fopen(filename, "r");
	
	//check for a good open
	if(fp != NULL)
	{
		char buffer[256];
		//while we have something in the file
		while(fgets(buffer, sizeof(buffer), fp))
		{	
			//check for useless lines
			if(buffer[0] == '#' || isBlankLine(buffer))
			{
				//this line is a comment
				continue;
			}
			
			//write our modifications to the hidden file
			FILE *tempfp;
			tempfp = fopen(".hiddenTestInputs", "w");
			if(tempfp != NULL)
			{
				//write our input course file
				fprintf(tempfp, "%s\n", buffer);
				
				std::string myStr = std::string(buffer);
				myStr = myStr.substr(0, myStr.size() - LENGTH_OF_EXTENSION);
				myStr = myStr + OUTPUT_ADDITION;
				
				//write our modified outputs
				fprintf(tempfp, "%s\n", myStr.c_str());
				
				fclose(tempfp);
			}
			else
			{
				printf("ERROR: Bad .hiddenTestInputs open!\n");
				continue;
			}
			
			//fork our process
			int pid;
			pid = fork();
	
			if(pid == 0)
			{
				//child process
				system("roslaunch AU_UAV_ROS evaluation.launch < .hiddenTestInputs");
			}
			else
			{
				//parent waits some time, then kills before starting new one
				sleep(SLEEP_TIME);
				printf("Killing Process ID #%d\n", pid);
				kill(pid, SIGTERM);
				
				//give the SIGINT time to work
				sleep(BUFFER_TIME);
			}
		}
		
		fclose(fp);
	}
	else
	{
		printf("ERROR: Bad file name\n");
	}
	
	return 0;
}
