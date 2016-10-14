#include "Config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
Configuration configuration;
#define MAX_PATH 200
Configuration* Global(){
   return &configuration;
}

void Configuration::LoadConfiguration(char* file){
	FILE* f = fopen(file,"r");
	if(f==NULL){
	   printf("Can not find the configuration file\n");
	   exit(0);
	}
	char parameter[MAX_PATH];
	char value[MAX_PATH];
	while(!feof(f)){
	  fscanf(f,"%s %s",&parameter,&value);
	  if(!strcmp(parameter,"number_frame")){
		  number_frame = atoi(value);
	  }
	}
}