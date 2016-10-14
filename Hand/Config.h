#pragma once

class Configuration{

public:
	Configuration(){
	
	}

	~Configuration(){
	
	}

	void LoadConfiguration(char* file);

public:
	int number_frame;




};

Configuration* Global();