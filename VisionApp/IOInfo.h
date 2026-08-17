#pragma once
#include <string>

class IOInfo
{
public:
	IOInfo();
	~IOInfo();

	std::string name;
	int port;
	int bit;
};