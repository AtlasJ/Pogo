#ifndef ERRORINFO_H
#define ERRORINFO_H

class ErrorInfo
{
public:
	ErrorInfo();
	~ErrorInfo();

	bool _isRunFail = false;
	char _error[1024] = {0};
};
#endif // ERRORINFO_H