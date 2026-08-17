#ifndef OPENWORKSPACEINFO_H
#define OPENWORKSPACEINFO_H

class OpenWorkspaceInfo
{
public:
	OpenWorkspaceInfo();
	~OpenWorkspaceInfo();

	char _workspacePath[1024] = {0};
	char _workspaceName[1024] = { 0 };
};
#endif // OPENWORKSPACEINFO_H