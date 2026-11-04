#pragma once
#include <QString>

/*
* The gantry log: everything X / Y / Z do, in its own file next to the main log - one file an hour,
* <Log_Folder>gantry_YYYYMMDD_HH.log, the same hours as the main log's log_YYYYMMDD_HH.log (a day in
* production would be tens of MB in one file). One line per event: what was asked (absolute, relative, velocity
* jog, home, stop, servo, counter re-sync), the target in mm and pulses, where the axis was (command
* and encoder counters) when it was sent, the speed it went at, when the card said done and where it
* stood then, and every settle verdict (on target / re-issue / given up) with its error.
*
* Motion_APS writes the card-level lines, so every caller is covered; the callers that know WHY a
* move happens (a jog type, a Guided Setup action) wrap it in a Scope, whose text goes in brackets on
* every line written meanwhile on that thread. The thread id is the main log's [T:...] for matching.
* Read-only position polling is never logged.
*/
namespace gantrylog {

void init(const QString& folder);                 //the main log's Log_Folder; until called, "log/"
void write(const char* fmt, ...);                 //one line, time + thread + scope prepended
const char* axisName(int axis);                   //"X" / "Y" / "Z" / "A3"...

//names the reason for the moves made on this thread while it lives; scopes nest ("GS2 Teach / jog 2D")
class Scope {
public:
	explicit Scope(const QString& what);
	~Scope();
	Scope(const Scope&) = delete;
	Scope& operator=(const Scope&) = delete;
private:
	int m_prevLen;
};

}
