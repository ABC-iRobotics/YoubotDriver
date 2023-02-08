#ifndef LOGGER_HPP
#define	LOGGER_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

enum severity_level {
  everything = 0,
  trace = 1,
  debug = 2,
  info = 3,
  warning = 4,
  error = 5,
  fatal = 6,
  nothing = 7 
};
    
///////////////////////////////////////////////////////////////////////////////
/// Implementation logging to console and to a file
///////////////////////////////////////////////////////////////////////////////
class Logger {
private:
    std::stringstream out;
    bool print;
    severity_level level;
public:

    Logger(const std::string &funcName, const int &lineNo, const std::string &fileName, severity_level level);
    ~Logger();
        
    static bool toConsole;
    static bool toFile;
    static bool toROS;
    static severity_level logginLevel;

    template <class T>
    Logger & operator<<(const T &v) {
        out << v;
        return *this;
    }
};

#define LOG(level) Logger(__PRETTY_FUNCTION__, __LINE__ , __FILE__, level)

#endif

