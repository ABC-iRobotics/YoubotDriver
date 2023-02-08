#include "Logger.hpp"

bool Logger::toConsole = true;
bool Logger::toFile = false;
bool Logger::toROS = false;
severity_level Logger::logginLevel = info;

Logger::Logger(const std::string &funcName, const int &lineNo, const std::string &fileName, severity_level level) {
    
  this->level = level;
  if (toConsole || toFile) {
    if (level >= logginLevel) {
      print = true;

      switch (level) {
        case trace:
          out << "Trace" << ": ";
          break;
        case debug:
          out << "Debug" << ": ";
          break;
        case info:
          out << "Info" << ": ";
          break;
        case warning:
          out << "Warning" << ": ";
          break;
        case error:
          out << "Error" << ": ";
          break;
        case fatal:
          out << "Fatal" << ": ";
          break;
        default:
          break;
      }
      //  out << "function " << funcName << ": ";
      //  out << "line " << lineNo << ": ";
      //  out << "fileName " << fileName << ": ";
      //  out << "time " << boost::posix_time::microsec_clock::local_time() << ": ";
    } else {
      print = false;
    }
  } else {
    print = false;
  }
}

Logger::~Logger() {
  //end of message
  if (toConsole && print) {
    printf("%s\n", out.str().c_str());
  //  std::cout << out.str() << std::endl;
  }

  if (toFile && print) {
    std::fstream filestr;
    filestr.open("log.txt", std::fstream::out | std::fstream::app);
    filestr << out.str() << std::endl;
    filestr.close();
  }
}
