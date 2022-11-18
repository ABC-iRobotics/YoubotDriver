#ifndef LOGGER_HPP
#define	LOGGER_HPP

#include <map>
#include <string>

namespace Log {

  enum LogLevel {
	trace = 0,
	debug = 1,
	info = 2,
	warning = 3,
	error = 4,
	fatal = 5,
	off = 6,
	n_levels = 7
  };

  void Setup(const std::map<std::string, std::string>& settings);

  void ConsoleSetup(LogLevel print_over);

  void FileSetup(LogLevel print_over);

  void DropLogger();
};

void log(const std::string& funcName, const int& lineNo,
  const std::string& fileName, Log::LogLevel level, const std::string& message);

void log(const std::string& funcName, Log::LogLevel level, const std::string& message);

void log(Log::LogLevel level, const std::string& message);

#endif

