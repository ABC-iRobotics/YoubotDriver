#ifndef LOGGER_HPP
#define	LOGGER_HPP

#include <map>
#include <string>

namespace Log {

  /// <summary>
  /// Possible importance levels of log messages
  /// </summary>
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

  /// <summary>
  /// Setup the logger
  /// </summary>
  /// <param name="settings"> NamevalueMap (std::map<std::string, std::string>) e.g.:
  ///   {{"ConsoleLevel","info"},{"LogFileLevel","info"}}
  /// </param>
  void Setup(const std::map<std::string, std::string>& settings);

  /// <summary>
  /// Setup the console logging
  /// </summary>
  /// <param name="print_over"> messages on this level or higher will be displayed </param>
  void ConsoleSetup(LogLevel print_over);

  /// <summary>
  /// Setup the file logging
  /// </summary>
  /// <param name="print_over"> messages on this level or higher will be saved </param>
  void FileSetup(LogLevel print_over);

  /// <summary>
  /// Release logger
  /// </summary>
  void DropLogger();
};

void log(const std::string& funcName, const int& lineNo,
  const std::string& fileName, Log::LogLevel level, const std::string& message);

void log(const std::string& funcName, Log::LogLevel level, const std::string& message);

void log(Log::LogLevel level, const std::string& message);

#endif

