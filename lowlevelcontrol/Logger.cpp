#include"spdlog/spdlog.h"
#include"spdlog/async.h"
#include"spdlog/sinks/basic_file_sink.h"
#include"spdlog/details/os.h"
#include"spdlog/details/fmt_helper.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include <spdlog/fmt/ostr.h>
#include "Logger.hpp"

std::string toString(Log::LogLevel level) {
  switch (level)
  {
  case Log::trace:
    return "trace";
  case Log::debug:
    return "debug";
  case Log::info:
    return "info";
  case Log::warning:
    return "warning";
  case Log::error:
    return "error";
  case Log::fatal:
    return "fatal";
  }
  return "nothing";
};

std::shared_ptr<spdlog::logger> spd_logger = NULL;
std::shared_ptr<spdlog::sinks::sink> stdout_sink = NULL, file_sink = NULL;

Log::LogLevel fromString(const std::string& name) {
  if (name == "trace")
    return Log::trace;
  if (name == "debug")
    return Log::debug;
  if (name == "info")
    return Log::info;
  if (name == "warning")
    return Log::warning;
  if (name == "error")
    return Log::error;
  if (name == "fatal")
    return Log::fatal;
  if (name == "off")
    return Log::off;
  return Log::n_levels;
}

void Log::Setup(const std::map<std::string, std::string>& settings) {
  std::string filename;
  if (settings.find("LogFileName") != settings.end())
    filename = settings.at("LogFileName");
  else
    filename = "log_output_" + std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count()) + ".log";
  spdlog::init_thread_pool(8192, 1);
  stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt >();
  file_sink = std::make_shared < spdlog::sinks::basic_file_sink_mt>(filename.c_str());
  std::vector<spdlog::sink_ptr> sinks{ stdout_sink, file_sink };
  spd_logger = std::make_shared<spdlog::async_logger>("youbot", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
  spdlog::register_logger(spd_logger);
  if (settings.find("ConsoleLevel") != settings.end())
    Log::ConsoleSetup(fromString(settings.at("ConsoleLevel")));
  else
    Log::ConsoleSetup(Log::trace);
  if (settings.find("LogFileLevel") != settings.end())
    Log::FileSetup(fromString(settings.at("LogFileLevel")));
  else
    Log::FileSetup(Log::trace);
}

void Log::ConsoleSetup(Log::LogLevel print_over) {
  if (stdout_sink)
    stdout_sink->set_level(spdlog::level::level_enum(print_over));
}

void Log::FileSetup(Log::LogLevel print_over) {
  if (file_sink)
    file_sink->set_level(spdlog::level::level_enum(print_over));
}

void Log::DropLogger() {
  spdlog::drop_all();
  stdout_sink = NULL;
  file_sink = NULL;
  spd_logger = NULL;
}

void log(const std::string& funcName, const int& lineNo,
  const std::string& fileName, Log::LogLevel level, const std::string& message) {
  if (spd_logger)
    spd_logger->log(spdlog::level::level_enum(level), "'{}' sent by '{}' at line {} of {}", message, funcName, lineNo, fileName);
}

void log(const std::string& funcName, Log::LogLevel level, const std::string& message) {
  if (spd_logger)
    spd_logger->log(spdlog::level::level_enum(level), "'{}' sent by '{}'}", message, funcName);
}

void log(Log::LogLevel level, const std::string& message) {
  if (spd_logger)
    spd_logger->log(spdlog::level::level_enum(level), "'{}'", message);
}
