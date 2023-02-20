#ifndef DATA_HPP
#define DATA_HPP

#include <chrono>

namespace youbot {
  /// <summary>
  /// Start of the program used as timestamp in uninitialized Data-s
  /// </summary>
  static std::chrono::steady_clock::time_point started_at = std::chrono::steady_clock::now() - std::chrono::minutes(1);

  /// <summary>
  /// Template class for data related to given (known) time point
  /// </summary>
  /// <typeparam name="T"> type of the Data </typeparam>
  template <class T>
  struct Data {
    T value;
    std::chrono::steady_clock::time_point origin;

    /// <summary>
    /// Empty constructor
    /// </summary>
    Data() : value(0), origin(started_at) {}

    /// <summary>
    /// Constructor for a given value using the current timestamp
    /// </summary>
    /// <param name="val"> value to be saved </param>
    Data(T val) : value(val), origin(std::chrono::steady_clock::now()) {}

    /// <summary>
    /// Constructor for a given value using the given timestamp
    /// </summary>
    /// <param name="val"> value to be saved </param>
    /// <param name="origin"> date of origin </param>
    Data(T val, std::chrono::steady_clock::time_point origin) : value(val), origin(origin) {};
  };
}
#endif
