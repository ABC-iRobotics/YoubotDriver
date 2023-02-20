#ifndef PROCESS_BUFFER_HPP
#define PROCESS_BUFFER_HPP

#include <memory>

namespace youbot {

  /// <summary>
  /// Base class used for Process communication
  /// 
  /// Simple buffer to be used for
  /// 
  ///  - setting process messages to be sent
  /// 
  ///  - getting process messages received
  /// </summary>
  class ProcessBuffer {
	uint8_t size;

  public:
	uint8_t* buffer;

	/// <summary>
	/// Constructor with buffer size 0
	/// </summary>
	ProcessBuffer();

	/// <summary>
	/// Constructor with given buffer size
	/// </summary>
	/// <param name="size"> Buffer size</param>
	ProcessBuffer(uint8_t size);

	/// <summary>
	/// Constructor with given buffer size and buffer
	/// </summary>
	/// <param name="size"> Buffer size </param>
	/// <param name="buff"> Buffer content </param>
	ProcessBuffer(uint8_t size, uint8_t* buff);

	/// <summary>
	/// Copy constructor
	/// </summary>
	/// <param name="in"></param>
	ProcessBuffer(ProcessBuffer& in);

	/// <summary>
	/// Copy constructor
	/// </summary>
	/// <param name="in"></param>
	ProcessBuffer(const ProcessBuffer& in);

	/// <summary>
	/// Destructor
	/// </summary>
	~ProcessBuffer();

	/// <summary>
	/// Assign operator to copy the content
	/// </summary>
	/// <param name="in"></param>
	/// <returns></returns>
	ProcessBuffer& operator=(const ProcessBuffer& in);

	/// <summary>
	/// Get the i-th value of the buffer
	/// </summary>
	/// <param name="i"></param>
	/// <returns></returns>
	uint8_t operator[](uint8_t i) const;

	/// <summary>
	/// Copy the content of the buffer to a given destination
	/// </summary>
	/// <param name="dest"> Destination (pointer) </param>
	/// <param name="maxSize"> Max. copied size </param>
	void CopyTo(uint8_t* dest, uint8_t maxSize) const;

	/// <summary>
	/// Log the content of the buffer - for debug purposes
	/// </summary>
	void Print() const;

	/// <summary>
	/// Get the size of the buffer
	/// </summary>
	/// <returns> Buffer size </returns>
	int Size() const;
  };
}
#endif
