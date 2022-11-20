#ifndef PROCESS_BUFFER_HPP
#define PROCESS_BUFFER_HPP

#include <memory>

namespace youbot {

  class ProcessBuffer {
	uint8_t size;

  public:
	uint8_t* buffer;

	ProcessBuffer();

	ProcessBuffer(uint8_t size);

	ProcessBuffer(uint8_t size, uint8_t* buff);

	ProcessBuffer(ProcessBuffer& in);

	ProcessBuffer(const ProcessBuffer& in);

	~ProcessBuffer();

	ProcessBuffer& operator=(const ProcessBuffer& in);

	uint8_t operator[](uint8_t i) const;

	void CopyTo(uint8_t* dest, uint8_t maxSize) const;

	void Print() const;

	int Size() const;
  };
}
#endif
