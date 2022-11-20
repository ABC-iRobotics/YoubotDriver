#include "ProcessBuffer.hpp"
#include <iostream>

using namespace youbot;

ProcessBuffer::ProcessBuffer() : size(0) {}

ProcessBuffer::ProcessBuffer(uint8_t size) : size(size), buffer(new uint8_t[size]) {}

ProcessBuffer::ProcessBuffer(uint8_t size, uint8_t* buff) : size(size), buffer(new uint8_t[size]) {
  memcpy(buffer, buff, size);
}

ProcessBuffer::ProcessBuffer(ProcessBuffer& in) : size(in.size), buffer(new uint8_t[in.size]) {
  memcpy(buffer, in.buffer, size);
}

ProcessBuffer::ProcessBuffer(const ProcessBuffer& in) : size(in.size), buffer(new uint8_t[in.size]) {
  memcpy(buffer, in.buffer, size);
}

ProcessBuffer::~ProcessBuffer() {
  if (size)
    delete[] buffer;
}

void ProcessBuffer::CopyTo(uint8_t* dest, uint8_t maxSize) const {
  memcpy(dest, buffer, size < maxSize ? size : maxSize);
}

void ProcessBuffer::ProcessBuffer::Print() const {
  std::cout << (int)size << " ";
  for (int i = 0; i < size; i++)
    std::cout << (int)buffer[i] << " ";
  std::cout << std::endl;
}

int ProcessBuffer::Size() const { return size; }

ProcessBuffer& ProcessBuffer::operator=(const ProcessBuffer& in) {
  if (size != in.size) {
    if (size)
      delete buffer;
    size = in.size;
    buffer = new uint8_t[in.size];
  }
  memcpy(buffer, in.buffer, size);
  return *this;
}

uint8_t ProcessBuffer::operator[](uint8_t i) const {
  return buffer[i];
}