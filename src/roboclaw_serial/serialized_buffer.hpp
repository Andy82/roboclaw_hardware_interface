// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <string>

namespace roboclaw_serial
{
template<std::size_t Size>
class SerializedBuffer
{
public:
  using value_type = std::byte;
  using iterator = std::byte *;

  SerializedBuffer() = default;
  ~SerializedBuffer() = default;

  inline auto data() {return data_.data() + start_idx_;}

  inline auto begin() {return data_.begin() + start_idx_;}
  inline auto end() {return data_.begin() + end_idx_;}

  inline auto begin() const {return data_.cbegin() + start_idx_;}
  inline auto end() const {return data_.cbegin() + end_idx_;}

  inline auto size() const {return end_idx_ - start_idx_;}

  constexpr inline auto max_size() const {return Size;}

  inline auto resize(const std::size_t size)
  {
    start_idx_ = 0;
    end_idx_ = size;
  }

  inline virtual void clear()
  {
    start_idx_ = 0;
    end_idx_ = 0;
  }

  inline constexpr value_type & operator[](size_t pos) {return data_[start_idx_ + pos];}

  inline constexpr const value_type & operator[](size_t pos) const
  {
    return data_[start_idx_ + pos];
  }

  template<typename T>
  inline void push_back(const T & arg)
  {
    // static_assert(std::is_arithmetic<T>::value, "The provided type is not an arithmetic type.");
    static_assert(
      std::is_arithmetic<T>::value, "SerializedBuffer::push_back only supports arithmetic types.");

    // cast the arg to a bytes array
    auto data_ptr = reinterpret_cast<const std::byte *>(&arg);

    // Add the data to the buffer
    for (size_t i = 0; i < sizeof(T); ++i) {
      // Reverse the byte ordering to store the value in big-endian
      this->push_back(data_ptr[sizeof(T) - 1 - i]);
    }
  }

  template<typename ... Args>
  inline void unpack(Args &... values)
  {
    // Use a fold expression to call pop_front() for each value
    ((values = this->pop_front<std::remove_reference_t<Args>>()), ...);
  }

  template<typename T>
  inline T pop_back()
  {
    // Ensure that the range of valid data greater than or equal to zero
    T value;

    // Reinterpret the output type as a byte array
    std::byte * data_ptr = reinterpret_cast<std::byte *>(&value);

    // Deserialize each byte into a big-endian array
    for (std::size_t i = 0; i < sizeof(T); ++i) {
      data_ptr[i] = this->pop_back();
    }

    return value;
  }

  // Use SFINAE to ensure this function gets called for non-strings
  template<typename T>
  inline typename std::enable_if<!std::is_same<T, std::string>::value, T>::type pop_front()
  {
    T value;

    // Reinterpret the output type as a byte array
    std::byte * data_ptr = reinterpret_cast<std::byte *>(&value);

    // Deserialize each byte into a big-endian array
    for (size_t i = 0; i < sizeof(T); ++i) {
      data_ptr[sizeof(T) - 1 - i] = this->pop_front();
    }

    return value;
  }

  // Use SFINAE to ensure that this function gets called for strings
  template<typename T>
  inline typename std::enable_if<std::is_same<T, std::string>::value, std::string>::type pop_front()
  {
    // The string will be the size of the entire buffer
    std::string result;
    result.reserve(this->size());

    // Loop backwords
    while (this->size()) {
      result.push_back(this->pop_front<char>());
    }

    return result;
  }

  // Push a single byte into the buffer
  inline void push_back(std::byte byte)
  {
    // Allocate an extra byte to store the value
    this->expand(sizeof(std::byte));

    // Copy the byte into the data array
    data_[end_idx_ - 1] = byte;
  }

  inline std::byte pop_front()
  {
    if (this->size() < sizeof(std::byte)) {
      throw std::underflow_error("Underflow error front!");
    }

    std::byte value = data_[start_idx_];
    start_idx_ += sizeof(std::byte);

    return value;
  }

  inline std::byte pop_back()
  {
    if (this->size() < sizeof(std::byte)) {
      throw std::underflow_error("Underflow error back!");
    }

    std::byte value = data_[end_idx_ - 1];
    end_idx_ -= sizeof(std::byte);

    return value;
  }

private:
  // Expand the size of the buffer by num_bytes
  inline void expand(const std::size_t num_bytes)
  {
    if (this->size() + num_bytes > Size) {
      throw std::overflow_error("Tried to allocate more values than buffer can hold");
    }

    // Expand the buffer
    end_idx_ += num_bytes;
  }

  std::array<value_type, Size> data_;

  std::size_t start_idx_ = 0;
  std::size_t end_idx_ = 0;
};
}  // namespace roboclaw_serial
