// https://chromium.googlesource.com/chromium/src/+/master/LICENSE
// Copyright 2015 The Chromium Authors. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//    * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// AlignedMemory is a POD type that gives you a portable way to specify static
// or local stack data of a given alignment and size. For example, if you need
// static storage for a class, but you want manual control over when the object
// is constructed and destructed (you don't want static initialization and
// destruction), use AlignedMemory:
//
//   static AlignedMemory<sizeof(MyClass), ALIGNOF(MyClass)> my_class;
//
//   // ... at runtime:
//   new(my_class.void_data()) MyClass();
//
//   // ... use it:
//   MyClass* mc = my_class.data_as<MyClass>();
//
//   // ... later, to destruct my_class:
//   my_class.data_as<MyClass>()->MyClass::~MyClass();
#ifndef BASE_MEMORY_ALIGNED_MEMORY_H_
#define BASE_MEMORY_ALIGNED_MEMORY_H_

// Specify memory alignment for structs, classes, etc.
// Use like:
//   class ALIGNAS(16) MyClass { ... }
//   ALIGNAS(16) int array[4];
#if defined(_MSC_VER)
#define ALIGNAS(byte_alignment) __declspec(align(byte_alignment))
#elif defined(__GNUC__)
#define ALIGNAS(byte_alignment) __attribute__((aligned(byte_alignment)))
#endif

// Return the byte alignment of the given type (available at compile time).
// Use like:
//   ALIGNOF(int32)  // this would be 4
#if defined(_MSC_VER)
#define ALIGNOF(type) __alignof(type)
#elif defined(__GNUC__)
#define ALIGNOF(type) __alignof__(type)
#endif

// AlignedMemory is specialized for all supported alignments.
// Make sure we get a compiler error if someone uses an unsupported alignment.
template <size_t Size, size_t ByteAlignment>
struct AlignedMemory {};

#define BASE_DECL_ALIGNED_MEMORY(byte_alignment) \
	template <size_t Size> \
	class AlignedMemory<Size, byte_alignment> { \
		public: \
		ALIGNAS(byte_alignment) unsigned char data_[Size]; \
		void* void_data() { return reinterpret_cast<void*>(data_); } \
		const void* void_data() const { \
		return reinterpret_cast<const void*>(data_); \
		} \
		template<typename Type> \
		Type* data_as() { return reinterpret_cast<Type*>(void_data()); } \
		template<typename Type> \
		const Type* data_as() const { \
		return reinterpret_cast<const Type*>(void_data()); \
		} \
		private: \
		void* operator new(size_t); \
		void operator delete(void*); \
	}

// Specialization for all alignments is required because MSVC (as of VS 2008)
// does not understand ALIGNAS(ALIGNOF(Type)) or ALIGNAS(template_param).
// Greater than 4096 alignment is not supported by some compilers, so 4096 is
// the maximum specified here.
BASE_DECL_ALIGNED_MEMORY(1);
BASE_DECL_ALIGNED_MEMORY(2);
BASE_DECL_ALIGNED_MEMORY(4);
BASE_DECL_ALIGNED_MEMORY(8);
BASE_DECL_ALIGNED_MEMORY(16);
BASE_DECL_ALIGNED_MEMORY(32);
BASE_DECL_ALIGNED_MEMORY(64);
BASE_DECL_ALIGNED_MEMORY(128);
BASE_DECL_ALIGNED_MEMORY(256);
BASE_DECL_ALIGNED_MEMORY(512);
BASE_DECL_ALIGNED_MEMORY(1024);
BASE_DECL_ALIGNED_MEMORY(2048);
BASE_DECL_ALIGNED_MEMORY(4096);

#endif  // BASE_MEMORY_ALIGNED_MEMORY_H_
