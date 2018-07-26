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

#ifndef BASE_STACK_CONTAINER_H
#define BASE_STACK_CONTAINER_H

#include <string>
#include <vector>

#include "AlignedMemory.h"

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
	TypeName(const TypeName&);             \
	void operator=(const TypeName&)

// This allocator can be used with STL containers to provide a stack buffer
// from which to allocate memory and overflows onto the heap. This stack buffer
// would be allocated on the stack and allows us to avoid heap operations in
// some situations.
//
// STL likes to make copies of allocators, so the allocator itself can't hold
// the data. Instead, we make the creator responsible for creating a
// StackAllocator::Source which contains the data. Copying the allocator
// merely copies the pointer to this shared source, so all allocators created
// based on our allocator will share the same stack buffer.
//
// This stack buffer implementation is very simple. The first allocation that
// fits in the stack buffer will use the stack buffer. Any subsequent
// allocations will not use the stack buffer, even if there is unused room.
// This makes it appropriate for array-like containers, but the caller should
// be sure to reserve() in the container up to the stack buffer size. Otherwise
// the container will allocate a small array which will "use up" the stack
// buffer.
template<typename T, size_t stack_capacity>
class StackAllocator : public std::allocator<T> 
{
public:
	typedef typename std::allocator<T>::pointer pointer;
	typedef typename std::allocator<T>::size_type size_type;

	// Backing store for the allocator. The container owner is responsible for
	// maintaining this for as long as any containers using this allocator are
	// live.
	struct Source 
	{
		Source() : m_usedStackBuffer(false) {}

		// Casts the buffer in its right type.
		T* getStackBuffer()
		{ 
			return m_stackBuffer.template data_as<T>(); 
		}

		const T* getStackBuffer() const 
		{
			return m_stackBuffer.template data_as<T>();
		}

		// The buffer itself. It is not of type T because we don't want the
		// constructors and destructors to be automatically called. Define a POD
		// buffer of the right size instead.
		AlignedMemory<sizeof(T[stack_capacity]), ALIGNOF(T)> m_stackBuffer;

		// Set when the stack buffer is used for an allocation. We do not track
		// how much of the buffer is used, only that somebody is using it.
		bool m_usedStackBuffer;
	};

	// Used by containers when they want to refer to an allocator of type U.
	template<typename U>
	struct rebind 
	{
		typedef StackAllocator<U, stack_capacity> other;
	};

	// For the straight up copy c-tor, we can share storage.
	StackAllocator(const StackAllocator<T, stack_capacity>& rhs)
		: std::allocator<T>(), m_source(rhs.m_source) 
	{}

	// ISO C++ requires the following constructor to be defined,
	// and std::vector in VC++2008SP1 Release fails with an error
	// in the class _Container_base_aux_alloc_real (from <xutility>)
	// if the constructor does not exist.
	// For this constructor, we cannot share storage; there's
	// no guarantee that the Source buffer of Ts is large enough
	// for Us.
	// TODO: If we were fancy pants, perhaps we could share storage
	// iff sizeof(T) == sizeof(U).
	template<typename U, size_t other_capacity>
	StackAllocator(const StackAllocator<U, other_capacity>& other) : m_source(NULL) 
	{
	}

	explicit StackAllocator(Source* source) : m_source(source) 
	{
	}

	// Actually do the allocation. Use the stack buffer if nobody has used it yet
	// and the size requested fits. Otherwise, fall through to the standard
	// allocator.
	pointer allocate(size_type n, void* hint = 0) 
	{
		if (m_source != NULL && !m_source->m_usedStackBuffer && n <= stack_capacity) 
		{
			m_source->m_usedStackBuffer = true;
			return m_source->getStackBuffer();
		}
		else 
		{
			return std::allocator<T>::allocate(n, hint);
		}
	}

	// Free: when trying to free the stack buffer, just mark it as free. For
	// non-stack-buffer pointers, just fall though to the standard allocator.
	void deallocate(pointer p, size_type n) 
	{
		if (m_source != NULL && p == m_source->getStackBuffer())
		{
			m_source->m_usedStackBuffer = false;
		}
		else
		{
			std::allocator<T>::deallocate(p, n);
		}
	}

private:
	Source* m_source;
};

// A wrapper around STL containers that maintains a stack-sized buffer that the
// initial capacity of the vector is based on. Growing the container beyond the
// stack capacity will transparently overflow onto the heap. The container must
// support reserve().
//
// WATCH OUT: the ContainerType MUST use the proper StackAllocator for this
// type. This object is really intended to be used only internally. You'll want
// to use the wrappers below for different types.
template<typename TContainerType, int stack_capacity>
class StackContainer 
{
public:
	typedef TContainerType ContainerType;
	typedef typename ContainerType::value_type ContainedType;
	typedef StackAllocator<ContainedType, stack_capacity> Allocator;

	// Allocator must be constructed before the container!
	StackContainer() : m_allocator(&m_stack_data), m_container(m_allocator) 
	{
		// Make the container use the stack allocation by reserving our buffer size
		// before doing anything else.
		m_container.reserve(stack_capacity);
	}

	// Getters for the actual container.
	//
	// Danger: any copies of this made using the copy constructor must have
	// shorter lifetimes than the source. The copy will share the same allocator
	// and therefore the same stack buffer as the original. Use std::copy to
	// copy into a "real" container for longer-lived objects.
	ContainerType& container() { return m_container; }
	const ContainerType& container() const { return m_container; }

	// Support operator-> to get to the container. This allows nicer syntax like:
	//   StackContainer<...> foo;
	//   std::sort(foo->begin(), foo->end());
	ContainerType* operator->() { return &m_container; }
	const ContainerType* operator->() const { return &m_container; }

protected:
	typename Allocator::Source m_stack_data;
	Allocator m_allocator;
	ContainerType m_container;

	DISALLOW_COPY_AND_ASSIGN(StackContainer);
};

// StackString
template<size_t stack_capacity>
class StackString : public StackContainer<
    std::basic_string<char,
                      std::char_traits<char>,
                      StackAllocator<char, stack_capacity> >,
    stack_capacity> 
{
public:
	StackString() : StackContainer<
		std::basic_string<char,
						std::char_traits<char>,
						StackAllocator<char, stack_capacity> >,
		stack_capacity>() 
	{
	}

private:
  DISALLOW_COPY_AND_ASSIGN(StackString);
};

// StackWString
template<size_t stack_capacity>
class StackWString : public StackContainer<
    std::basic_string<wchar_t,
                      std::char_traits<wchar_t>,
                      StackAllocator<wchar_t, stack_capacity> >,
    stack_capacity>
{
public:
	StackWString() : StackContainer<
		std::basic_string<wchar_t,
						std::char_traits<wchar_t>,
						StackAllocator<wchar_t, stack_capacity> >,
		stack_capacity>()
	{
	}

private:
  DISALLOW_COPY_AND_ASSIGN(StackWString);
};

// StackVector
//
// Example:
//   StackVector<int, 16> foo;
//   foo->push_back(22);  // we have overloaded operator->
//   foo[0] = 10;         // as well as operator[]
template<typename T, size_t stack_capacity>
class StackVector : public StackContainer<
    std::vector<T, StackAllocator<T, stack_capacity> >,
    stack_capacity> 
{
public:
	StackVector() : StackContainer<
		std::vector<T, StackAllocator<T, stack_capacity> >,
		stack_capacity>()
	{
	}

	// We need to put this in STL containers sometimes, which requires a copy
	// constructor. We can't call the regular copy constructor because that will
	// take the stack buffer from the original. Here, we create an empty object
	// and make a stack buffer of its own.
	StackVector(const StackVector<T, stack_capacity>& other)
		: StackContainer<
			std::vector<T, StackAllocator<T, stack_capacity> >,
			stack_capacity>()
	{
		this->container().assign(other->begin(), other->end());
	}

	StackVector<T, stack_capacity>& operator=(const StackVector<T, stack_capacity>& other) 
	{
		this->container().assign(other->begin(), other->end());
		return *this;
	}

	// Vectors are commonly indexed, which isn't very convenient even with
	// operator-> (using "->at()" does exception stuff we don't want).
	T& operator[](size_t i)
	{
		return this->container().operator[](i); 
	}

	const T& operator[](size_t i) const 
	{
		return this->container().operator[](i);
	}
};

#endif  // BASE_STACK_CONTAINER_H