#ifndef ATOMIC_PRIMITIVES_H
#define ATOMIC_PRIMITIVES_H

#include "ClientGeometry_CAPI.h"

#include <atomic>
#include <assert.h>

class AtomicUnitVector3f
{
public:
	AtomicUnitVector3f();
	AtomicUnitVector3f(const PSVRVector3f &v);

	void set(const PSVRVector3f &v);
	PSVRVector3f get() const;

private:
	std::atomic_uint64_t m_quantizedUnitVector;
};

class AtomicQuaternionf
{
public:
	AtomicQuaternionf();
	AtomicQuaternionf(const PSVRQuatf &q);

	void set(const PSVRQuatf &v);
	PSVRQuatf get() const;

private:
	std::atomic_uint64_t m_quantizedQuaternion;
};

// Triple buffered lock free atomic generic object
// Inspired by: https://gist.github.com/andrewrk/03c369c82de4701625e3
template<typename t_object_type>
class AtomicObject 
{
public:
    AtomicObject() 
	{
        m_inUseIndex = 0;
        m_activeIndex = 0;
        m_write_index = -1;

		for (int i = 0; i < 3; ++i)
		{
			m_objects[i] = new t_object_type;
		}
    }
    
	virtual ~AtomicObject() 
	{
		for (int i = 0; i < 3; ++i)
		{
			delete m_objects[i];
		}
	}

	void storeValue(const t_object_type &object)
	{
		t_object_type *objectSlot= writeBegin();
		*objectSlot= object;
		writeEnd();
	}

	void fetchValue(t_object_type &out_object)
	{
		out_object= *getReadPtr();
	}

protected:
    t_object_type *writeBegin() 
	{
        assert(m_write_index == -1);
        int in_use_index = m_inUseIndex.load();
        int active_index = m_activeIndex.load();

		if (in_use_index != 0 && active_index != 0)
		{
            m_write_index = 0;
		}
		else if (in_use_index != 1 && active_index != 1)
		{
            m_write_index = 1;
		}
		else
		{
            m_write_index = 2;
		}

        return m_objects[m_write_index];
    }

    void writeEnd() 
	{
        assert(m_write_index != -1);
        m_activeIndex.store(m_write_index);
        m_write_index = -1;
    }

    t_object_type *getReadPtr() 
	{
        m_inUseIndex.store(m_activeIndex.load());

        return m_objects[m_inUseIndex];
    }

private:
    t_object_type* m_objects[3];
    std::atomic_int m_inUseIndex;
    std::atomic_int m_activeIndex;
    int m_write_index;

    AtomicObject(const AtomicObject &copy) = delete;
    AtomicObject &operator=(const AtomicObject &copy) = delete;
};

// Adapted From: https://github.com/mstump/queues/blob/master/include/spsc-bounded-queue.hpp
template<typename t_element_type>
class AtomicRingBufferSPSC
{
public:
    AtomicRingBufferSPSC(size_t capacity) 
		: m_capacity(capacity)
        , m_mask(capacity - 1)
        , m_buffer(reinterpret_cast<t_element_type*>(new aligned_t[m_capacity + 1])) // need one extra element for a guard
        , m_head(0)
        , m_tail(0)
    {
        // make sure it's a power of 2
        assert((m_capacity != 0) && ((m_capacity & (~m_capacity + 1)) == m_capacity));
    }

    ~AtomicRingBufferSPSC()
    {
        delete[] m_buffer;
    }

	size_t getCapacity() const
	{
		return m_capacity;
	}

    bool enqueue(const t_element_type& input)
    {
        const size_t head = m_head.load(std::memory_order_relaxed);

        if (((m_tail.load(std::memory_order_acquire) - (head + 1)) & m_mask) >= 1)
		{
            m_buffer[head & m_mask] = input;
            m_head.store(head + 1, std::memory_order_release);

            return true;
        }

        return false;
    }

    bool dequeue(t_element_type& output)
    {
        const size_t tail = m_tail.load(std::memory_order_relaxed);

        if (((m_head.load(std::memory_order_acquire) - tail) & m_mask) >= 1) 
		{
            output = m_buffer[m_tail & m_mask];
            m_tail.store(tail + 1, std::memory_order_release);

            return true;
        }

        return false;
    }

private:
    typedef typename std::aligned_storage<sizeof(t_element_type), std::alignment_of<t_element_type>::value>::type aligned_t;
    typedef char cache_line_pad_t[64];

    cache_line_pad_t    pad0;
    const size_t        m_capacity;
    const size_t        m_mask;
    t_element_type* const m_buffer;

    cache_line_pad_t    pad1;
    std::atomic<size_t> m_head;

    cache_line_pad_t    pad2;
    std::atomic<size_t> m_tail;

    AtomicRingBufferSPSC(const AtomicRingBufferSPSC&) {}
    void operator=(const AtomicRingBufferSPSC&) {}
};

#endif // ATOMIC_PRIMITIVES_H
