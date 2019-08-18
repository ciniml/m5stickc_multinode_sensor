#ifndef RINGBUFFER_HPP__
#define RINGBUFFER_HPP__

#include <cstdint>
#include <array>

// RingBuffer

template<typename TItem, std::size_t Capacity>
class RingBuffer
{
private:
    typedef RingBuffer<TItem, Capacity> SelfType;
    static constexpr std::size_t Mask = Capacity - 1;
    static constexpr std::size_t AdvanceMask = (Capacity<<1) - 1;
    static_assert((Mask & Capacity) == 0, "Capacity must be power of two.");
    static_assert((Capacity<<1) > 0, "Capacity is too big to be expressed with std::size_t.");

    std::array<TItem, Capacity> buffer;
    std::size_t read_index;
    std::size_t write_index;

public:
    RingBuffer() : read_index(0), write_index(0) {}

    bool is_empty() const { return this->read_index == this->write_index; }
    bool is_full() const { return (this->read_index ^ this->write_index) == Capacity; }

    bool queue(const TItem& item)
    {
        if( this->is_full() ) {
            return false;
        }
        this->buffer[this->write_index & Mask] = item;
        this->write_index = (this->write_index + 1) & AdvanceMask;
        return true;
    }

    Result<TItem, bool> dequeue()
    {
        if( this->is_empty() ) {
            return failure(false);
        }

        auto value = success<TItem>(this->buffer[this->read_index & Mask]);
        this->read_index = (this->read_index + 1) & AdvanceMask;
        return std::move(value);
    }

    void reset() 
    {
        while(!this->is_empty()) {
            this->dequeue();
        }
    }
};

#endif // RINGBUFFER_HPP__