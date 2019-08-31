#ifndef VECTOR3_HPP__
#define VECTOR3_HPP__

#include <array>
#include <cstdint>

template<typename TElement>
struct Vector3
{
    std::array<TElement, 3> items;

    Vector3() = default;
    Vector3(const TElement p) : items(p) {}
    Vector3(TElement x, TElement y, TElement z) : items({x, y, z}) {}
    Vector3(const TElement* items) : items({items[0], items[1], items[2]}) {}

    TElement& x() { return this->items[0]; }
    TElement& y() { return this->items[1]; }
    TElement& z() { return this->items[2]; }

    TElement* x_pointer() { return this->items.data() + 0; }
    TElement* y_pointer() { return this->items.data() + 1; }
    TElement* z_pointer() { return this->items.data() + 2; }

    TElement x_const() const { return this->items.at(0); }
    TElement y_const() const { return this->items.at(1); }
    TElement z_const() const { return this->items.at(2); }

    Vector3<TElement>& operator+=(const Vector3& rhs) {
        for(std::size_t i = 0; i < this->items.size(); i++) { this->items[i] += rhs.items[i]; }
        return *this;
    }
    Vector3<TElement>& operator-=(const Vector3& rhs) {
        for(std::size_t i = 0; i < this->items.size(); i++) { this->items[i] -= rhs.items[i]; }
        return *this;
    }
    Vector3<TElement>& operator*=(TElement rhs) {
        for(std::size_t i = 0; i < this->items.size(); i++) { this->items[i] *= rhs; }
        return *this;
    }
    Vector3<TElement>& operator/=(TElement rhs) {
        for(std::size_t i = 0; i < this->items.size(); i++) { this->items[i] /= rhs; }
        return *this;
    }
    
    template<typename OtherElement>
    Vector3<OtherElement> operator*(OtherElement rhs) {
        return Vector3<OtherElement>(this->items[0] * rhs,
                                     this->items[1] * rhs,
                                     this->items[2] * rhs);
    }
    template<typename OtherElement>
    Vector3<OtherElement> operator/(OtherElement rhs) {
        return Vector3<OtherElement>(this->items[0] / rhs,
                                     this->items[1] / rhs,
                                     this->items[2] / rhs);
    }
};

typedef Vector3<float> Vector3F;
typedef Vector3<std::int16_t> Vector3I16;
typedef Vector3<std::int32_t> Vector3I32;

#endif // VECTOR3_HPP__