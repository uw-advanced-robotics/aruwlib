/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_DUMMY_ALLOCATOR_HPP_
#define TAPROOT_DUMMY_ALLOCATOR_HPP_

#include <memory>

namespace tap
{
namespace display
{
template <typename T>
class DummyAllocator : public std::allocator<T>
{
public:
    DummyAllocator() : std::allocator<T>() {}

    DummyAllocator(const DummyAllocator& other) = default;

    T* allocate(size_t) { return nullptr; }

    void deallocate(T*) {}
    void destroy(T*) {}
};  // class DummyAllocator
}  // namespace display
}  // namespace tap

#endif  // TAPROOT_DUMMY_ALLOCATOR_HPP_