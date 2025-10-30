/*
 * ghex-org
 *
 * Copyright (c) 2014-2023, ETH Zurich
 * All rights reserved.
 *
 * Please, refer to the LICENSE file in the root directory.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include <hwmalloc/detail/segment.hpp>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <functional>

namespace hwmalloc
{
namespace detail
{
template<typename Context>
class pool
{
  public:
    using segment_type = segment<Context>;
    using block_type = typename segment_type::block;
    using stack_type = boost::lockfree::stack<block_type>;
    using segment_map = std::unordered_map<segment_type*, std::unique_ptr<segment_type>>;
    using segment_alloc_cb_type = std::function<void()>;

  private:
    static std::size_t num_pages(std::size_t segment_size) noexcept
    {
        auto x = (segment_size + numa().page_size() - 1) / numa().page_size();
        return x;
    }

    static auto check_allocation(numa_tools::allocation const& a, std::size_t expected_numa_node)
    {
        if (!a) { throw std::runtime_error("could not allocate system memory"); }
        else if (a.node != expected_numa_node)
        {
            numa().free(a);
            throw std::runtime_error("could not allocate on requested numa node "
              + std::to_string(expected_numa_node));
        }
        return a;
    }

  private:
    Context*                     m_context;
    std::size_t                  m_block_size;
    std::size_t                  m_segment_size;
    std::size_t                  m_numa_node;
    bool                         m_never_free;
    std::size_t                  m_num_reserve_segments;
    stack_type                   m_free_stack;
    segment_map                  m_segments;
    std::mutex                   m_mutex;
    int                          m_device_id = 0;
    bool                         m_allocate_on_device = false;
    // Callback executed when a segment is allocated. Can be used to track the number of actual
    // memory allocations.
    const segment_alloc_cb_type& m_segment_alloc_cb;

    void add_segment()
    {
        if (m_segment_alloc_cb)
            m_segment_alloc_cb();
        auto a =
            check_allocation(numa().allocate(num_pages(m_segment_size), m_numa_node), m_numa_node);
#if HWMALLOC_ENABLE_DEVICE
        if (m_allocate_on_device)
        {
            const auto tmp = get_device_id();
            set_device_id(m_device_id);
            void* device_ptr = device_malloc(a.size);

            auto s = std::make_unique<segment_type>(this,
                hwmalloc::register_memory(*m_context, a.ptr, a.size), a,
                hwmalloc::register_device_memory(*m_context, m_device_id, device_ptr, a.size), device_ptr,
                m_device_id, m_block_size, m_free_stack);
            m_segments[s.get()] = std::move(s);
            set_device_id(tmp);
        }
        else
#endif
        {
            auto s = std::make_unique<segment_type>(this,
                hwmalloc::register_memory(*m_context, a.ptr, a.size), a, m_block_size,
                m_free_stack);
            m_segments[s.get()] = std::move(s);
        }
    }

  public:
    pool(Context* context, std::size_t block_size, std::size_t segment_size, std::size_t numa_node,
        bool never_free, std::size_t num_reserve_segments,
        const segment_alloc_cb_type& segment_alloc_cb)
    : m_context{context}
    , m_block_size{block_size}
    , m_segment_size{segment_size}
    , m_numa_node{numa_node}
    , m_never_free{never_free}
    , m_num_reserve_segments{std::max(num_reserve_segments, 1ul)}
    , m_free_stack(segment_size / block_size)
    , m_segment_alloc_cb(segment_alloc_cb)
    {
    }

#if HWMALLOC_ENABLE_DEVICE
    pool(Context* context, std::size_t block_size, std::size_t segment_size, std::size_t numa_node,
        int device_id, bool never_free, std::size_t num_reserve_segments,
        const segment_alloc_cb_type& segment_alloc_cb)
    : pool(context, block_size, segment_size, numa_node, never_free, num_reserve_segments, segment_alloc_cb)
    {
        m_device_id = device_id;
        m_allocate_on_device = true;
    }
#endif

    auto allocate()
    {
        block_type b;
        if (m_free_stack.pop(b)) return b;
        m_mutex.lock();
        if (m_free_stack.pop(b))
        {
            m_mutex.unlock();
            return b;
        }
        for (auto& kvp : m_segments) kvp.first->collect(m_free_stack);
        if (m_free_stack.pop(b))
        {
            m_mutex.unlock();
            return b;
        }
        unsigned int counter = 0;
        while (!m_free_stack.pop(b))
        {
            // add segments every 2nd iteration
            if (counter++ % 2 == 0) add_segment();
        }
        m_mutex.unlock();
        return b;
    }

    void free(block_type const& b)
    {
        b.m_segment->free(b);
        if (!m_never_free && b.m_segment->is_empty())
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (b.m_segment->is_empty() && m_segments.size() > m_num_reserve_segments)
            {
#if HWMALLOC_ENABLE_DEVICE
                if (m_allocate_on_device)
                {
                    const auto tmp = get_device_id();
                    set_device_id(m_device_id);
                    m_segments.erase(b.m_segment);
                    set_device_id(tmp);
                }
                else
#endif
                    m_segments.erase(b.m_segment);
            }
        }
    }
};

} // namespace detail
} // namespace hwmalloc
