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

#include <hwmalloc/detail/pool.hpp>
#include <vector>

namespace hwmalloc
{
namespace detail
{
template<typename Context>
class fixed_size_heap
{
  public:
    using pool_type = pool<Context>;
    using block_type = typename pool_type::block_type;

  private:
    Context*                                m_context;
    std::size_t                             m_block_size;
    std::size_t                             m_segment_size;
    bool                                    m_never_free;
    std::size_t                             m_num_reserve_segments;
    std::vector<std::unique_ptr<pool_type>> m_pools;
#if HWMALLOC_ENABLE_DEVICE
    std::size_t                             m_num_devices;
    std::vector<std::unique_ptr<pool_type>> m_device_pools;
#endif

  public:
    fixed_size_heap(Context* context, std::size_t block_size, std::size_t segment_size,
        bool never_free, std::size_t num_reserve_segments,
        const typename pool_type::segment_alloc_cb_type& segment_alloc_cb = nullptr)
    : m_context(context)
    , m_block_size(block_size)
    , m_segment_size(segment_size)
    , m_never_free(never_free)
    , m_num_reserve_segments{num_reserve_segments}
    , m_pools(numa().local_nodes().size())
#if HWMALLOC_ENABLE_DEVICE
    , m_num_devices{(std::size_t)get_num_devices()}
    , m_device_pools(numa().local_nodes().size() * m_num_devices)
#endif
    {
        for (auto [n, i] : numa().local_nodes())
        {
            m_pools[i] = std::make_unique<pool_type>(m_context, m_block_size, m_segment_size, n,
                m_never_free, m_num_reserve_segments, segment_alloc_cb);
#if HWMALLOC_ENABLE_DEVICE
            for (unsigned int j = 0; j < m_num_devices; ++j)
            {
                m_device_pools[i * m_num_devices + j] = std::make_unique<pool_type>(m_context,
                    m_block_size, m_segment_size, n, (int)j, m_never_free, m_num_reserve_segments,
                    segment_alloc_cb);
            }
#endif
        }
    }

    fixed_size_heap(fixed_size_heap const&) = delete;
    fixed_size_heap(fixed_size_heap&&) = default;

    block_type allocate(std::size_t numa_node)
    {
        return m_pools[numa_node_index(numa_node)]->allocate();
    }

#if HWMALLOC_ENABLE_DEVICE
    block_type allocate(std::size_t numa_node, int device_id)
    {
        return m_device_pools[numa_node_index(numa_node) * m_num_devices + device_id]->allocate();
    }
#endif

    void free(block_type const& b) { b.release(); }

  private:
    auto numa_node_index(std::size_t numa_node) const noexcept
    {
        auto it = numa().local_nodes().find(numa_node);
        return (it != numa().local_nodes().end()
                    ? it->second
                    : numa().local_nodes().find(numa().local_node())->second);
    }
};

} // namespace detail
} // namespace hwmalloc
