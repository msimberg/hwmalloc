/*
 * ghex-org
 *
 * Copyright (c) 2014-2025, ETH Zurich
 * All rights reserved.
 *
 * Please, refer to the LICENSE file in the root directory.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma once

#include <cstddef>

namespace hwmalloc
{
namespace detail
{
inline constexpr std::size_t
log2_c(std::size_t n) noexcept
{
    return ((n < 2) ? 1 : 1 + log2_c(n >> 1));
}

inline constexpr std::size_t
round_to_pow_of_2(std::size_t n) noexcept
{
    return 1u << log2_c(n - 1);
}
} // namespace detail

struct heap_config
{
    bool                         m_never_free;
    std::size_t                  m_num_reserve_segments;
    std::size_t                  m_tiny_limit;
    std::size_t                  m_small_limit;
    std::size_t                  m_large_limit;
    std::size_t                  m_bucket_shift = detail::log2_c(m_tiny_limit) - 1;
    std::size_t                  m_tiny_segment_size;
    std::size_t                  m_small_segment_size;
    std::size_t                  m_large_segment_size;
    std::size_t                  m_num_huge_alloc_warn_threshold;
    static constexpr std::size_t m_tiny_increment_shift = 3;
    static constexpr std::size_t m_tiny_increment = (1u << m_tiny_increment_shift);
    std::size_t                  m_num_tiny_heaps = m_tiny_limit / m_tiny_increment;
    std::size_t m_num_small_heaps = detail::log2_c(m_small_limit) - detail::log2_c(m_tiny_limit);
    std::size_t m_num_large_heaps = detail::log2_c(m_large_limit) - detail::log2_c(m_small_limit);

    heap_config(bool never_free, std::size_t num_reserve_segments, std::size_t tiny_limit,
        std::size_t small_limit, std::size_t large_limit, std::size_t tiny_segment_size,
        std::size_t small_segment_size, std::size_t large_segment_size, std::size_t num_huge_alloc_warn_threshold);
};

heap_config const& get_default_heap_config();
} // namespace hwmalloc
