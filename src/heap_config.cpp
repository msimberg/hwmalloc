/*
 * ghex-org
 *
 * Copyright (c) 2014-2025, ETH Zurich
 * All rights reserved.
 *
 * Please, refer to the LICENSE file in the root directory.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <hwmalloc/config.hpp>
#include <hwmalloc/heap_config.hpp>

#ifdef HWMALLOC_ENABLE_LOGGING
#include <hwmalloc/log.hpp>

#include <string>
#endif

#include <cstdlib>
#include <sstream>
#include <stdexcept>

namespace hwmalloc
{
namespace detail
{
static bool
get_env_bool(const char* name, bool default_value) noexcept
{
    const char* env_value = std::getenv(name);
    if (env_value)
    {
        try
        {
            return std::stoul(env_value) != 0;
        }
        catch (...)
        {
#ifdef HWMALLOC_ENABLE_LOGGING
            HWMALLOC_LOG("failed to parse boolean configuration option", name, "=", env_value,
                "(expected 0 or 1), using default =", default_value);
#endif
            return default_value;
        }
    }

    return default_value;
}

static std::size_t
get_env_size_t(const char* name, std::size_t default_value) noexcept
{
    const char* env_value = std::getenv(name);
    if (env_value)
    {
        try
        {
            return std::stoul(env_value);
        }
        catch (...)
        {
#ifdef HWMALLOC_ENABLE_LOGGING
            HWMALLOC_LOG("failed to parse configuration option", name, "=", env_value,
                ", using default =", default_value);
#endif
            return default_value;
        }
    }

    return default_value;
}
} // namespace detail

heap_config::heap_config(bool never_free, std::size_t num_reserve_segments, std::size_t tiny_limit,
    std::size_t small_limit, std::size_t large_limit, std::size_t tiny_segment_size,
    std::size_t small_segment_size, std::size_t large_segment_size, std::size_t num_huge_alloc_warn_threshold)
: m_never_free{never_free}
, m_num_reserve_segments{num_reserve_segments}
, m_tiny_limit{detail::round_to_pow_of_2(tiny_limit)}
, m_small_limit{detail::round_to_pow_of_2(small_limit)}
, m_large_limit{detail::round_to_pow_of_2(large_limit)}
, m_tiny_segment_size{detail::round_to_pow_of_2(tiny_segment_size)}
, m_small_segment_size{detail::round_to_pow_of_2(small_segment_size)}
, m_large_segment_size{detail::round_to_pow_of_2(large_segment_size)}
, m_num_huge_alloc_warn_threshold{num_huge_alloc_warn_threshold}
{
    // Validate that tiny_limit < small_limit < large_limit
    if (!(m_tiny_limit < m_small_limit && m_small_limit < m_large_limit))
    {
        std::ostringstream os;
        os << "Invalid heap size configuration: HWMALLOC_TINY_LIMIT < HWMALLOC_SMALL_LIMIT < HWMALLOC_LARGE_LIMIT must hold. ";
        os << "Got HWMALLOC_TINY_LIMIT=" << m_tiny_limit
           << ", HWMALLOC_SMALL_LIMIT=" << m_small_limit
           << ", HWMALLOC_LARGE_LIMIT=" << m_large_limit << ".";
        throw std::runtime_error(os.str());
    }

    // Validate that limits are at most segment sizes
    if (!(tiny_limit <= tiny_segment_size && small_limit <= small_segment_size &&
            large_limit <= large_segment_size))
    {
        std::ostringstream os;
        os << "Invalid heap size configuration: Limits must be at most segment sizes. ";
        os << "Got HWMALLOC_TINY_LIMIT=" << m_tiny_limit
           << ", HWMALLOC_TINY_SEGMENT_SIZE=" << m_tiny_segment_size
           << ", HWMALLOC_SMALL_LIMIT=" << m_small_limit
           << ", HWMALLOC_SMALL_SEGMENT_SIZE=" << m_small_segment_size
           << ", HWMALLOC_LARGE_LIMIT=" << m_large_limit
           << ", HWMALLOC_LARGE_SEGMENT_SIZE=" << m_large_segment_size << ".";
        throw std::runtime_error(os.str());
    }
}

heap_config const&
get_default_heap_config()
{
    static heap_config config{
        detail::get_env_bool("HWMALLOC_NEVER_FREE", false),
        detail::get_env_size_t("HWMALLOC_NUM_RESERVE_SEGMENTS", 16u),
        detail::get_env_size_t("HWMALLOC_TINY_LIMIT", (1u << 7)),          // 128B
        detail::get_env_size_t("HWMALLOC_SMALL_LIMIT", (1u << 12)),        // 4KiB
        detail::get_env_size_t("HWMALLOC_LARGE_LIMIT", (1u << 21)),        // 2MiB
        detail::get_env_size_t("HWMALLOC_TINY_SEGMENT_SIZE", (1u << 16)),  // 64KiB
        detail::get_env_size_t("HWMALLOC_SMALL_SEGMENT_SIZE", (1u << 16)), // 64KiB
        detail::get_env_size_t("HWMALLOC_LARGE_SEGMENT_SIZE", (1u << 21)),  // 2MiB
        detail::get_env_size_t("HWMALLOC_NUM_HUGE_ALLOC_WARN_THERESHOLD", 10)
    };

    return config;
}
} // namespace hwmalloc
