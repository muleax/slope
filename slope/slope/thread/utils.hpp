#pragma once
#include <tuple>

namespace slope {

inline std::pair<size_t, size_t> select_sequence_chunk(int worker_id, int concurrency, size_t sequence_size)
{
    size_t chunk_size = sequence_size / concurrency;

    size_t chunk_beg = chunk_size * worker_id;
    size_t chunk_end = (worker_id == concurrency - 1)
                    ? sequence_size
                    : (chunk_beg + chunk_size);

    return { chunk_beg, chunk_end };
}

template <class Sequence>
std::pair<typename Sequence::iterator, typename Sequence::iterator> select_sequence_chunk(
    int worker_id, int concurrency, Sequence& sequence)
{
    auto [chunk_beg, chunk_end] = select_sequence_chunk(worker_id, concurrency, sequence.size());
    return { sequence.begin() + chunk_beg, sequence.begin() + chunk_end };
}

template <class Sequence, class Callback>
void iterate_sequence_chunk(int worker_id, int concurrency, Sequence& sequence, const Callback& callback)
{
    auto [chunk_beg, chunk_end] = select_sequence_chunk(worker_id, concurrency, sequence);
    for (auto it = chunk_beg; it != chunk_end; it++)
        callback(it);
}

} // slope
