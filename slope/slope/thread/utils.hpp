#pragma once
#include <tuple>

namespace slope {

template <class Sequence>
std::pair<typename Sequence::iterator, typename Sequence::iterator> select_sequence_chunk(
    int worker_id, int concurrency, Sequence& sequence)
{
    size_t chunk_size = sequence.size() / concurrency;

    size_t chunk_beg = chunk_size * worker_id;
    size_t chunk_end = (worker_id == concurrency - 1)
                       ? sequence.size()
                       : (chunk_beg + chunk_size);

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
