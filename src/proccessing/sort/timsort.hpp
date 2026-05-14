/**
 * @file timsort.hpp
 * @author Charlie Kushelevsky (charliekushelevsky@gmail.com)
 * @brief Templated in-place timsort optimised for large, mostly-sorted vectors.
 * @date 2026-05-12
 */
#ifndef TIMSORT_HPP
#define TIMSORT_HPP

#include <algorithm>
#include <functional>
#include <cstddef>
#include <vector>

namespace sf::proc {

namespace detail {

/**
 * @brief Compute the minimum run length for a vector of size @p n.
 *
 * Takes the top 6 bits of @p n and rounds up by 1 if any lower bits are set,
 * giving a value in [32, 64]. This keeps the total number of runs close to a
 * power of two so that subsequent merges stay balanced.
 *
 * @param n  Number of elements to be sorted.
 * @return   Minimum run length in [32, 64].
 */
inline std::size_t compute_min_run(std::size_t n)
{
    std::size_t r = 0;
    while (n >= 64) {
        r |= (n & 1);
        n >>= 1;
    }
    return n + r;
}

/**
 * @brief Stable insertion sort on the sub-range @p v[lo..hi] (inclusive).
 *
 * Used to extend short natural runs up to @p min_run before they are pushed
 * onto the merge stack. Insertion sort outperforms more complex algorithms on
 * small or nearly-sorted sub-ranges.
 *
 * @tparam T     Element type.
 * @tparam Comp  Strict-weak-ordering comparator.
 * @param  v     Vector containing the sub-range to sort.
 * @param  lo    Index of the first element in the sub-range.
 * @param  hi    Index of the last element in the sub-range (inclusive).
 * @param  comp  Comparator instance.
 */
template <typename T, typename Comp>
void insertion_sort(std::vector<T>& v, std::size_t lo, std::size_t hi,
                    Comp& comp)
{
    for (std::size_t i = lo + 1; i <= hi; ++i) {
        T key = std::move(v[i]);
        std::size_t j = i;
        while (j > lo && comp(key, v[j - 1])) {
            v[j] = std::move(v[j - 1]);
            --j;
        }
        v[j] = std::move(key);
    }
}

/**
 * @brief Stable merge of @p v[lo..mid] and @p v[mid+1..hi] in place.
 *
 * Copies the left half into @p buf, then merges back into @p v. Elements in
 * the right half that are already in their final position are never moved.
 * Equal elements from the left half are always placed before those from the
 * right half, preserving stability.
 *
 * @tparam T     Element type.
 * @tparam Comp  Strict-weak-ordering comparator.
 * @param  v     Vector containing both runs.
 * @param  lo    Start index of the left run.
 * @param  mid   Last index of the left run (inclusive).
 * @param  hi    Last index of the right run (inclusive).
 * @param  buf   Scratch buffer; resized to fit the left half on each call.
 * @param  comp  Comparator instance.
 */
template <typename T, typename Comp>
void merge_runs(std::vector<T>& v, std::size_t lo, std::size_t mid,
                std::size_t hi, std::vector<T>& buf, Comp& comp)
{
    buf.assign(std::make_move_iterator(v.begin() + lo),
               std::make_move_iterator(v.begin() + mid + 1));

    std::size_t i = 0;         // cursor into buf  (left half)
    std::size_t j = mid + 1;   // cursor into v    (right half)
    std::size_t k = lo;        // write cursor

    const std::size_t left_len = buf.size();
    while (i < left_len && j <= hi) {
        // Prefer left on equality -> stable.
        if (comp(v[j], buf[i]))
            v[k++] = std::move(v[j++]);
        else
            v[k++] = std::move(buf[i++]);
    }
    while (i < left_len)
        v[k++] = std::move(buf[i++]);
    // Remaining right-half elements are already in position.
}

} // namespace detail

/**
 * @brief In-place stable timsort for @c std::vector<T>.
 *
 * Timsort is a hybrid merge/insertion sort that exploits existing order in the
 * input. It scans for natural ascending and strictly-descending runs, extends
 * any run shorter than @p min_run using insertion sort, then merges runs while
 * maintaining two stack invariants:
 * @code
 *   stack[-3].len > stack[-2].len + stack[-1].len
 *   stack[-2].len > stack[-1].len
 * @endcode
 * These invariants bound the total merge work to O(n log n) while keeping
 * merges balanced. Already-sorted input completes in O(n).
 *
 * Vectors with fewer than 64 elements are sorted entirely with insertion sort.
 *
 * @tparam T     Element type. Must be @c MoveConstructible and @c MoveAssignable.
 * @tparam Comp  Strict-weak-ordering binary predicate (defaults to
 *               @c std::less<T>). @p comp(a, b) must return @c true iff
 *               @p a should appear before @p b.
 * @param  vec   Vector to sort in place.
 * @param  comp  Comparator instance.
 */
template <typename T, typename Comp = std::less<T>>
void timsort(std::vector<T>& vec, Comp comp = {})
{
    const std::size_t n = vec.size();
    if (n < 2)
        return;

    if (n < 64) {
        detail::insertion_sort(vec, 0, n - 1, comp);
        return;
    }

    const std::size_t min_run = detail::compute_min_run(n);

    struct Run { std::size_t start, len; };
    std::vector<Run> stack;
    stack.reserve(64); // log2(2^64 / 32) upper bound

    std::vector<T> buf;
    buf.reserve(n / 2);

    // Merge stack[idx] with stack[idx+1] in place.
    auto do_merge = [&](std::size_t idx) {
        Run& A = stack[idx];
        Run& B = stack[idx + 1];
        detail::merge_runs(vec, A.start, A.start + A.len - 1,
                           B.start + B.len - 1, buf, comp);
        A.len += B.len;
        stack.erase(stack.begin() + idx + 1);
    };

    // Restore the two timsort invariants after each run is pushed.
    auto enforce = [&]() {
        while (stack.size() >= 2) {
            const std::size_t s = stack.size();
            bool   should_merge = false;
            std::size_t merge_at = s - 2; // default: merge top two runs

            if (s >= 3) {
                const std::size_t zl = stack[s - 3].len;
                const std::size_t yl = stack[s - 2].len;
                const std::size_t xl = stack[s - 1].len;
                if (zl <= yl + xl) {
                    // Merge whichever neighbour of Y is smaller.
                    merge_at    = (zl < xl) ? s - 3 : s - 2;
                    should_merge = true;
                } else if (yl <= xl) {
                    should_merge = true; // merge_at already s-2
                }
            } else {
                if (stack[s - 2].len <= stack[s - 1].len)
                    should_merge = true;
            }

            if (!should_merge)
                break;
            do_merge(merge_at);
        }
    };

    std::size_t pos = 0;
    while (pos < n) {
        const std::size_t run_start = pos;
        std::size_t       run_end   = pos + 1;

        // Detect a natural run (ascending or strictly descending).
        if (run_end < n) {
            if (comp(vec[run_end], vec[run_start])) {
                // Strictly descending — find extent then reverse for stability.
                while (run_end < n && comp(vec[run_end], vec[run_end - 1]))
                    ++run_end;
                std::reverse(vec.begin() + run_start, vec.begin() + run_end);
            } else {
                // Non-descending.
                while (run_end < n && !comp(vec[run_end], vec[run_end - 1]))
                    ++run_end;
            }
        }

        // Extend short runs to min_run with insertion sort.
        const std::size_t forced_end = std::min(run_start + min_run, n);
        if (run_end < forced_end) {
            detail::insertion_sort(vec, run_start, forced_end - 1, comp);
            run_end = forced_end;
        }

        stack.push_back({run_start, run_end - run_start});
        pos = run_end;
        enforce();
    }

    // Final collapse: merge all remaining runs from the top down.
    while (stack.size() >= 2) {
        const std::size_t s = stack.size();
        do_merge(s - 2);
    }
}

} // namespace sf::proc

#endif // TIMSORT_HPP
