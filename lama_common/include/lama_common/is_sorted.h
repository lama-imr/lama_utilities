#pragma once
#ifndef LAMA_COMMON_IS_SORTED_H
#define LAMA_COMMON_IS_SORTED_H

namespace lama_common
{

/** Checks if the elements in range [first, last) are sorted in ascending order
  *
  * is_sorted is in STL since C++11.
  * copied from http://en.cppreference.com/w/cpp/algorithm/is_sorted and
  * http://en.cppreference.com/w/cpp/algorithm/is_sorted_until.
  */
template<class ForwardIt>
inline bool is_sorted(ForwardIt first, ForwardIt last)
{
  if (first != last)
  {
    ForwardIt next = first;
    while (++next != last)
    {
      if (*next < *first)
      {
        return false;
      }
      first = next;
    }
  }
  return true;
}

} /* namespace lama_common */

#endif /* end of include guard: LAMA_COMMON_IS_SORTED_H */
