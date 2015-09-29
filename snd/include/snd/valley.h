#pragma once
#ifndef SND_VALLEY_H
#define SND_VALLEY_H

#include <cmath> // For std::abs.
#include <vector>

#include <snd/gap.h>
#include <snd/utilities.h>

namespace snd
{

class Valley
{
  public :

    Valley(const Gap& right_gap, const Gap& left_gap);
    
    Gap right_gap;  //!< The gap with the lowest index.
    Gap left_gap;  //!< The gap with the highest index.

    bool bestGapIsRightGap() const {return best_gap_is_right_;}

    /* A reference to the best gap.
     *
     * If two rising gaps, this is the right one. If two falling gaps, the left one. Otherwise, the gap which is the closest to the goal.
     */
    const Gap& bestGap() const;

    /* A reference to the gap which is not the best_gap.
     */
    const Gap& otherGap() const;

    bool angleInValley(double angle, double margin=0.0) const;

  protected :

    bool best_gap_is_right_;
};

} /* namespace snd */

#endif /* SND_VALLEY_H */

