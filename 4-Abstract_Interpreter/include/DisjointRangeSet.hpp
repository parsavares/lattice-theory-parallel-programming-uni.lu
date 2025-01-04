#ifndef DISJOINTED_INTERVALS_HPP
#define DISJOINTED_INTERVALS_HPP

/*
  =============================================================================
    This file declares the IntervalsUnion class, which represents a set of
    disjoint intervals over integers. It's built on top of the Range type,
    defined in ValueInterval.hpp, to handle arithmetic, joining, and splitting.

    Main responsibilities:
      - Storing multiple disjoint intervals (e.g., [1,3] U [7,10]).
      - Providing basic interval operations like union, meet, addition, etc.
      - Handling potential adjacency merges (e.g., merging [1,2] and [3,3]
        into [1,3]) as an optional heuristic.

    Key methods:
      - join(): merges with another IntervalsUnion.
      - meet(): intersects with another IntervalsUnion (logical "AND").
      - operator+, operator-, operator*, operator/: Overloaded arithmetic.
      - addRange(), removeRange(): expands or shrinks the internal set of intervals.
      - shift(): helps define arithmetic operations via a simple offset.
      - print(): outputs intervals in a user-friendly notation (e.g., "⊥*" or "[1,3] U [7,10]").

    Internally, intervals are kept in a std::set<Range, RangeOrder> to maintain
    them in disjoint sorted form, ensuring efficient merges and lookups.
  =============================================================================
*/

#include <set>
#include <vector>
#include <algorithm>
#include "ValueInterval.hpp"

/**
 * IntervalsUnion:
 *   Represents a collection of disjoint integer ranges and
 *   performs interval arithmetic. Often used to model possible
 *   values of a variable in abstract interpretation.
 */
class IntervalsUnion {
public:
    // Default empty
    IntervalsUnion() = default;

    // Single-range constructor
    IntervalsUnion(const Range &rg) {
        ranges.insert(rg);
    }

    // Build from a vector of Ranges
    IntervalsUnion(const std::vector<Range> &rVec) {
        for (auto &r : rVec) {
            addRange(r);
        }
    }

    // Copy constructor
    IntervalsUnion(const IntervalsUnion &oth) {
        ranges = oth.ranges;
    }

    // Range constructor
    IntervalsUnion(int lb, int ub) {
        ranges.insert(Range(lb, ub));
    }

    /**
     * join():
     *   Creates a new IntervalsUnion that covers the min() of lower bounds
     *   to the max() of upper bounds between this and another union.
     *   It's effectively a bounding "union" if we consider single minimal intervals.
     */
    IntervalsUnion join(const IntervalsUnion &oth) const {
        if (oth.ranges.empty()) return *this;
        if (ranges.empty()) return oth;
        return IntervalsUnion(
            Range(std::min(lb(), oth.lb()),
                  std::max(ub(), oth.ub()))
        );
    }

    /**
     * lb(), ub():
     *   Return the global minimum or maximum bound across all intervals in the set.
     *   If empty, defaults to 0 to avoid UB.
     */
    int lb() const {
        if (ranges.empty()) return 0;
        return ranges.begin()->lb();
    }

    int ub() const {
        if (ranges.empty()) return 0;
        return ranges.rbegin()->ub();
    }

    /**
     * addRange():
     *   Inserts a new range, merging any that overlap or are adjacent.
     *   Ensures the set stays disjoint and sorted.
     */
    void addRange(const Range &rg) {
        if (rg.isEmpty()) return;
        Range tmp = rg;

        // Find insertion spot
        auto it = ranges.lower_bound(tmp);
        if (it != ranges.begin() && std::prev(it)->ub() >= tmp.lb()) {
            --it;
        }

        // Merge overlapping
        while (it != ranges.end() && it->lb() <= tmp.ub()) {
            tmp = tmp.unify(*it);
            it = ranges.erase(it);
        }
        ranges.insert(tmp);

        // Combine adjacency (e.g., [1,2] + [3,3] => [1,3])
        for (auto it2 = ranges.begin(); it2 != ranges.end();) {
            auto nxt = std::next(it2);
            if (nxt != ranges.end() && it2->ub() + 1 == nxt->lb()) {
                Range newR(it2->lb(), nxt->ub());
                it2 = ranges.erase(it2);
                ranges.erase(nxt);
                ranges.insert(newR);
            } else {
                ++it2;
            }
        }
    }

    /**
     * subtract():
     *   Removes ranges from 'this' that appear in the argument.
     */
    void subtract(const IntervalsUnion &oth) {
        for (auto &r : oth.ranges) {
            removeRange(r);
        }
    }

    /**
     * Overloaded arithmetic:
     *   +, -, *, / define how intervals in 'this' combine with intervals in 'oth'.
     *   e.g., A + B shifts all intervals of A by the lb/ub of B, then merges.
     */
    IntervalsUnion operator+(const IntervalsUnion &oth) const {
        if (oth.ranges.empty()) return *this;
        if (ranges.empty()) return oth;

        IntervalsUnion a(*this), b(*this);
        int lbO = oth.lb();
        int ubO = oth.ub();

        auto aShifted = a.shift(lbO);
        auto bShifted = b.shift(ubO);

        return aShifted.join(bShifted);
    }

    IntervalsUnion operator-(const IntervalsUnion &oth) const {
        if (oth.ranges.empty()) return *this;
        if (ranges.empty()) return *this;

        IntervalsUnion a(*this), b(*this);
        int lbO = oth.lb();
        int ubO = oth.ub();

        auto aShifted = a.shift(-lbO);
        auto bShifted = b.shift(-ubO);
        return aShifted.join(bShifted);
    }

    IntervalsUnion operator*(const IntervalsUnion &oth) const {
        if (oth.ranges.empty() || ranges.empty()) return IntervalsUnion();
        IntervalsUnion result;
        for (auto &r1 : ranges) {
            for (auto &r2 : oth.ranges) {
                Range nr = (r1 * r2);
                if (!nr.isEmpty()) {
                    result.addRange(nr);
                }
            }
        }
        return result;
    }

    IntervalsUnion operator/(const IntervalsUnion &oth) const {
        if (oth.ranges.empty() || ranges.empty()) return IntervalsUnion();
        IntervalsUnion result;
        for (auto &r1 : ranges) {
            for (auto &r2 : oth.ranges) {
                Range nr = (r1 / r2);
                if (!nr.isEmpty()) {
                    result.addRange(nr);
                }
            }
        }
        return result;
    }

    /**
     * meet():
     *   Intersection of two sets (logical "AND").
     */
    IntervalsUnion meet(const IntervalsUnion &oth) const {
        std::vector<Range> v;
        for (auto &r1 : ranges) {
            for (auto &r2 : oth.ranges) {
                Range z = r1.intersect(r2);
                if (!z.isEmpty()) {
                    v.push_back(z);
                }
            }
        }
        IntervalsUnion out;
        for (auto &rg : v) {
            out.addRange(rg);
        }
        return out;
    }

    // Basic equality/inequality checks
    bool operator==(const IntervalsUnion &oth) const {
        return (ranges == oth.ranges);
    }
    bool operator!=(const IntervalsUnion &oth) const {
        return !(*this == oth);
    }

    // Simple less/greater checks based on global bounds
    bool operator<(const IntervalsUnion &oth) const {
        return (ub() < oth.lb());
    }
    bool operator>(const IntervalsUnion &oth) const {
        return (lb() > oth.ub());
    }
    bool operator<=(const IntervalsUnion &oth) const {
        return (ub() <= oth.lb());
    }
    bool operator>=(const IntervalsUnion &oth) const {
        return (lb() >= oth.ub());
    }

    /**
     * contains():
     *   Does 'this' hold the integer x or entirely hold the Range rg?
     */
    bool contains(int x) const {
        Range point(x, x);
        auto it = ranges.upper_bound(point);
        if (it == ranges.begin()) return false;
        --it;
        return (it->lb() <= x && x <= it->ub());
    }
    bool contains(const Range &rg) const {
        for (auto &r : ranges) {
            if (r.includes(rg)) {
                return true;
            }
        }
        return false;
    }

    // Number of disjoint intervals
    size_t size() const {
        return ranges.size();
    }

    /**
     * print():
     *   Render this union in a user-friendly format, e.g.
     *   "⊥*" for empty or "[1, 3] U [7, 10]" for a pair.
     */
    void print() const {
        if (ranges.empty()) {
            std::cout << "⊥*";
            return;
        }
        else if (size() == 1 && contains(Range(-INT_MAX, INT_MAX))) {
            std::cout << "⊤*";
            return;
        }
        bool first = true;
        for (auto &r : ranges) {
            if (!first) std::cout << " U ";
            std::cout << r;
            first = false;
        }
    }

    // Allows direct streaming via <<
    friend std::ostream &operator<<(std::ostream &os, const IntervalsUnion &iu) {
        if (iu.ranges.empty()) {
            os << "⊥*";
        } 
        else if (iu.size() == 1 && iu.contains(Range(-INT_MAX, INT_MAX))) {
            os << "⊤*";
        } 
        else {
            bool first = true;
            for (auto &r : iu.ranges) {
                if (!first) os << " U ";
                os << r;
                first = false;
            }
        }
        return os;
    }

private:
    /**
     * RangeOrder:
     *   Compare intervals by comparing their upper/lower bounds to keep them sorted.
     */
    struct RangeOrder {
        bool operator()(const Range &a, const Range &b) const {
            return (a.ub() < b.lb());
        }
    };

    // Internal set of disjoint intervals, sorted by RangeOrder
    std::set<Range, RangeOrder> ranges;

    // Shifts all intervals by a given integer offset
    IntervalsUnion shift(int val) const {
        IntervalsUnion shifted;
        for (auto &rr : ranges) {
            Range singleShift = rr + Range(val, val);
            shifted.addRange(singleShift);
        }
        return shifted;
    }

    // removeRange(): helper for subtract()
    void removeRange(const Range &rg) {
        Range removal = rg;
        int st = removal.lb();
        int ed = removal.ub();

        auto it = ranges.lower_bound(removal);
        if (it != ranges.begin() && std::prev(it)->ub() >= st) {
            --it;
        }

        while (it != ranges.end() && it->lb() <= ed) {
            std::vector<Range> splitted;
            Range overlap = it->intersect(removal);

            // If part of it is below st
            if (it->lb() < st) {
                splitted.push_back(Range(it->lb(), st - 1));
            }
            // If part goes beyond ed
            if (it->ub() > ed) {
                splitted.push_back(Range(ed + 1, it->ub()));
            }

            it = ranges.erase(it);
            for (auto &subR : splitted) {
                ranges.insert(subR);
            }
        }
    }
};

#endif
