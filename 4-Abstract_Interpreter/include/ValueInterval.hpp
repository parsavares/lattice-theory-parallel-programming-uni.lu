#ifndef INTERVAL_HPP
#define INTERVAL_HPP

/*
  =============================================================================
    ValueInterval.hpp

    Declares the Range class, representing a single integer interval [low, high]
    or an empty interval. Provides arithmetic operators (e.g. +, -, /, *) and
    intersection/union logic. This is the atomic building block of the multi-
    range sets (IntervalsUnion).
  =============================================================================
*/

#include <utility>
#include <algorithm>
#include <climits>
#include <assert.h>
#include <iostream>

/**
 * Range:
 *   A simple [low, high] integer interval, with an emptyFlag to signify
 *   a bottom (unusable) range. Provides basic arithmetic, unify, intersect, etc.
 */
class Range {
    std::pair<int, int> bounds;
    bool emptyFlag;

public:
    // Basic constructor with bounds, ensuring low <= high
    Range(int low, int high) : emptyFlag(false) {
        assert(low <= high);
        bounds = std::make_pair(low, high);
    }

    // Default: top interval [-INT_MAX, INT_MAX]
    Range() : bounds(-INT_MAX, INT_MAX), emptyFlag(false) {}

    // Creates an empty (bottom) interval
    static Range mkEmpty() {
        Range tmp;
        tmp.emptyFlag = true;
        return tmp;
    }

    void setEmpty(bool st) {
        emptyFlag = st;
    }

    int lb() const {
        return emptyFlag ? 0 : bounds.first;
    }
    int ub() const {
        return emptyFlag ? 0 : bounds.second;
    }

    void setLb(int v) {
        if (!emptyFlag) bounds.first = v;
    }
    void setUb(int v) {
        if (!emptyFlag) bounds.second = v;
    }

    // unify(): bounding union of two Ranges
    Range unify(const Range &oth) const {
        if (emptyFlag) return oth;
        if (oth.emptyFlag) return *this;
        return Range(
            std::min(bounds.first, oth.bounds.first),
            std::max(bounds.second, oth.bounds.second)
        );
    }

    // intersect(): bounding intersection
    Range intersect(const Range &oth) const {
        if (emptyFlag || oth.emptyFlag) return mkEmpty();
        int newL = std::max(bounds.first, oth.bounds.first);
        int newU = std::min(bounds.second, oth.bounds.second);
        if (newL > newU) return mkEmpty();
        return Range(newL, newU);
    }

    // Overloaded arithmetic: merges bounds
    Range operator+(const Range &oth) const {
        if (emptyFlag || oth.emptyFlag) return mkEmpty();
        return Range(
            bounds.first + oth.bounds.first,
            bounds.second + oth.bounds.second
        );
    }
    Range operator-(const Range &oth) const {
        if (emptyFlag || oth.emptyFlag) return mkEmpty();
        return Range(
            bounds.first - oth.bounds.second,
            bounds.second - oth.bounds.first
        );
    }
    Range operator*(const Range &oth) const {
        if (emptyFlag || oth.emptyFlag) return mkEmpty();
        int a = bounds.first  * oth.bounds.first;
        int b = bounds.first  * oth.bounds.second;
        int c = bounds.second * oth.bounds.first;
        int d = bounds.second * oth.bounds.second;
        return Range(std::min({a,b,c,d}), std::max({a,b,c,d}));
    }
    Range operator/(const Range &oth) const {
        if (emptyFlag || oth.emptyFlag) return mkEmpty();

        // If the divisor is strictly 0 => can't proceed
        if (oth.bounds.first == 0 && oth.bounds.second == 0) {
            return mkEmpty();
        }

        // Exclude zero from the divisor's range
        Range tmp = oth;
        if (tmp.ub() == 0) tmp.setUb(-1);
        if (tmp.lb() == 0) tmp.setLb(1);

        int a = bounds.first  / tmp.bounds.first;
        int b = bounds.first  / tmp.bounds.second;
        int c = bounds.second / tmp.bounds.first;
        int d = bounds.second / tmp.bounds.second;
        return Range(std::min({a,b,c,d}), std::max({a,b,c,d}));
    }

    // Basic comparisons
    bool operator==(const Range &oth) const {
        return (emptyFlag == oth.emptyFlag) && (bounds == oth.bounds);
    }
    bool operator!=(const Range &oth) const {
        return !(*this == oth);
    }

    // For the domain we consider: "range <= range" means upper bound is below or equal
    bool operator<=(const Range &oth) const {
        if (emptyFlag) return true;
        if (oth.emptyFlag) return false;
        return bounds.second <= oth.bounds.first;
    }
    bool operator>=(const Range &oth) const {
        if (emptyFlag) return false;
        if (oth.emptyFlag) return true;
        return bounds.first >= oth.bounds.second;
    }

    bool isEmpty() const {
        return emptyFlag;
    }

    // Check if we fully include another Range
    bool includes(const Range &oth) const {
        if (emptyFlag) return false;
        if (oth.emptyFlag) return true;
        return (bounds.first <= oth.bounds.first &&
                bounds.second >= oth.bounds.second);
    }

    // Output formatting for debugging
    friend std::ostream &operator<<(std::ostream &os, const Range &rr) {
        if (rr.emptyFlag) {
            os << "⊥"; // bottom
        }
        else if (rr == Range(-INT_MAX, INT_MAX)) {
            os << "⊤"; // top
        }
        else {
            os << "[" << rr.bounds.first << ", " << rr.bounds.second << "]";
        }
        return os;
    }
};

#endif // INTERVAL_HPP
