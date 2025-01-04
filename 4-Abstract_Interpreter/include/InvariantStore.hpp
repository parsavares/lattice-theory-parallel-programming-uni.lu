#ifndef STORE_HPP
#define STORE_HPP

/*
  =============================================================================
    InvariantStore.hpp

    Declares VariablesState, our container for each variable's possible values
    (IntervalsUnion). Also handles combining states (unionWith) and refining
    them (narrowing, widening).
  =============================================================================
*/

#include "DisjointRangeSet.hpp"     // IntervalsUnion definition
#include <unordered_map>
#include <string>
#include <iostream>

/**
 * VariablesState:
 *   - Maps variable names (std::string) to a set of possible values (IntervalsUnion).
 *   - Supports unionWith (join of states), narrowing (meet), and a simple widening.
 *   - bottomFlag indicates whether this state is "bottom" (e.g., no valid assignments).
 */
class VariablesState {
protected:
    // varMapping: keeps track of each variable's IntervalsUnion
    std::unordered_map<std::string, IntervalsUnion> varMapping;
    bool bottomFlag = false;

public:
    VariablesState() = default;
    VariablesState(const VariablesState &oth)
        : varMapping(oth.varMapping), bottomFlag(oth.bottomFlag) {}

    // Mark this state as the "bottom" one
    void markBottomState(bool val) {
        bottomFlag = val;
    }
    bool isBottomState() const {
        return bottomFlag;
    }

    // Equality comparison includes bottomFlag and varMapping
    bool operator==(const VariablesState &oth) const {
        return (varMapping == oth.varMapping) && (bottomFlag == oth.bottomFlag);
    }
    bool operator!=(const VariablesState &oth) const {
        return !(*this == oth);
    }

    // Check if a variable name is in varMapping
    bool hasVariable(const std::string &v) const {
        return (varMapping.find(v) != varMapping.end());
    }

    // Quick access to an IntervalsUnion for a given variable
    IntervalsUnion &operator[](const std::string &v) {
        return varMapping[v];
    }
    const IntervalsUnion &getValue(const std::string &v) const {
        return varMapping.at(v);
    }

    // Directly sets the IntervalsUnion for a variable
    void setValue(const std::string &var, const IntervalsUnion &val) {
        varMapping[var] = val;
    }

    // Returns how many distinct variables we track
    size_t count() const {
        return varMapping.size();
    }

    // ----------------------------------------------------
    // unionWith():
    //   Creates a new state that is the union of *this and oth.
    //   For each variable, intervals are merged (join).
    // ----------------------------------------------------
    VariablesState unionWith(const VariablesState &oth) const {
        VariablesState result(*this);
        for (auto &pr : oth.varMapping) {
            const std::string &varName = pr.first;
            const IntervalsUnion &newVal = pr.second;

            if (!result.hasVariable(varName)) {
                result.varMapping[varName] = newVal;
            } else {
                result.varMapping[varName] =
                    result.varMapping[varName].join(newVal);
            }
        }
        return result;
    }

    // ----------------------------------------------------
    // NARROWING:
    //   We can meet (intersect) intervals to further refine them.
    // ----------------------------------------------------
    void narrow(const std::string &var, const Range &rng) {
        if (!hasVariable(var)) return;
        IntervalsUnion tmp(rng);
        varMapping[var] = varMapping[var].meet(tmp);
    }
    void filterEqual(const std::string &var, const IntervalsUnion &val) {
        if (!hasVariable(var)) return;
        varMapping[var] = varMapping[var].meet(val);
    }
    void filterNotEqual(const std::string &var, const IntervalsUnion &val) {
        if (!hasVariable(var)) return;
        varMapping[var].subtract(val);
    }

    // ----------------------------------------------------
    // widenWith():
    //   Over-approximate intervals if they're strictly larger
    //   than in the oldState. This is used to ensure loops
    //   eventually converge to a fixpoint.
    // ----------------------------------------------------
    VariablesState widenWith(const VariablesState &oldState) const {
        VariablesState widened(*this);

        // For each variable in oldState, see if we need to widen
        for (auto &oldEntry : oldState.varMapping) {
            const std::string &vName = oldEntry.first;
            const IntervalsUnion &oldVal = oldEntry.second;

            // If new state doesn't know about vName, adopt oldVal
            if (!widened.hasVariable(vName)) {
                widened.varMapping[vName] = oldVal;
                continue;
            }

            IntervalsUnion &newVal = widened.varMapping[vName];
            newVal = doIntervalWiden(oldVal, newVal);
        }
        // Extra variables from *this remain as is

        return widened;
    }

    // Helper for interval-based widening
    static IntervalsUnion doIntervalWiden(
        const IntervalsUnion &oldIU,
        const IntervalsUnion &newIU
    ) {
        int oldLB = oldIU.lb();
        int oldUB = oldIU.ub();
        int newLB = newIU.lb();
        int newUB = newIU.ub();

        // By default, keep new bounds
        int finalLB = newLB;
        int finalUB = newUB;

        // If newLB < oldLB => push lower to -∞
        if (newLB < oldLB) {
            finalLB = -INT_MAX;
        }
        // If newUB > oldUB => push upper to +∞
        if (newUB > oldUB) {
            finalUB = INT_MAX;
        }
        return IntervalsUnion(Range(finalLB, finalUB));
    }

    // For debugging/printing the current variable -> intervals
    void display() const {
        if (varMapping.empty()) {
            std::cout << "(all unconstrained)" << std::endl;
            return;
        }
        for (auto &pair : varMapping) {
            std::cout << pair.first << " -> " << pair.second << std::endl;
        }
    }
};

#endif // STORE_HPP
