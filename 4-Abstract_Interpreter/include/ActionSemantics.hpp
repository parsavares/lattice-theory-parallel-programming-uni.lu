#ifndef SEMANTICS_HPP
#define SEMANTICS_HPP

/*
   =============================================================================
   This header defines the semantic components for the analyzer: expressions 
   (Expr, Literal, VarExpression, BinExpression, etc.) and control-point actions 
   (SolverInstruction, MergeStates, SetOperator, etc.). Each element manipulates
   or evaluates the VariablesState in some way (e.g., restricting intervals, 
   performing arithmetic, or merging multiple states).
   =============================================================================
*/

#include "InvariantStore.hpp"    // Access to VariablesState & intervals
#include "ValueInterval.hpp"     // Contains IntervalsUnion & Range
#include "ast.hpp"               // LogicOp, BinOp, AST structure

#include <functional>
#include <vector>
#include <memory>
#include <utility>
#include <stdexcept>
#include <algorithm>

// Everything is organized under the `semantics` namespace.
namespace semantics {

    // =========================================================================
    // Expressions: Each implements "evaluate(...)" to produce IntervalsUnion
    // =========================================================================
    class Expr {
    public:
        virtual ~Expr() = default;
        virtual IntervalsUnion evaluate(const VariablesState &vstate) const = 0;
    };
    using ExprPointer = std::shared_ptr<Expr>;

    // A boolean expression: compares two sub-expressions with a LogicOp.
    class BoolExpr {
        ExprPointer left;
        ExprPointer right;
        LogicOp cond;
    public:
        explicit BoolExpr(LogicOp op, ExprPointer l, ExprPointer r)
            : left(std::move(l)), right(std::move(r)), cond(op) {}
        ~BoolExpr() = default;

        bool evaluate(const VariablesState &vstate) {
            IntervalsUnion L = left->evaluate(vstate);
            IntervalsUnion R = right->evaluate(vstate);
            switch (cond) {
                case LogicOp::LE:  // L < R
                case LogicOp::LEQ: // L <= R
                    return (L <= R);
                case LogicOp::GE:  // L > R
                case LogicOp::GEQ: // L >= R
                    return (L >= R);
                case LogicOp::EQ:
                    return (L == R);
                case LogicOp::NEQ:
                    return (L != R);
                default:
                    throw std::runtime_error("Unknown logic operator");
            }
        }
    };
    using BoolExprPointer = std::shared_ptr<BoolExpr>;

    // A literal expression: yields a specific interval [leftVal, rightVal].
    class Literal : public Expr {
        int leftVal;
        int rightVal;
    public:
        explicit Literal(int val) : leftVal(val), rightVal(val) {}
        Literal(int lb, int ub) : leftVal(lb), rightVal(ub) {}
        IntervalsUnion evaluate(const VariablesState&) const override {
            return IntervalsUnion(leftVal, rightVal);
        }
    };

    // A variable expression: obtains the variable's interval from VariablesState.
    class VarExpression : public Expr {
        std::string varName;
    public:
        explicit VarExpression(const std::string &name) : varName(name) {}
        IntervalsUnion evaluate(const VariablesState &vstate) const override {
            // If variable doesn't exist in vstate, assume top to avoid crash
            if (!vstate.hasVariable(varName)) {
                return IntervalsUnion(Range(-INT_MAX, INT_MAX));
            }
            return vstate.getValue(varName);
        }
    };

    // A binary expression: applies an arithmetic BinOp to two Expr sub-expressions.
    class BinExpression : public Expr {
        BinOp operation;
        ExprPointer left;
        ExprPointer right;
        std::function<void(std::string)> notifyWarn;
    public:
        BinExpression(BinOp op, ExprPointer lhs, ExprPointer rhs,
                      std::function<void(std::string)> wfunc)
            : operation(op), left(std::move(lhs)), right(std::move(rhs)), notifyWarn(std::move(wfunc)) {}

        IntervalsUnion evaluate(const VariablesState &vstate) const override {
            IntervalsUnion L = left->evaluate(vstate);
            IntervalsUnion R = right->evaluate(vstate);
            IntervalsUnion result;

            switch (operation) {
                case BinOp::ADD:
                    result = (L + R);
                    break;
                case BinOp::SUB:
                    result = (L - R);
                    break;
                case BinOp::MUL:
                    result = (L * R);
                    break;
                case BinOp::DIV: {
                    // If R could be zero, raise a warning or produce empty.
                    if (R == IntervalsUnion(Range(0, 0))) {
                        notifyWarn("[ERROR] Division by zero!");
                        result = IntervalsUnion();
                    } else if (R.contains(0)) {
                        notifyWarn("[WARNING] Possible division by zero!");
                    }
                    result = (L / R);
                    break;
                }
                default:
                    throw std::runtime_error("Unknown bin op");
            }
            return result;
        }
    };

    // =========================================================================
    // Actions on control points: instructions that manipulate VariablesState.
    // Each instruction has an "execute(...)" method to transform old->new states.
    // =========================================================================
    class SolverInstruction {
    public:
        virtual ~SolverInstruction() = default;
        virtual void execute(std::vector<VariablesState> &prevStates,
                             std::vector<VariablesState> &nextStates) const = 0;
    };

    // Merges multiple states into one (like an if-else join).
    class MergeStates : public SolverInstruction {
        std::vector<size_t> controlPoints;
        size_t targetCP;
    public:
        explicit MergeStates(size_t tCP, std::vector<size_t> cPoints)
            : controlPoints(std::move(cPoints)), targetCP(tCP) {}

        void execute(std::vector<VariablesState> &oldFrames,
                     std::vector<VariablesState> &newFrames) const override {
            VariablesState &dest = newFrames[targetCP];
            // Start from the first branch
            dest = oldFrames[controlPoints[0]];
            // Join the rest
            for (size_t i = 1; i < controlPoints.size(); i++) {
                dest = dest.unionWith(oldFrames[controlPoints[i]]);
            }
        }
    };

    // A base class for instructions that read from one location
    // and write to another (StateOperator).
    class StateOperator : public SolverInstruction {
    protected:
        size_t inputCP;
        size_t outputCP;
    public:
        StateOperator(size_t inCP, size_t outCP)
            : inputCP(inCP), outputCP(outCP) {}
        StateOperator(size_t inCP)
            : inputCP(inCP), outputCP((inCP < 0) ? 0 : inCP + 1) {}
        virtual ~StateOperator() = default;

        virtual void actOn(const VariablesState &inState, 
                           VariablesState &outState) const = 0;

        void execute(std::vector<VariablesState> &prevStates,
                     std::vector<VariablesState> &nextStates) const override {
            assert(inputCP < prevStates.size());
            assert(outputCP < nextStates.size());

            // If the input isn't empty, update the output accordingly
            if (prevStates[inputCP].isBottomState() || prevStates[inputCP].count() > 0) {
                // Copy (or combine) the old state, then act
                VariablesState temp = prevStates[inputCP];
                actOn(prevStates[inputCP], temp);
                // Union with the existing state at outputCP
                nextStates[outputCP] = nextStates[outputCP].unionWith(temp);
            }
        }
    };

    // SetOperator: assigns the result of an expression to a variable.
    class SetOperator : public StateOperator {
        std::string varTarget;
        ExprPointer expr;
    public:
        SetOperator(const std::string &varName, ExprPointer e, size_t inCP)
            : StateOperator(inCP), varTarget(varName), expr(std::move(e)) {}

        SetOperator(const std::string &varName, ExprPointer e, size_t inCP, size_t outCP)
            : StateOperator(inCP, outCP), varTarget(varName), expr(std::move(e)) {}

        void actOn(const VariablesState &inState, VariablesState &outState) const override {
            outState = inState;
            // If variable not declared, we could do nothing or auto-declare
            outState.setValue(varTarget, expr->evaluate(inState));
        }
    };

    // VarAllocator: a "declaration" that inserts a variable at top range.
    class VarAllocator : public StateOperator {
        std::string varName;
    public:
        VarAllocator(const std::string &v, size_t inCP)
            : StateOperator(inCP), varName(v) {}
        VarAllocator(const std::string &v, size_t inCP, size_t outCP)
            : StateOperator(inCP, outCP), varName(v) {}

        void actOn(const VariablesState &inState, VariablesState &outState) const override {
            outState = inState;
            // New var: default to top interval
            outState.setValue(varName, IntervalsUnion(Range(INT_MIN, INT_MAX)));
        }
    };

    // CheckOperator: evaluates a BoolExpr and logs an error if false.
    class CheckOperator : public StateOperator {
        BoolExprPointer boolExpr;
        std::function<void(std::string)> pushWarn;
    public:
        CheckOperator(BoolExprPointer bExpr, std::function<void(std::string)> wCallback, size_t inCP)
            : StateOperator(inCP), boolExpr(std::move(bExpr)), pushWarn(std::move(wCallback)) {}
        CheckOperator(BoolExprPointer bExpr, std::function<void(std::string)> wCallback, size_t inCP, size_t outCP)
            : StateOperator(inCP, outCP), boolExpr(std::move(bExpr)), pushWarn(std::move(wCallback)) {}

        void actOn(const VariablesState &inState, VariablesState &outState) const override {
            bool result = boolExpr->evaluate(inState);
            if (!result) {
                pushWarn("[ERROR] Assertion not satisfied!");
            }
            outState = inState;
        }
    };

    // RestrictOperator: Narrows a variable's interval based on a LogicOp with an expression.
    class RestrictOperator : public StateOperator {
        std::string leftVar;
        ExprPointer rightVal;
        LogicOp cond;
    public:
        RestrictOperator(LogicOp op, const std::string &lVar, ExprPointer rExpr, size_t inCP)
            : StateOperator(inCP), leftVar(lVar), rightVal(std::move(rExpr)), cond(op) {}

        RestrictOperator(LogicOp op, const std::string &lVar, ExprPointer rExpr,
                         size_t inCP, size_t outCP)
            : StateOperator(inCP, outCP), leftVar(lVar), rightVal(std::move(rExpr)), cond(op) {}

        void actOn(const VariablesState &inState, VariablesState &outState) const override {
            outState = inState;
            IntervalsUnion L = inState.hasVariable(leftVar)
                ? inState.getValue(leftVar)
                : IntervalsUnion(Range(-INT_MAX, INT_MAX));
            IntervalsUnion R = rightVal->evaluate(inState);

            // For now, assume R is [c,c].
            int rlb = R.lb();
            int rub = R.ub();

            switch (cond) {
                case LogicOp::LE:
                    outState.narrow(leftVar, Range(INT_MIN, rub - 1));
                    break;
                case LogicOp::LEQ:
                    outState.narrow(leftVar, Range(INT_MIN, rub));
                    break;
                case LogicOp::GE:
                    outState.narrow(leftVar, Range(rlb + 1, INT_MAX));
                    break;
                case LogicOp::GEQ:
                    outState.narrow(leftVar, Range(rlb, INT_MAX));
                    break;
                case LogicOp::EQ:
                    outState.filterEqual(leftVar, R);
                    break;
                case LogicOp::NEQ:
                    outState.filterNotEqual(leftVar, R);
                    break;
            }
        }
    };

} // end namespace semantics

#endif
