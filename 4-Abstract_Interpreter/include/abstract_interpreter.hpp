#ifndef ABSTRACT_INTERPRETER_HPP
#define ABSTRACT_INTERPRETER_HPP

/*
   =============================================================================
   This header declares the FancyAnalyzer class, which drives the core of our
   abstract interpretation. It produces SolverInstructions from the AST and
   iterates the resulting system until reaching a fixpoint. The main pieces are:
   
   - solverActions: a list of semantic commands (assign, merge, restrict, etc.).
   - frames: a sequence of VariablesState objects, one per program location.
   - warnings: a simple map tracking potential issues (e.g., division-by-zero).
   
   The workflow:
     1) constructSolverSystem() builds solverActions from the AST.
     2) solveSystem() repeatedly calls solverStep() to update frames.
     3) if it doesn't converge soon, applyWidening(...) triggers an overapprox.
     4) once stable, results can be printed (printSystemState) along with any
        warnings (printSystemNotes).
   =============================================================================
*/

#include "InvariantStore.hpp"      // Access to VariablesState definition
#include "ValueInterval.hpp"       // Interval logic for numeric analysis
#include "ast.hpp"                 // Our AST node definitions
#include "ActionSemantics.hpp"     // Classes for semantic actions (assign, merge, etc.)

#include <vector>
#include <functional>
#include <memory>
#include <variant>
#include <unordered_map>
#include <cassert>

// Each location in the program has a VariablesState
using StatesCollection = std::vector<VariablesState>;

// A list of semantic instructions that update or merge VariablesState objects
using SolverComponentList = std::vector<std::shared_ptr<semantics::SolverInstruction>>;

class FancyAnalyzer {
private:
    SolverComponentList solverActions;          // All the instructions we run over time
    StatesCollection frames;                    // Current environment for each location
    std::unordered_map<size_t, std::string> warnings; // Records errors/warnings (nodeID->message)

    // Executes each instruction once, updates frames, and checks stability
    bool solverStep();

    // Enlarges intervals to speed up convergence on loops
    void applyWidening(const StatesCollection &oldFrames, StatesCollection &newFrames);

    // Recursively builds an arithmetic BinExpression from an AST node
    std::shared_ptr<semantics::BinExpression> 
        buildBinaryExpression(const ASTNode &node, std::function<void(std::string)> warnFunc);

public:
    // Walks the AST to build the solver instructions
    void constructSolverSystem(const ASTNode &node);

    // Iterates until fixpoint or max iterations, possibly widens along the way
    void solveSystem();

    // Prints the final environment of variables for each location
    void printSystemState() const;

    // Prints any warnings or errors that arose during interpretation
    void printSystemNotes() const;
};

#endif // ABSTRACT_INTERPRETER_HPP
