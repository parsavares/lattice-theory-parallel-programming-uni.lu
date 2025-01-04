#include "abstract_interpreter.hpp"

/**
 * =============================================================================
 * File: abstract_interpreter.cpp
 * =============================================================================
 * 
 * This file contains the implementation of the `FancyAnalyzer` class, which
 * serves as the central component of the abstract interpretation framework. 
 * This static analysis tool aims to verify user-defined assertions and detect 
 * potential program bugs, such as division-by-zero and integer overflow, 
 * using interval-based abstract domains.
 *
 * =============================================================================
 * Key Features and Design:
 * =============================================================================
 * 
 * 1. Abstract Interpretation Framework:
 *    - Constructs a symbolic execution engine that evaluates the Abstract Syntax Tree (AST).
 *    - Tracks variable states using an interval-based representation to identify bugs.
 * 
 * 2. Static Analysis Goals:
 *    - Verify user-provided assertions (`assert(condition)`).
 *    - Identify and warn about general property violations, including:
 *      - Division-by-zero
 *      - Overflow within 32-bit integers
 * 
 * 3. Recursive AST Traversal:
 *    - Translates AST nodes into a sequence of `SolverInstruction` objects, which
 *      are processed iteratively to compute variable invariants.
 *    - Handles control flow constructs like conditionals (`if-else`) and loops,
 *      ensuring accurate propagation of abstract states.
 * 
 * 4. Widening for Convergence:
 *    - Implements widening techniques to guarantee termination in the presence 
 *      of loops or unbounded iteration.
 * 
 * =============================================================================
 * Implementation Details:
 * =============================================================================
 * 
 * - Binary Expression Handling:
 *   The `buildBinaryExpression` function recursively parses arithmetic operations
 *   into `BinExpression` objects, facilitating abstract evaluation.
 * 
 * - Solver System Construction:
 *   The `constructSolverSystem` function converts AST nodes into executable 
 *   semantic actions (`SolverInstruction` objects) for evaluation.
 * 
 * - Fixpoint Computation:
 *   The `solveSystem` function iteratively evaluates the solver system until a 
 *   fixpoint is achieved or a maximum iteration limit is exceeded.
 * 
 * - Widening Techniques:
 *   The `applyWidening` function adjusts variable states to ensure convergence,
 *   particularly for loops and unbounded execution paths.
 * 
 * - Logging and Reporting:
 *   Comprehensive logging is integrated to provide insight into analysis progress,
 *   including variable state transitions, warnings, and error messages.
 * 
 * =============================================================================
 * Usage in the Context of Static Analysis:
 * =============================================================================
 * 
 * This implementation is tailored for the "Abstract Interpreter" project, which
 * requires incremental improvements to handle various test programs:
 * 
 * - Assertions:
 *   Verifies explicit assertions in the form of `assert(condition)` to ensure
 *   the correctness of program invariants.
 * 
 * - General Property Violations:
 *   Detects division-by-zero and overflow errors through interval analysis.
 * 
 * - Control Flow Constructs:
 *   Supports the analysis of `if-else` statements and loops by splitting 
 *   execution paths and joining states at merge points.
 * 
 * - Equational Form and Fixpoint Computation:
 *   Lays the foundation for implementing an abstract fixpoint engine, supporting
 *   bounded and unbounded loop constructs.
 * 
 * =============================================================================
 * Example Workflow:
 * =============================================================================
 * 
 * Given the following program:
 * 
 * ```
 * int a;
 * int b;
 * void main() {
 *     // !npk b between 0 and 1 
 *     if (b == 0) {
 *         a = 1;
 *     } else {
 *         a = 2;
 *     }
 *     assert(a <= 2);
 *     assert(a >= 1);
 * }
 * 
 * 
 * The abstract interpreter performs the following steps:
 * 
 * 1. Initialize the interval store to represent all variables as unconstrained (⊤).
 * 2. Narrow the interval of `b` based on the precondition (`b ∈ [0, 1]`).
 * 3. Split execution into two branches (`b == 0` and `b != 0`) and evaluate them separately.
 * 4. Compute the interval for `a` in each branch (`a ∈ [1, 1]` and `a ∈ [2, 2]`).
 * 5. Join the intervals from both branches (`a ∈ [1, 2]`).
 * 6. Verify the assertions against the computed interval for `a`.
 * 
 * =============================================================================
 * Author: Parsa Vares
 * =============================================================================
 */

std::shared_ptr<semantics::BinExpression> 
FancyAnalyzer::buildBinaryExpression(const ASTNode &node, std::function<void(std::string)> warnFunc) {
    using namespace semantics;

    // Extract the operator type (e.g., +, -, *, /) from the AST node's value.
    BinOp op = std::get<BinOp>(node.value);

    // Access the left and right child nodes of the AST node.
    ASTNode leftNode = node.children[0];
    ASTNode rightNode = node.children[1];

    // Initialize shared pointers to hold the left and right expressions.
    std::shared_ptr<Expr> leftExpr = nullptr;
    std::shared_ptr<Expr> rightExpr = nullptr;

    // Handle the left operand:
    if (leftNode.type == NodeType::INTEGER) {
        // If the left operand is an integer literal, create a `Literal` expression.
        leftExpr = std::make_shared<Literal>(std::get<int>(leftNode.value));
    } 
    else if (leftNode.type == NodeType::VARIABLE) {
        // If the left operand is a variable, create a `VarExpression`.
        leftExpr = std::make_shared<VarExpression>(std::get<std::string>(leftNode.value));
    } 
    else if (leftNode.type == NodeType::ARITHM_OP) {
        // If the left operand is another arithmetic operation, recursively build its expression.
        leftExpr = buildBinaryExpression(leftNode, warnFunc);
    }

    // Handle the right operand:
    if (rightNode.type == NodeType::INTEGER) {
        // If the right operand is an integer literal, create a `Literal` expression.
        rightExpr = std::make_shared<Literal>(std::get<int>(rightNode.value));
    } 
    else if (rightNode.type == NodeType::VARIABLE) {
        // If the right operand is a variable, create a `VarExpression`.
        rightExpr = std::make_shared<VarExpression>(std::get<std::string>(rightNode.value));
    } 
    else if (rightNode.type == NodeType::ARITHM_OP) {
        // If the right operand is another arithmetic operation, recursively build its expression.
        rightExpr = buildBinaryExpression(rightNode, warnFunc);
    }

    // Ensure that both the left and right expressions are successfully constructed.
    assert(leftExpr && rightExpr && "Both left and right expressions must be valid.");

    // Log the creation of the binary expression.
    std::cout 
        << "[FancyAnalyzer] Constructing a Binary Expression with operator `" 
        << op << "`." << std::endl;

    // Return a shared pointer to the constructed `BinExpression` object.
    return std::make_shared<BinExpression>(op, leftExpr, rightExpr, warnFunc);
}

/**
 * Recursively builds the solver system from the AST nodes.
 */
void FancyAnalyzer::constructSolverSystem(const ASTNode &node) {
    using namespace semantics;

    // ----------------------------
    // Handling Variable Declarations
    // ----------------------------
    if (node.type == NodeType::DECLARATION) {
        // Log the discovery of a declaration node.
        std::cout 
            << "[FancyAnalyzer] Discovered a Declaration node. Creating a VarAllocator." 
            << std::endl;

        // Extract the variable node from the children of the declaration node.
        ASTNode varNode = node.children[0];
        
        // Ensure that the child node is indeed a variable.
        assert(varNode.type == NodeType::VARIABLE && "Expected VARIABLE node in DECLARATION.");

        // Retrieve the variable name from the AST node.
        std::string varName = std::get<std::string>(varNode.value);

        // Instantiate a VarAllocator semantic action for the declared variable.
        auto declCmd = std::make_shared<VarAllocator>(varName, solverActions.size());
        
        // Add the VarAllocator to the list of solver actions.
        solverActions.push_back(std::move(declCmd));
    }

    // ----------------------------
    // Handling Assignments
    // ----------------------------
    if (node.type == NodeType::ASSIGNMENT) {
        // Log the encounter of an assignment node.
        std::cout 
            << "[FancyAnalyzer] Encountered an Assignment node. Processing the assigned value..." 
            << std::endl;

        // Extract the target variable name from the assignment node.
        std::string varName = std::get<std::string>(node.children[0].value);
        
        // Extract the expression node representing the value to be assigned.
        ASTNode exprNode = node.children[1];

        // ----------------------------
        // Assignment to an Integer Literal
        // ----------------------------
        if (exprNode.type == NodeType::INTEGER) {
            // Retrieve the integer value from the AST node.
            int numericVal = std::get<int>(exprNode.value);
            
            // Log the assignment of an integer value to the variable.
            std::cout 
                << "    [Info] Assigning integer " << numericVal 
                << " to variable `" << varName << "`." 
                << std::endl;

            // Create a Literal expression representing the integer value.
            auto literalExpr = std::make_shared<Literal>(numericVal);
            
            // Instantiate a SetOperator semantic action to assign the literal to the variable.
            auto setOp = std::make_shared<SetOperator>(varName, literalExpr, solverActions.size());
            
            // Add the SetOperator to the list of solver actions.
            solverActions.push_back(std::move(setOp));
        }
        // ----------------------------
        // Assignment from Another Variable
        // ----------------------------
        else if (exprNode.type == NodeType::VARIABLE) {
            // Retrieve the source variable name from which the value is mirrored.
            std::string fromVar = std::get<std::string>(exprNode.value);
            
            // Log the assignment of one variable's value to another.
            std::cout 
                << "    [Info] Variable `" << varName 
                << "` now mirrors the value of `" << fromVar << "`." 
                << std::endl;

            // Create a VarExpression representing the source variable.
            auto varExpr = std::make_shared<VarExpression>(fromVar);
            
            // Instantiate a SetOperator semantic action to assign the VarExpression to the target variable.
            auto setOp = std::make_shared<SetOperator>(varName, varExpr, solverActions.size());
            
            // Add the SetOperator to the list of solver actions.
            solverActions.push_back(std::move(setOp));
        }
        // ----------------------------
        // Assignment from an Arithmetic Expression
        // ----------------------------
        else if (exprNode.type == NodeType::ARITHM_OP) {
            // Log the processing of an arithmetic expression in the assignment.
            std::cout 
                << "    [Info] The right-hand side is an arithmetic expression. Generating BinExpression..."
                << std::endl;

            // Define a lambda function to handle warnings during expression construction.
            auto logWarning = [this, exprNode](std::string warningMsg) {
                std::cout << "    [Warning] Logging a caution: " << warningMsg << std::endl;
                this->warnings[exprNode.id] = warningMsg;
            };

            // Recursively build the binary expression from the arithmetic operation node.
            auto binExpr = buildBinaryExpression(exprNode, logWarning);
            
            // Instantiate a SetOperator semantic action to assign the BinExpression to the target variable.
            auto setOp = std::make_shared<SetOperator>(varName, binExpr, solverActions.size());
            
            // Add the SetOperator to the list of solver actions.
            solverActions.push_back(std::move(setOp));
        }
    }
    // ----------------------------
    // Handling Pre-Conditions (PreCon)
    // ----------------------------
    else if (node.type == NodeType::PRE_CON) {
        // Log the discovery of a pre-condition node.
        std::cout 
            << "[FancyAnalyzer] Found a PreConstraint node. We'll interpret it as narrowing the variable range." 
            << std::endl;

        // Extract the left and right side of the pre-condition (e.g., >= and <= operations).
        ASTNode leftSide  = node.children[0]; // Typically represents a lower bound (e.g., b >= 0)
        ASTNode rightSide = node.children[1]; // Typically represents an upper bound (e.g., b <= 1)

        // Ensure that both sides are logical operations.
        assert(leftSide.type == NodeType::LOGIC_OP && "Expected LOGIC_OP node for left side of PreCon.");
        assert(rightSide.type == NodeType::LOGIC_OP && "Expected LOGIC_OP node for right side of PreCon.");

        // Ensure that the operations are GEQ (>=) and LEQ (<=) respectively.
        assert(std::get<LogicOp>(leftSide.value) == LogicOp::GEQ && "Expected GEQ operator in PreCon left side.");
        assert(std::get<LogicOp>(rightSide.value) == LogicOp::LEQ && "Expected LEQ operator in PreCon right side.");

        // Extract the bounds and target variable from both sides.
        ASTNode lLow    = leftSide.children[0]; // Lower bound value (e.g., 0)
        ASTNode lVar    = leftSide.children[1]; // Variable being constrained (e.g., b)
        ASTNode rHigh   = rightSide.children[0]; // Upper bound value (e.g., 1)
        ASTNode rVar    = rightSide.children[1]; // Variable being constrained (e.g., b)
        
        // Validate the types of the extracted nodes.
        assert(lLow.type == NodeType::INTEGER && "Expected INTEGER node for lower bound in PreCon.");
        assert(lVar.type == NodeType::VARIABLE && "Expected VARIABLE node for variable in PreCon.");
        assert(rHigh.type == NodeType::INTEGER && "Expected INTEGER node for upper bound in PreCon.");
        assert(rVar.type == NodeType::VARIABLE && "Expected VARIABLE node for variable in PreCon.");

        // Retrieve the numerical bounds and variable name.
        int lowerBound = std::get<int>(lLow.value);
        int upperBound = std::get<int>(rHigh.value);
        std::string targetVar = std::get<std::string>(lVar.value);

        // Create a Literal expression representing the narrowed range [lowerBound, upperBound].
        auto rangeExpr = std::make_shared<Literal>(lowerBound, upperBound);
        
        // Instantiate a SetOperator semantic action to narrow the variable's range.
        auto narrowCmd = std::make_shared<SetOperator>(targetVar, rangeExpr, solverActions.size());
        
        // Add the SetOperator to the list of solver actions.
        solverActions.push_back(std::move(narrowCmd));
    }
    // ----------------------------
    // Handling Post-Conditions (PostCon) - Assertions
    // ----------------------------
    else if (node.type == NodeType::POST_CON) {
        // Log the encounter of a post-condition (assertion) node.
        std::cout 
            << "[FancyAnalyzer] Encountered a PostConstraint (assertion) node." 
            << std::endl;

        // Extract the logical expression inside the assertion.
        ASTNode logicNode = node.children[0];
        
        // Ensure that the logical expression is a logical operation.
        assert(logicNode.type == NodeType::LOGIC_OP && "Expected LOGIC_OP node inside PostCon.");

        // Retrieve the logical operator (e.g., <=, >=, ==) from the logic node.
        LogicOp conditionOp = std::get<LogicOp>(logicNode.value);
        
        // Extract the left and right operands of the logical operation.
        ASTNode leftNode = logicNode.children[0];
        ASTNode rightNode = logicNode.children[1];

        // Define a lambda function to handle and log warnings related to the assertion.
        auto pushWarning = [this, node](std::string w) {
            std::cout << "    [AssertionCheck] Recording an issue: " << w << std::endl;
            this->warnings[node.id] = w;
        };

        // Initialize shared pointers for the left and right expressions of the assertion.
        std::shared_ptr<Expr> leftExpr = nullptr;
        std::shared_ptr<Expr> rightExpr = nullptr;

        // Construct the left-hand side expression based on its type.
        if (leftNode.type == NodeType::VARIABLE) {
            leftExpr = std::make_shared<VarExpression>(std::get<std::string>(leftNode.value));
        }
        else if (leftNode.type == NodeType::INTEGER) {
            leftExpr = std::make_shared<Literal>(std::get<int>(leftNode.value));
        }
        else if (leftNode.type == NodeType::ARITHM_OP) {
            leftExpr = buildBinaryExpression(leftNode, pushWarning);
        }

        // Construct the right-hand side expression based on its type.
        if (rightNode.type == NodeType::VARIABLE) {
            rightExpr = std::make_shared<VarExpression>(std::get<std::string>(rightNode.value));
        }
        else if (rightNode.type == NodeType::INTEGER) {
            rightExpr = std::make_shared<Literal>(std::get<int>(rightNode.value));
        }
        else if (rightNode.type == NodeType::ARITHM_OP) {
            rightExpr = buildBinaryExpression(rightNode, pushWarning);
        }

        // Create a BoolExpr representing the logical assertion (e.g., a <= 2).
        auto boolExpr = std::make_shared<BoolExpr>(conditionOp, leftExpr, rightExpr);
        
        // Instantiate a CheckOperator semantic action to evaluate the assertion.
        auto assertCmd = std::make_shared<CheckOperator>(boolExpr, pushWarning, solverActions.size());
        
        // Add the CheckOperator to the list of solver actions.
        solverActions.push_back(std::move(assertCmd));
    }
    // ----------------------------
    // Handling If-Then-Else Constructs
    // ----------------------------
    else if (node.type == NodeType::IFELSE) {
        // Ensure the node represents an If-Then-Else structure.
        if (std::get<std::string>(node.value) == "IfElse") {
            // Log the discovery of an If-Then-Else structure.
            std::cout 
                << "[FancyAnalyzer] Discovered an If-Then-Else structure. Splitting into separate paths..." 
                << std::endl;

            // Extract the condition node and the bodies of the 'if' and 'else' branches.
            ASTNode condNode = node.children[0]; 
            ASTNode ifBody   = node.children[1];
            ASTNode logic    = condNode.children[0];

            // Ensure that the condition node contains a logical operation.
            assert(logic.type == NodeType::LOGIC_OP && "Expected LOGIC_OP node in If condition.");
            assert(logic.children[0].type == NodeType::VARIABLE && "Expected VARIABLE node in If condition's left operand.");

            // Retrieve the logical operator and variable name from the condition.
            LogicOp condLogic = std::get<LogicOp>(logic.value);
            std::string lVar  = std::get<std::string>(logic.children[0].value);
            
            // Ensure that the right operand of the condition is an integer.
            assert(logic.children[1].type == NodeType::INTEGER && "Expected INTEGER node in If condition's right operand.");

            // Retrieve the condition value from the AST node.
            auto ifVal    = std::make_shared<Literal>(std::get<int>(logic.children[1].value));
            auto elseVal  = std::make_shared<Literal>(std::get<int>(logic.children[1].value));

            // Record the current location before branching.
            size_t preIfLoc = solverActions.size();

            // Instantiate a RestrictOperator to apply the condition for the 'if' branch.
            auto branchIf = std::make_shared<RestrictOperator>(condLogic, lVar, ifVal, preIfLoc);
            solverActions.push_back(branchIf);

            // Recursively process the 'if' branch body.
            for (const auto &child : ifBody.children) {
                constructSolverSystem(child);
            }

            // Record the location after processing the 'if' branch.
            size_t ifEndLoc = solverActions.size();
            std::cout 
                << "    [Info] Completed the If-branch at location " 
                << ifEndLoc << std::endl;

            // Check if an 'else' branch exists.
            if (node.children.size() == 3) {
                ASTNode elseBody = node.children[2];

                // Instantiate a RestrictOperator to apply the negated condition for the 'else' branch.
                auto branchElse = std::make_shared<RestrictOperator>(
                    get_opposite(condLogic), lVar, elseVal, preIfLoc, ifEndLoc + 1
                );
                solverActions.push_back(branchElse);

                // Recursively process the 'else' branch body.
                for (const auto &child : elseBody.children) {
                    constructSolverSystem(child);
                }

                // Record the location after processing the 'else' branch.
                size_t elseEndLoc = solverActions.size();
                std::cout 
                    << "    [Info] Completed the Else-branch at location " 
                    << elseEndLoc << std::endl;

                // Instantiate a MergeStates semantic action to unify the 'if' and 'else' branches.
                auto unifyBoth = std::make_shared<MergeStates>(
                    elseEndLoc + 1, std::vector<size_t>{ifEndLoc, elseEndLoc}
                );
                solverActions.push_back(unifyBoth);
            }
            else {
                // If there is no 'else' branch, merge the 'if' branch with the subsequent location.
                auto unifyIf = std::make_shared<MergeStates>(
                    ifEndLoc + 1, std::vector<size_t>{preIfLoc, ifEndLoc}
                );
                solverActions.push_back(unifyIf);
            }
            return;
        }
    }
    // ----------------------------
    // Handling While Loops
    // ----------------------------
    else if (node.type == NodeType::WHILELOOP) {
        // Ensure the node represents a While-Loop structure.
        if (std::get<std::string>(node.value) == "WhileLoop") {
            // Log the recognition of a while-loop.
            std::cout 
                << "[FancyAnalyzer] Recognized a While-Loop. We'll handle the fixpoint for repeated iteration." 
                << std::endl;

            // Extract the condition node and the body of the loop.
            ASTNode condNode = node.children[0];
            ASTNode body     = node.children[1];

            // Extract the logical operation from the condition node.
            ASTNode logicNode = condNode.children[0];
            assert(logicNode.type == NodeType::LOGIC_OP && "Expected LOGIC_OP node in While condition.");
            assert(logicNode.children[0].type == NodeType::VARIABLE && "Expected VARIABLE node in While condition's left operand.");

            // Retrieve the logical operator and variable name from the condition.
            LogicOp condOp = std::get<LogicOp>(logicNode.value);
            std::string varName = std::get<std::string>(logicNode.children[0].value);
            
            // Ensure that the right operand of the condition is an integer.
            assert(logicNode.children[1].type == NodeType::INTEGER && "Expected INTEGER node in While condition's right operand.");

            // Retrieve the condition value from the AST node.
            auto condValue = std::make_shared<Literal>(std::get<int>(logicNode.children[1].value));

            // Record the current location before entering the loop.
            size_t beforeLoop = solverActions.size();
            
            // Insert a placeholder for the merge action, which will be updated later.
            solverActions.push_back(nullptr); // placeholder
            
            // Record the location of the RestrictOperator for the loop condition.
            size_t joinCP = solverActions.size();

            // Instantiate a RestrictOperator to apply the loop condition.
            auto filterLoop = std::make_shared<RestrictOperator>(condOp, varName, condValue, joinCP);
            solverActions.push_back(filterLoop);

            // Recursively process the loop body.
            for (const auto &child : body.children) {
                constructSolverSystem(child);
            }

            // Record the location after processing the loop body.
            size_t bodyEnd = solverActions.size();
            std::cout 
                << "    [Info] Finished exploring the While-Loop body at location " 
                << bodyEnd << std::endl;

            // Update the placeholder with a MergeStates action to loop back.
            solverActions[joinCP - 1] = std::make_shared<MergeStates>(
                joinCP, std::vector<size_t>{beforeLoop, bodyEnd}
            );

            // Instantiate a RestrictOperator for the loop exit condition (negation of the loop condition).
            auto exitFilter = std::make_shared<RestrictOperator>(
                get_opposite(condOp), varName, condValue, joinCP, solverActions.size() + 1
            );
            solverActions.push_back(exitFilter);
            return;
        }
    }

    // ----------------------------
    // Handling Other Node Types (Recursion)
    // ----------------------------
    // If the node type does not match any of the above, recursively process its children.
    for (const auto &sub : node.children) {
        constructSolverSystem(sub);
    }
}

/**
 * Run a single step of the solver. If the newly produced invariants
 * match the old ones, we have reached a fixpoint.
 */
bool FancyAnalyzer::solverStep() {
    // Initialize a fresh collection of states, one for each solver action plus an initial state.
    StatesCollection nextStates(solverActions.size() + 1, VariablesState());
    
    // The first state is typically the entry point of the program, marked as "bottom" state.
    nextStates[0].markBottomState(true);

    // Iterate over all solver actions (commands) and execute them sequentially.
    // Each command updates `nextStates` based on the current states in `frames`.
    for (auto &cmd : solverActions) {
        cmd->execute(frames, nextStates);
    }

    // Check if the new states (`nextStates`) are identical to the current states (`frames`).
    // If they are equal, it indicates that the analysis has stabilized (fixpoint reached).
    bool stable = (frames == nextStates);

    // Move the newly computed states to `frames`, replacing the old states.
    frames = std::move(nextStates);

    // Return whether a fixpoint was reached.
    return stable;
}

/**
 * Orchestrates the iterative fixpoint computation for the solver
 * This method repeatedly applies solver steps until either a fixpoint is reached 
 * (i.e., states stabilize) or a predefined iteration limit is exceeded. The process 
 * integrates a widening mechanism to ensure termination, particularly for loops 
 * with unbounded iterations. 
 */
void FancyAnalyzer::solveSystem() {
    // Log the total number of solver actions for debugging purposes.
    std::cout 
        << "[FancyAnalyzer] We have " << solverActions.size() 
        << " solver actions total." << std::endl;

    // Initialize the state collection for the solver.
    // Each state corresponds to a program location; the first state is marked as the "bottom state."
    frames = StatesCollection(solverActions.size() + 1, VariablesState());
    frames[0].markBottomState(true);

    int iterationCount = 0;  // Track the number of iterations performed.
    bool isFixedPoint = false;  // Flag to determine if a fixpoint has been reached.

    // Define thresholds for widening and maximum iterations.
    const int WIDEN_THRESHOLD = 5;  // Number of iterations before applying widening.
    const int MAX_ITERS = 100;      // Maximum number of iterations to prevent infinite loops.

    // Main iteration loop for fixpoint computation.
    do {
        std::cout 
            << ">>> Iteration #" << iterationCount 
            << " begins..." << std::endl;

        // Preserve the current state frames in case widening is required.
        StatesCollection oldFrames = frames;

        // Perform a single step of the solver.
        isFixedPoint = solverStep();
        iterationCount++;

        // Apply widening if the fixpoint is not reached after the defined threshold.
        if (!isFixedPoint && iterationCount == WIDEN_THRESHOLD) {
            std::cout 
                << ">>> Widening the analysis at iteration #" 
                << iterationCount << "..." << std::endl;
            applyWidening(oldFrames, frames);  // Use old frames to adjust the new ones.
        }

        // Check for exceeding the maximum iteration limit and issue a warning if necessary.
        if (iterationCount > MAX_ITERS) {
            std::cout 
                << "!!! Warning: Exceeded " << MAX_ITERS 
                << " iterations without a fixpoint." 
                << std::endl;
            break;  // Exit the loop to avoid infinite computation.
        }
    } while (!isFixedPoint);  // Continue until a fixpoint is achieved.

    // Log the completion of the fixpoint computation and the total iterations taken.
    std::cout 
        << "[FancyAnalyzer] Fixed point reached after " 
        << iterationCount << " iterations." 
        << std::endl;
}

/**
 * Print the final invariants at each location after fixpoint is reached.
 */
void FancyAnalyzer::printSystemState() const {
    size_t locIndex = 0;  // Index for program locations.
    for (const auto &state : frames) {
        // Print the program location index.
        std::cout << "\n=== Program Location " << locIndex << " ===" << std::endl;
        // Display the state, including variable-to-interval mappings.
        state.display(); 
        locIndex++;  // Move to the next location.
    }
    std::cout << std::endl;  // Final newline for formatting.
}


/**
 * Print all warnings and errors that were encountered (e.g. potential division by zero, assertion failures).
 */
void FancyAnalyzer::printSystemNotes() const {
    std::cout << "\n=== Summary of Warnings/Errors ===" << std::endl;

    // Check if there are any recorded warnings or errors.
    if (warnings.empty()) {
        // If no warnings exist, notify the user.
        std::cout << "No warnings or errors recorded." << std::endl;
    } else {
        // Print each warning, associating it with the relevant AST node.
        for (const auto &[nodeID, message] : warnings) {
            std::cout 
                << "Node ID " << nodeID 
                << " => " << message << std::endl;
        }
    }

    std::cout << std::endl;  // Final newline for formatting.
}


/**
 * A naive widening procedure: for each location i, 
 * the new frames[i] is "widened" with the old frames[i].
 */
void FancyAnalyzer::applyWidening(const StatesCollection &oldFrames, StatesCollection &newFrames) {
    std::cout << "[Widening] Attempting to accelerate convergence..." << std::endl;

    // Iterate over each program location and apply the widening operation.
    for (size_t i = 0; i < newFrames.size(); i++) {
        newFrames[i] = newFrames[i].widenWith(oldFrames[i]);  // Update with widened state.
    }

    std::cout << "[Widening] Done." << std::endl;  // Indicate completion of the widening process.
}
