#include "abstract_interpreter.hpp"

/**
 * Builds a BinExpression object from an AST node containing an arithmetic operation.
 * We'll produce a more descriptive message in the logs.
 */
std::shared_ptr<semantics::BinExpression> 
FancyAnalyzer::buildBinaryExpression(const ASTNode &node, std::function<void(std::string)> warnFunc) {
    using namespace semantics;

    // Identify the binop from the AST
    BinOp op = std::get<BinOp>(node.value);

    // Extract left and right child nodes
    ASTNode leftNode = node.children[0];
    ASTNode rightNode = node.children[1];

    // Convert them into expressions
    std::shared_ptr<Expr> leftExpr = nullptr;
    if (leftNode.type == NodeType::INTEGER) {
        leftExpr = std::make_shared<Literal>(std::get<int>(leftNode.value));
    } 
    else if (leftNode.type == NodeType::VARIABLE) {
        leftExpr = std::make_shared<VarExpression>(std::get<std::string>(leftNode.value));
    }
    else if (leftNode.type == NodeType::ARITHM_OP) {
        leftExpr = buildBinaryExpression(leftNode, warnFunc);
    }

    std::shared_ptr<Expr> rightExpr = nullptr;
    if (rightNode.type == NodeType::INTEGER) {
        rightExpr = std::make_shared<Literal>(std::get<int>(rightNode.value));
    } 
    else if (rightNode.type == NodeType::VARIABLE) {
        rightExpr = std::make_shared<VarExpression>(std::get<std::string>(rightNode.value));
    }
    else if (rightNode.type == NodeType::ARITHM_OP) {
        rightExpr = buildBinaryExpression(rightNode, warnFunc);
    }

    assert(leftExpr && rightExpr);

    std::cout 
        << "    [FancyAnalyzer] Constructing a BinExpression with operator `" 
        << op << "`." << std::endl;

    return std::make_shared<BinExpression>(op, leftExpr, rightExpr, warnFunc);
}

/**
 * Recursively builds the solver system from the AST nodes.
 */
void FancyAnalyzer::constructSolverSystem(const ASTNode &node) {
    using namespace semantics;

    // Handle each node type
    if (node.type == NodeType::DECLARATION) {
        std::cout 
            << "[FancyAnalyzer] Discovered a Declaration node. Creating a VarAllocator." 
            << std::endl;

        ASTNode varNode = node.children[0];
        assert(varNode.type == NodeType::VARIABLE);
        std::string varName = std::get<std::string>(varNode.value);

        auto declCmd = std::make_shared<VarAllocator>(varName, solverActions.size());
        solverActions.push_back(std::move(declCmd));
    }

    if (node.type == NodeType::ASSIGNMENT) {
        std::cout 
            << "[FancyAnalyzer] Encountered an Assignment node. Processing the assigned value..." 
            << std::endl;

        std::string varName = std::get<std::string>(node.children[0].value);
        ASTNode exprNode = node.children[1];

        if (exprNode.type == NodeType::INTEGER) {
            int numericVal = std::get<int>(exprNode.value);
            std::cout 
                << "    [Info] Assigning integer " << numericVal 
                << " to variable `" << varName << "`." 
                << std::endl;

            auto literalExpr = std::make_shared<Literal>(numericVal);
            auto setOp = std::make_shared<SetOperator>(varName, literalExpr, solverActions.size());
            solverActions.push_back(std::move(setOp));
        }
        else if (exprNode.type == NodeType::VARIABLE) {
            std::string fromVar = std::get<std::string>(exprNode.value);
            std::cout 
                << "    [Info] Variable `" << varName 
                << "` now mirrors the value of `" << fromVar << "`." 
                << std::endl;

            auto varExpr = std::make_shared<VarExpression>(fromVar);
            auto setOp = std::make_shared<SetOperator>(varName, varExpr, solverActions.size());
            solverActions.push_back(std::move(setOp));
        }
        else if (exprNode.type == NodeType::ARITHM_OP) {
            std::cout 
                << "    [Info] The right-hand side is an arithmetic expression. Generating BinExpression..."
                << std::endl;

            auto logWarning = [this, exprNode](std::string warningMsg) {
                std::cout << "    [Warning] Logging a caution: " << warningMsg << std::endl;
                this->warnings[exprNode.id] = warningMsg;
            };

            auto binExpr = buildBinaryExpression(exprNode, logWarning);
            auto setOp = std::make_shared<SetOperator>(varName, binExpr, solverActions.size());
            solverActions.push_back(std::move(setOp));
        }
    }
    else if (node.type == NodeType::PRE_CON) {
        std::cout 
            << "[FancyAnalyzer] Found a PreConstraint node. We'll interpret it as narrowing the variable range." 
            << std::endl;

        ASTNode leftSide  = node.children[0]; // >=
        ASTNode rightSide = node.children[1]; // <=
        assert(leftSide.type == NodeType::LOGIC_OP);
        assert(rightSide.type == NodeType::LOGIC_OP);
        assert(std::get<LogicOp>(leftSide.value) == LogicOp::GEQ);
        assert(std::get<LogicOp>(rightSide.value) == LogicOp::LEQ);

        ASTNode lLow    = leftSide.children[0];
        ASTNode lVar    = leftSide.children[1];
        ASTNode rHigh   = rightSide.children[0];
        ASTNode rVar    = rightSide.children[1];
        
        assert(lLow.type == NodeType::INTEGER);
        assert(lVar.type == NodeType::VARIABLE);
        assert(rHigh.type == NodeType::INTEGER);
        assert(rVar.type == NodeType::VARIABLE);

        int lowerBound = std::get<int>(lLow.value);
        int upperBound = std::get<int>(rHigh.value);
        std::string targetVar = std::get<std::string>(lVar.value);

        auto rangeExpr = std::make_shared<Literal>(lowerBound, upperBound);
        auto narrowCmd = std::make_shared<SetOperator>(targetVar, rangeExpr, solverActions.size());
        solverActions.push_back(std::move(narrowCmd));
    }
    else if (node.type == NodeType::POST_CON) {
        std::cout 
            << "[FancyAnalyzer] Encountered a PostConstraint (assertion) node." 
            << std::endl;

        ASTNode logicNode = node.children[0];
        assert(logicNode.type == NodeType::LOGIC_OP);

        LogicOp conditionOp = std::get<LogicOp>(logicNode.value);
        ASTNode leftNode = logicNode.children[0];
        ASTNode rightNode = logicNode.children[1];

        auto pushWarning = [this, node](std::string w) {
            std::cout << "    [AssertionCheck] Recording an issue: " << w << std::endl;
            this->warnings[node.id] = w;
        };

        std::shared_ptr<Expr> leftExpr = nullptr;
        std::shared_ptr<Expr> rightExpr = nullptr;

        if (leftNode.type == NodeType::VARIABLE) {
            leftExpr = std::make_shared<VarExpression>(std::get<std::string>(leftNode.value));
        }
        else if (leftNode.type == NodeType::INTEGER) {
            leftExpr = std::make_shared<Literal>(std::get<int>(leftNode.value));
        }
        else if (leftNode.type == NodeType::ARITHM_OP) {
            leftExpr = buildBinaryExpression(leftNode, pushWarning);
        }

        if (rightNode.type == NodeType::VARIABLE) {
            rightExpr = std::make_shared<VarExpression>(std::get<std::string>(rightNode.value));
        }
        else if (rightNode.type == NodeType::INTEGER) {
            rightExpr = std::make_shared<Literal>(std::get<int>(rightNode.value));
        }
        else if (rightNode.type == NodeType::ARITHM_OP) {
            rightExpr = buildBinaryExpression(rightNode, pushWarning);
        }

        auto boolExpr = std::make_shared<BoolExpr>(conditionOp, leftExpr, rightExpr);
        auto assertCmd = std::make_shared<CheckOperator>(boolExpr, pushWarning, solverActions.size());
        solverActions.push_back(std::move(assertCmd));
    }
    else if (node.type == NodeType::IFELSE) {
        if (std::get<std::string>(node.value) == "IfElse") {
            std::cout 
                << "[FancyAnalyzer] Discovered an If-Then-Else structure. Splitting into separate paths..." 
                << std::endl;

            ASTNode condNode = node.children[0]; 
            ASTNode ifBody   = node.children[1];
            ASTNode logic    = condNode.children[0];
            assert(logic.type == NodeType::LOGIC_OP);
            assert(logic.children[0].type == NodeType::VARIABLE);

            LogicOp condLogic = std::get<LogicOp>(logic.value);
            std::string lVar  = std::get<std::string>(logic.children[0].value);
            assert(logic.children[1].type == NodeType::INTEGER);

            auto ifVal    = std::make_shared<Literal>(std::get<int>(logic.children[1].value));
            auto elseVal  = std::make_shared<Literal>(std::get<int>(logic.children[1].value));

            size_t preIfLoc = solverActions.size();

            auto branchIf = std::make_shared<RestrictOperator>(condLogic, lVar, ifVal, preIfLoc);
            solverActions.push_back(branchIf);

            // Explore the if-body
            for (const auto &child : ifBody.children) {
                constructSolverSystem(child);
            }

            size_t ifEndLoc = solverActions.size();
            std::cout 
                << "    [Info] Completed the If-branch at location " 
                << ifEndLoc << std::endl;

            // Possibly else
            if (node.children.size() == 3) {
                ASTNode elseBody = node.children[2];

                auto branchElse = std::make_shared<RestrictOperator>(
                    get_opposite(condLogic), lVar, elseVal, preIfLoc, ifEndLoc + 1
                );
                solverActions.push_back(branchElse);

                // Explore the else-body
                for (const auto &child : elseBody.children) {
                    constructSolverSystem(child);
                }

                size_t elseEndLoc = solverActions.size();
                std::cout 
                    << "    [Info] Completed the Else-branch at location " 
                    << elseEndLoc << std::endl;

                auto unifyBoth = std::make_shared<MergeStates>(
                    elseEndLoc + 1, std::vector<size_t>{ifEndLoc, elseEndLoc}
                );
                solverActions.push_back(unifyBoth);
            }
            else {
                // No else branch
                auto unifyIf = std::make_shared<MergeStates>(
                    ifEndLoc + 1, std::vector<size_t>{preIfLoc, ifEndLoc}
                );
                solverActions.push_back(unifyIf);
            }
            return;
        }
    }
    else if (node.type == NodeType::WHILELOOP) {
        if (std::get<std::string>(node.value) == "WhileLoop") {
            std::cout 
                << "[FancyAnalyzer] Recognized a While-Loop. We'll handle the fixpoint for repeated iteration." 
                << std::endl;

            ASTNode condNode = node.children[0];
            ASTNode body     = node.children[1];

            ASTNode logicNode = condNode.children[0];
            assert(logicNode.type == NodeType::LOGIC_OP);
            assert(logicNode.children[0].type == NodeType::VARIABLE);

            LogicOp condOp = std::get<LogicOp>(logicNode.value);
            std::string varName = std::get<std::string>(logicNode.children[0].value);
            assert(logicNode.children[1].type == NodeType::INTEGER);

            auto condValue = std::make_shared<Literal>(std::get<int>(logicNode.children[1].value));

            size_t beforeLoop = solverActions.size();
            solverActions.push_back(nullptr); // placeholder
            size_t joinCP = solverActions.size();

            auto filterLoop = std::make_shared<RestrictOperator>(condOp, varName, condValue, joinCP);
            solverActions.push_back(filterLoop);

            // Evaluate the loop body
            for (const auto &child : body.children) {
                constructSolverSystem(child);
            }
            size_t bodyEnd = solverActions.size();
            std::cout 
                << "    [Info] Finished exploring the While-Loop body at location " 
                << bodyEnd << std::endl;

            // Fill the placeholder with a merge of beforeLoop & bodyEnd
            solverActions[joinCP - 1] = std::make_shared<MergeStates>(
                joinCP, std::vector<size_t>{beforeLoop, bodyEnd}
            );

            // The negation -> exit condition
            auto exitFilter = std::make_shared<RestrictOperator>(
                get_opposite(condOp), varName, condValue, joinCP, solverActions.size() + 1
            );
            solverActions.push_back(exitFilter);
            return;
        }
    }

    // Recur on children to ensure entire sub-AST is processed
    for (const auto &sub : node.children) {
        constructSolverSystem(sub);
    }
}

/**
 * Run a single step of the solver. If the newly produced invariants
 * match the old ones, we have reached a fixpoint.
 */
bool FancyAnalyzer::solverStep() {
    // Create a fresh set of states
    StatesCollection nextStates(solverActions.size() + 1, VariablesState());
    nextStates[0].markBottomState(true);

    // Invoke each solver action to update nextStates from frames
    for (auto &cmd : solverActions) {
        cmd->execute(frames, nextStates);
    }

    bool stable = (frames == nextStates);
    frames = std::move(nextStates);
    return stable;
}

/**
 * Orchestrate repeated solver steps until a fixpoint is reached
 * or the maximum iteration limit is exceeded.
 */
void FancyAnalyzer::solveSystem() {
    std::cout 
        << "[FancyAnalyzer] We have " << solverActions.size() 
        << " solver actions total." << std::endl;

    frames = StatesCollection(solverActions.size() + 1, VariablesState());
    frames[0].markBottomState(true);

    int iterationCount = 0;
    bool isFixedPoint  = false;

    // Threshold for applying a naive widening approach
    const int WIDEN_THRESHOLD = 5;
    const int MAX_ITERS       = 100;

    // Start the iteration loop
    do {
        std::cout 
            << ">>> Iteration #" << iterationCount 
            << " begins..." << std::endl;

        // Keep a copy in case we need to widen
        StatesCollection oldFrames = frames;

        isFixedPoint = solverStep();
        iterationCount++;

        // If not stable by WIDEN_THRESHOLD, we widen once
        if (!isFixedPoint && iterationCount == WIDEN_THRESHOLD) {
            std::cout 
                << ">>> Widening the analysis at iteration #" 
                << iterationCount << "..." << std::endl;
            applyWidening(oldFrames, frames);
        }

        if (iterationCount > MAX_ITERS) {
            std::cout 
                << "!!! Warning: Exceeded " << MAX_ITERS 
                << " iterations without a fixpoint." 
                << std::endl;
            break;
        }
    } while(!isFixedPoint);

    std::cout 
        << "[FancyAnalyzer] Fixed point reached after " 
        << iterationCount << " iterations." 
        << std::endl;
}

/**
 * Print the final invariants at each location after fixpoint is reached.
 */
void FancyAnalyzer::printSystemState() const {
    size_t locIndex = 0;
    for (const auto &state : frames) {
        std::cout << "\n=== Program Location " << locIndex << " ===" << std::endl;
        state.display(); // prints variable -> intervals
        locIndex++;
    }
    std::cout << std::endl;
}

/**
 * Print all warnings and errors that were encountered (e.g. potential division by zero, assertion failures).
 */
void FancyAnalyzer::printSystemNotes() const {
    std::cout << "\n=== Summary of Warnings/Errors ===" << std::endl;
    if (warnings.empty()) {
        std::cout << "No warnings or errors recorded." << std::endl;
    }
    else {
        for (const auto &[nodeID, message] : warnings) {
            std::cout 
                << "Node ID " << nodeID 
                << " => " << message << std::endl;
        }
    }
    std::cout << std::endl;
}

/**
 * A naive widening procedure: for each location i, 
 * the new frames[i] is "widened" with the old frames[i].
 */
void FancyAnalyzer::applyWidening(const StatesCollection &oldFrames, StatesCollection &newFrames) {
    std::cout << "[Widening] Attempting to accelerate convergence..." << std::endl;
    for (size_t i = 0; i < newFrames.size(); i++) {
        newFrames[i] = newFrames[i].widenWith(oldFrames[i]);
    }
    std::cout << "[Widening] Done." << std::endl;
}
