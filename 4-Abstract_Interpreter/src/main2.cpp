#include <iostream>
#include <fstream>
#include <sstream>
#include <climits> // For INT32_MIN and INT32_MAX
#include <unordered_map>
#include <map>
#include <set>
#include <queue>

#include "parser.hpp"
#include "ast.hpp"

// (Optional) We reuse or replicate the old checks for demonstration:
void checkDivisionByZero(const ASTNode& node) {
    if (node.type == NodeType::ARITHM_OP && std::get<BinOp>(node.value) == BinOp::DIV) {
        const auto& denominator = node.children[1];
        if (denominator.type == NodeType::INTEGER && std::get<int>(denominator.value) == 0) {
            std::cerr << "Warning: Division by zero detected!" << std::endl;
        }
    }
    for (const auto& child : node.children) {
        checkDivisionByZero(child);
    }
}

void checkOverflow(const ASTNode& node) {
    if (node.type == NodeType::ARITHM_OP) {
        const auto& op = std::get<BinOp>(node.value);
        const auto& left = node.children[0];
        const auto& right = node.children[1];

        if (left.type == NodeType::INTEGER && right.type == NodeType::INTEGER) {
            int leftVal = std::get<int>(left.value);
            int rightVal = std::get<int>(right.value);

            // Overflow checks
            if (op == BinOp::ADD && ((leftVal > 0 && rightVal > INT32_MAX - leftVal) ||
                                     (leftVal < 0 && rightVal < INT32_MIN - leftVal))) {
                std::cerr << "Warning: Addition overflow detected!" << std::endl;
            }
            if (op == BinOp::SUB && ((leftVal > 0 && rightVal < INT32_MIN + leftVal) ||
                                     (leftVal < 0 && rightVal > INT32_MAX + leftVal))) {
                std::cerr << "Warning: Subtraction overflow detected!" << std::endl;
            }
            if (op == BinOp::MUL && (leftVal != 0 && (rightVal > INT32_MAX / leftVal ||
                                                      rightVal < INT32_MIN / leftVal))) {
                std::cerr << "Warning: Multiplication overflow detected!" << std::endl;
            }
        }
    }
    for (const auto& child : node.children) {
        checkOverflow(child);
    }
}

// ====================================================================
// Equational Form / Fixpoint Code
// ====================================================================

// We'll store intervals at each "location" (statement index).
static std::unordered_map<int, std::map<std::string, Interval>> locationStores;
static std::set<std::string> allVariables;

// Helper struct to pair a location ID with the AST node at that location.
struct LabeledNode {
    int locationID;
    ASTNode node;
};

// Collect all variables from the AST
void collectAllVariables(const ASTNode& node) {
    if (node.type == NodeType::VARIABLE) {
        allVariables.insert(std::get<std::string>(node.value));
    }
    if (node.type == NodeType::DECLARATION) {
        for (auto &child : node.children) {
            if (child.type == NodeType::VARIABLE) {
                allVariables.insert(std::get<std::string>(child.value));
            }
        }
    }
    for (auto &c : node.children) {
        collectAllVariables(c);
    }
}

// Linearize the AST (naive approach: each top-level statement is a location).
void linearizeAST(const ASTNode& root, std::vector<LabeledNode>& labeledNodes) {
    static int currentID = 0;  // ensure unique IDs if we call multiple times
    if (root.type == NodeType::SEQUENCE) {
        for (auto &child : root.children) {
            LabeledNode ln{currentID++, child};
            labeledNodes.push_back(ln);
        }
    } else {
        // If not a sequence, treat the entire root as one location
        LabeledNode ln{currentID++, root};
        labeledNodes.push_back(ln);
    }
}

// Evaluate expression => Interval
Interval evaluateExpression(const ASTNode& expr, const std::map<std::string, Interval>& store) {
    switch (expr.type) {
        case NodeType::INTEGER: {
            int val = std::get<int>(expr.value);
            return Interval(val, val);
        }
        case NodeType::VARIABLE: {
            std::string varName = std::get<std::string>(expr.value);
            if (store.find(varName) != store.end()) {
                return store.at(varName);
            } else {
                return Interval(INT32_MIN, INT32_MAX); // top
            }
        }
        case NodeType::ARITHM_OP: {
            Interval leftI = evaluateExpression(expr.children[0], store);
            Interval rightI = evaluateExpression(expr.children[1], store);
            BinOp op = std::get<BinOp>(expr.value);
            try {
                if (op == BinOp::ADD) return leftI + rightI;
                if (op == BinOp::SUB) return leftI - rightI;
                if (op == BinOp::MUL) return leftI * rightI;
                if (op == BinOp::DIV) return leftI / rightI; // might throw
            } catch (std::runtime_error&) {
                // If division by zero => return top
                return Interval(INT32_MIN, INT32_MAX);
            }
        }
        default:
            return Interval(INT32_MIN, INT32_MAX);
    }
}

// Transfer function
std::map<std::string, Interval>
transferFunction(const ASTNode& stmt, const std::map<std::string, Interval>& inStore) {
    std::map<std::string, Interval> outStore = inStore; // copy inStore

    switch (stmt.type) {
    case NodeType::DECLARATION: {
        // e.g. int x, ...
        // For each variable child, set to top (or any default)
        for (auto &child : stmt.children) {
            if (child.type == NodeType::VARIABLE) {
                outStore[std::get<std::string>(child.value)] = Interval(INT32_MIN, INT32_MAX);
            }
        }
        break;
    }
    case NodeType::ASSIGNMENT: {
        // x = expr
        ASTNode varNode = stmt.children[0];
        ASTNode exprNode = stmt.children[1];
        std::string varName = std::get<std::string>(varNode.value);
        Interval valI = evaluateExpression(exprNode, inStore);
        outStore[varName] = valI;
        break;
    }
    case NodeType::SEQUENCE: {
        // Evaluate each child in order
        std::map<std::string, Interval> temp = inStore;
        for (auto &child : stmt.children) {
            temp = transferFunction(child, temp);
        }
        outStore = temp;
        break;
    }
    default:
        // For if/else, while, etc. you’d do merges or loops
        break;
    }

    return outStore;
}

// Interval join
Interval joinInterval(const Interval& a, const Interval& b) {
    return Interval(std::min(a.lower, b.lower), std::max(a.upper, b.upper));
}

// Store join
std::map<std::string, Interval>
joinStores(const std::map<std::string, Interval>& s1,
           const std::map<std::string, Interval>& s2) {
    std::map<std::string, Interval> result;
    std::set<std::string> vars;
    for (auto &kv : s1) vars.insert(kv.first);
    for (auto &kv : s2) vars.insert(kv.first);

    for (auto &v : vars) {
        auto it1 = s1.find(v);
        auto it2 = s2.find(v);
        if (it1 != s1.end() && it2 != s2.end()) {
            result[v] = joinInterval(it1->second, it2->second);
        } else if (it1 != s1.end()) {
            result[v] = it1->second;
        } else {
            result[v] = it2->second;
        }
    }
    return result;
}

// Fixpoint iteration
void fixpointComputation(const std::vector<LabeledNode>& labeledNodes) {
    bool changed = true;
    while (changed) {
        changed = false;
        for (size_t i = 0; i < labeledNodes.size(); i++) {
            int loc = labeledNodes[i].locationID;
            auto inStore = locationStores[loc];
            // Transfer
            auto outStore = transferFunction(labeledNodes[i].node, inStore);

            // If there's a "next" statement, unify outStore with nextLoc's input
            if (i + 1 < labeledNodes.size()) {
                int nextLoc = labeledNodes[i + 1].locationID;
                auto oldNext = locationStores[nextLoc];
                auto newNext = joinStores(oldNext, outStore);
                if (newNext != oldNext) {
                    locationStores[nextLoc] = newNext;
                    changed = true;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "usage: " << argv[0] << " <some_test_file.c>\n";
        return 1;
    }

    // 1. Load input file
    std::ifstream f(argv[1]);
    if (!f.is_open()) {
        std::cerr << "[ERROR] Cannot open file " << argv[1] << std::endl;
        return 1;
    }
    std::ostringstream buf;
    buf << f.rdbuf();
    std::string input = buf.str();
    f.close();

    // 2. Parse
    std::cout << "Parsing program `" << argv[1] << "`...\n";
    AbstractInterpreterParser parser;
    ASTNode ast = parser.parse(input);
    ast.print();

    // 3. (Optional) check for zero-division / overflow
    checkDivisionByZero(ast);
    checkOverflow(ast);

    // 4. Collect all variables
    allVariables.clear();
    collectAllVariables(ast);

    // 5. Linearize the AST
    std::vector<LabeledNode> labeledNodes;
    labeledNodes.clear();
    linearizeAST(ast, labeledNodes);

    // If no labeled nodes, wrap entire AST in one label
    if (labeledNodes.empty()) {
        labeledNodes.push_back({0, ast});
    }

    // 6. Initialize locationStores (top intervals)
    locationStores.clear();
    for (auto &ln : labeledNodes) {
        int loc = ln.locationID;
        std::map<std::string, Interval> topStore;
        for (auto &v : allVariables) {
            topStore[v] = Interval(INT32_MIN, INT32_MAX); 
        }
        locationStores[loc] = topStore;
    }

    // 7. Fixpoint computation
    fixpointComputation(labeledNodes);

    // 8. Print final results
    std::cout << "\n=== Final Interval Stores (Equational Form) ===\n";
    for (auto &ln : labeledNodes) {
        int loc = ln.locationID;
        std::cout << "Location " << loc << ":\n";
        for (auto &kv : locationStores[loc]) {
            std::cout << "   " << kv.first << " in " << kv.second << "\n";
        }
    }
    std::cout << "==============================================\n\n";

    return 0;
}
