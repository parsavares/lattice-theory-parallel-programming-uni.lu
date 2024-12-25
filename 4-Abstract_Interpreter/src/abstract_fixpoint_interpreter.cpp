#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <climits>   // For INT_MIN, INT_MAX
#include <queue>
#include "ast.hpp"
#include "parser.hpp"

/* 
 * This interpreter implements an equational/CFG-based fixpoint for interval analysis.
 * 
 * Key points:
 * 1) Each AST node is assigned a location ID.
 * 2) We build edges so that statement N flows to statement N+1 (Sequence).
 * 3) We store:   In[loc]  -> IntervalMap
 *                Out[loc] -> IntervalMap
 * 4) We iterate until stable:
 *       In[loc]  = ⋁( Out[pred(loc)] )  // join over predecessors
 *       Out[loc] = transfer(node, In[loc])
 * 5) Pre-conditions refine intervals in the In-store or Out-store at that node.
 * 6) We do minimal if-else support here. (Real branching would create multiple successors.)
 * 7) We do not handle loops or widening (that's Exercise 4).
 */

// A map from variable name to an Interval
using IntervalMap = std::map<std::string, Interval>;

// Structure to track each node's ID, pointers, and stores.
struct NodeLocation {
    int id;
    const ASTNode* node;
    IntervalMap inStore;
    IntervalMap outStore;
};

class EquationalFormInterpreter {
public:
    EquationalFormInterpreter() : nextId(0) {}

    ASTNode parse(const std::string &input) {
        AbstractInterpreterParser parser;
        ASTNode root = parser.parse(input);
        return root;
    }

    void buildLocations(const ASTNode &root) {
        // Recursively assign a unique ID to every node in the AST
        assignLocation(root);

        // The root node’s children might collectively form a "sequence"
        // but in many small C programs, the root itself might just be a Sequence.
        // If `root.type == NodeType::SEQUENCE`, link the children in order:
        if (root.type == NodeType::SEQUENCE) {
            linkSequence(root);
        } else {
            // If the root isn't a sequence, treat it as a single node with no predecessor
            // We'll still link any internal sequences if they appear in children
            for (auto &child : root.children) {
                if (child.type == NodeType::SEQUENCE) {
                    linkSequence(child);
                } else {
                    // Possibly link child if it’s also a statement
                    // or we rely on deeper recursion in assignLocation. 
                }
            }
        }
    }

    void solveFixpoint() {
        bool changed = true;
        while (changed) {
            changed = false;

            // For each location in ascending ID order
            for (auto &loc : locations) {
                IntervalMap newIn = joinPredecessors(loc.id);
                IntervalMap newOut = transfer(*loc.node, newIn);

                // Compare with old inStore/outStore
                if (!equalIntervalMaps(newIn, loc.inStore)) {
                    loc.inStore = newIn;
                    changed = true;
                }
                if (!equalIntervalMaps(newOut, loc.outStore)) {
                    loc.outStore = newOut;
                    changed = true;
                }
            }
        }
    }

    void printResults() const {
        for (auto &loc : locations) {
            std::cout << "\nLocation " << loc.id << " (" << loc.node->type << "):\n";
            std::cout << "  In:  ";  printIntervalMap(loc.inStore);
            std::cout << "  Out: ";  printIntervalMap(loc.outStore);
        }
    }

private:
    int nextId;
    // Each NodeLocation holds node->id, plus in/out store
    std::vector<NodeLocation> locations;
    // Map from node pointer to index in `locations`
    std::map<const ASTNode*, int> nodeToId;
    // CFG edges: for each location L, who are its predecessors?
    std::map<int, std::vector<int>> preds;

    //-------------------------------------------
    // 1) Assign IDs & store NodeLocation
    //-------------------------------------------
    void assignLocation(const ASTNode &node) {
        if (nodeToId.find(&node) != nodeToId.end()) {
            // Already assigned
            return;
        }

        // Create a new location
        NodeLocation loc;
        loc.id   = nextId++;
        loc.node = &node;
        // Initialize inStore/outStore as empty => meaning "unknown" for variables
        loc.inStore = IntervalMap{};
        loc.outStore = IntervalMap{};

        // Register in data structures
        nodeToId[&node] = loc.id;
        locations.push_back(loc);

        // Recurse on children
        for (auto &child : node.children) {
            assignLocation(child);

            // For a naive approach, we might add "loc -> child" edges. But for a 
            // statement-level analysis, we only want "sequence" edges in actual order, 
            // so we won't do it blindly here. We handle sequences with a separate function.
        }
    }

    //-------------------------------------------
    // 2) Build CFG Edges for a Sequence
    //-------------------------------------------
    // If a node is a Sequence, it might have children that represent statements in order.
    void linkSequence(const ASTNode &seqNode) {
        // Suppose seqNode.children = [s1, s2, s3, ...]
        // We want: preds[id(s2)] += id(s1), preds[id(s3)] += id(s2), etc.
        for (size_t i = 0; i + 1 < seqNode.children.size(); ++i) {
            int currId = nodeToId[&seqNode.children[i]];
            int nextId = nodeToId[&seqNode.children[i+1]];
            preds[nextId].push_back(currId);

            // Also, if a child is itself a SEQUENCE, recursively link that sub-sequence
            if (seqNode.children[i].type == NodeType::SEQUENCE) {
                linkSequence(seqNode.children[i]);
            }
        }
        // If the last child is also a SEQUENCE, link it internally
        if (!seqNode.children.empty()) {
            auto &lastChild = seqNode.children.back();
            if (lastChild.type == NodeType::SEQUENCE) {
                linkSequence(lastChild);
            }
        }
    }

    //-------------------------------------------
    // 3) In[loc] = join( Out[pred] ) 
    //-------------------------------------------
    IntervalMap joinPredecessors(int locId) {
        IntervalMap result;  // empty
        if (preds.find(locId) == preds.end()) {
            // No predecessors => no join => result stays empty
            // You might want to interpret no predecessor as "TOP" if the code 
            // means "start of program". But let's keep it empty for now.
            return result;
        }
        // Otherwise, join all preds
        for (int p : preds[locId]) {
            IntervalMap &predOut = locations[p].outStore;
            result = joinIntervalMaps(result, predOut);
        }
        return result;
    }

    //-------------------------------------------
    // 4) Transfer Function
    //-------------------------------------------
    IntervalMap transfer(const ASTNode &node, const IntervalMap &inEnv) {
        switch (node.type) {
            case NodeType::DECLARATION:
                // e.g. int x, ...
                return transferDeclaration(node, inEnv);

            case NodeType::ASSIGNMENT:
                // e.g. x = <expr>;
                return transferAssignment(node, inEnv);

            case NodeType::PRE_CON:
                // e.g. /*!npk var between LB and UB */
                return transferPreCon(node, inEnv);

            case NodeType::POST_CON:
                // We might do checks or refinements, or just pass inEnv through
                return inEnv;

            case NodeType::SEQUENCE:
                // The out of a sequence node itself is typically the out of its last child
                // But we haven't done sub-locations for each child, so let's just pass inEnv.
                // The actual “flow” is handled by linkSequence and fixpoint across children.
                return inEnv;

            case NodeType::IFELSE:
                // Minimal support: just pass inEnv out. 
                // (Full branch analysis would split inEnv into inEnvTrue & inEnvFalse.)
                return inEnv;

            case NodeType::WHILELOOP:
                // We’ll handle loops in Exercise 4. For now, pass it through
                return inEnv;

            case NodeType::ARITHM_OP:
            case NodeType::LOGIC_OP:
            case NodeType::VARIABLE:
            case NodeType::INTEGER:
                // These appear typically as sub-expressions, not top-level statements.
                return inEnv;
        }
        return inEnv; 
    }

    //-------------------------------------------
    // 4a) Transfer: Declaration
    //-------------------------------------------
    IntervalMap transferDeclaration(const ASTNode &node, const IntervalMap &inEnv) {
        IntervalMap outEnv = inEnv;
        // node.children might be the declared variables
        // Typically: "int x;" => child = (VARIABLE, "x")
        // If you prefer, set them to TOP: [INT_MIN, INT_MAX]
        for (auto &child : node.children) {
            if (child.type == NodeType::VARIABLE) {
                std::string varName = std::get<std::string>(child.value);
                if (outEnv.find(varName) == outEnv.end()) {
                    outEnv[varName] = Interval(INT_MIN, INT_MAX);
                }
            }
        }
        return outEnv;
    }

    //-------------------------------------------
    // 4b) Transfer: Assignment
    //-------------------------------------------
    IntervalMap transferAssignment(const ASTNode &node, const IntervalMap &inEnv) {
        // node.children[0] = variable
        // node.children[1] = expression
        IntervalMap outEnv = inEnv;
        std::string varName = std::get<std::string>(node.children[0].value);
        // Evaluate expression
        Interval val = evalExpression(node.children[1], inEnv);
        outEnv[varName] = val;
        return outEnv;
    }

    //-------------------------------------------
    // 4c) Transfer: PreCon
    //-------------------------------------------
    // We interpret the "/*!npk var between LB and UB*/"
    // as two children: 
    //   child[0] = (Logic Operation, "LB <= var")
    //   child[1] = (Logic Operation, "UB >= var")
    // We refine var’s interval in the environment to [LB, UB].
    //-------------------------------------------
    IntervalMap transferPreCon(const ASTNode &node, const IntervalMap &inEnv) {
        // child[0]: e.g. LogicOp <= with left=LB, right=var
        // child[1]: e.g. LogicOp >= with left=UB, right=var
        IntervalMap outEnv = inEnv;

        // Typically from the parser, child[0] says something like: LB <= var
        // We can parse out LB and var. Let's assume structure is exactly:
        //    children[0]: (LOGIC_OP = "<=")
        //        children[0]: (INTEGER) LB
        //        children[1]: (VARIABLE) var
        //    children[1]: (LOGIC_OP = ">=")
        //        children[0]: (INTEGER) UB
        //        children[1]: (VARIABLE) var
        //
        // We'll refine var to [LB, UB] in outEnv.

        const ASTNode &condLB = node.children[0];
        const ASTNode &condUB = node.children[1];
        // LB = condLB.children[0]
        // var = condLB.children[1]
        int lbValue = 0;
        std::string varName;
        if (condLB.children[0].type == NodeType::INTEGER) {
            lbValue = std::get<int>(condLB.children[0].value);
        }
        if (condLB.children[1].type == NodeType::VARIABLE) {
            varName = std::get<std::string>(condLB.children[1].value);
        }

        // UB = condUB.children[0]
        // var = condUB.children[1]
        int ubValue = 0;
        if (condUB.children[0].type == NodeType::INTEGER) {
            ubValue = std::get<int>(condUB.children[0].value);
        }

        // Refine outEnv[varName] to at least [lbValue, ubValue]
        auto it = outEnv.find(varName);
        if (it == outEnv.end()) {
            // var not in env => set to [lbValue, ubValue]
            outEnv[varName] = Interval(lbValue, ubValue);
        } else {
            // var is known => intersect with [lbValue, ubValue]
            Interval oldI = it->second;
            Interval newI(
                std::max(oldI.lower, lbValue),
                std::min(oldI.upper, ubValue)
            );
            outEnv[varName] = newI;
        }

        return outEnv;
    }

    //-------------------------------------------
    // Expression evaluation with intervals
    //-------------------------------------------
    Interval evalExpression(const ASTNode &expr, const IntervalMap &env) {
        switch (expr.type) {
            case NodeType::INTEGER:
            {
                int val = std::get<int>(expr.value);
                return Interval(val, val);
            }
            case NodeType::VARIABLE:
            {
                std::string varName = std::get<std::string>(expr.value);
                if (env.find(varName) == env.end()) {
                    // If unknown, treat as top
                    return Interval(INT_MIN, INT_MAX);
                }
                return env.at(varName);
            }
            case NodeType::ARITHM_OP:
            {
                BinOp op = std::get<BinOp>(expr.value);
                Interval leftI = evalExpression(expr.children[0], env);
                Interval rightI = evalExpression(expr.children[1], env);
                try {
                    switch (op) {
                        case BinOp::ADD: return leftI + rightI;
                        case BinOp::SUB: return leftI - rightI;
                        case BinOp::MUL: return leftI * rightI;
                        case BinOp::DIV:
                            if (rightI.containsZero()) {
                                // Uncertain -> top
                                return Interval(INT_MIN, INT_MAX);
                            }
                            return leftI / rightI;
                    }
                } catch (...) {
                    // If something goes wrong, return top
                    return Interval(INT_MIN, INT_MAX);
                }
            }
            default:
                // For LogicOp or other node types used as expressions, we might skip or return top
                return Interval(INT_MIN, INT_MAX);
        }
    }

    //-------------------------------------------
    // 5) Helpers for IntervalMap
    //-------------------------------------------
    IntervalMap joinIntervalMaps(const IntervalMap &m1, const IntervalMap &m2) {
        IntervalMap result = m1; 
        // For each var in m2, join with result
        for (auto &kv : m2) {
            const std::string &var = kv.first;
            const Interval &i2 = kv.second;
            auto it = result.find(var);
            if (it == result.end()) {
                // not in result => add
                result[var] = i2;
            } else {
                // join intervals
                Interval i1 = it->second;
                Interval joined(
                    std::min(i1.lower, i2.lower),
                    std::max(i1.upper, i2.upper)
                );
                result[var] = joined;
            }
        }
        return result;
    }

    bool equalIntervalMaps(const IntervalMap &m1, const IntervalMap &m2) {
        if (m1.size() != m2.size()) return false;
        for (auto &kv : m1) {
            auto it = m2.find(kv.first);
            if (it == m2.end()) return false;
            if (kv.second.lower != it->second.lower ||
                kv.second.upper != it->second.upper) {
                return false;
            }
        }
        return true;
    }

    void printIntervalMap(const IntervalMap &m) const {
        if (m.empty()) {
            std::cout << "(empty)\n";
            return;
        }
        std::cout << "{ ";
        bool first = true;
        for (auto &kv : m) {
            if (!first) std::cout << ", ";
            std::cout << kv.first << " -> [" << kv.second.lower 
                      << "," << kv.second.upper << "]";
            first = false;
        }
        std::cout << " }\n";
    }
};

// ----------------------------------------------------------------------

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <testfile.c>\n";
        return 1;
    }

    std::ifstream f(argv[1]);
    if (!f.is_open()) {
        std::cerr << "[ERROR] Cannot open `" << argv[1] << "`.\n";
        return 1;
    }

    // Read entire file
    std::ostringstream buf;
    buf << f.rdbuf();
    f.close();
    std::string input = buf.str();

    // Parse
    EquationalFormInterpreter eq;
    ASTNode root = eq.parse(input);
    std::cout << "Parsing succeeded!\n";

    // Build locations & edges
    eq.buildLocations(root);

    // Solve fixpoint
    eq.solveFixpoint();

    // Print
    eq.printResults();

    return 0;
}
