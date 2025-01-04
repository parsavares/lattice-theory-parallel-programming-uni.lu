#ifndef ABSTRACT_INTERPRETER_AST_HPP
#define ABSTRACT_INTERPRETER_AST_HPP

/*
   =============================================================================
   This header defines the AST (Abstract Syntax Tree) components for our analyzer.
   It includes:
     * BinOp, LogicOp enumerations for arithmetic and logical operations.
     * NodeType enumeration for various AST node categories.
     * ASTNode struct storing node type, value, and children, along with an ID.
   
   Each node can represent a statement (e.g., Assignment) or an expression 
   (Arithmetic, Logic). The `print()` method is handy for debugging the AST.
   =============================================================================
*/

#include <variant>
#include <cmath>
#include <iostream>
#include <atomic>
#include <vector>
#include <string>

// Basic arithmetic operations
enum class BinOp {ADD, SUB, MUL, DIV};

// Printing a BinOp as a symbolic string
inline std::ostream &operator<<(std::ostream &os, BinOp op) {
    switch (op) {
        case BinOp::ADD: os << "+"; break;
        case BinOp::SUB: os << "-"; break;
        case BinOp::MUL: os << "*"; break;
        case BinOp::DIV: os << "/"; break;
    }
    return os;
}

// Logical comparison operators
enum class LogicOp {LE, LEQ, GE, GEQ, EQ, NEQ};

// Helper to invert a logic operator (e.g., for while condition negation)
inline LogicOp get_opposite(LogicOp lop) {
    switch (lop) {
        case LogicOp::LE:  return LogicOp::GEQ;
        case LogicOp::LEQ: return LogicOp::GE;
        case LogicOp::GE:  return LogicOp::LEQ;
        case LogicOp::GEQ: return LogicOp::LE;
        case LogicOp::EQ:  return LogicOp::NEQ;
        case LogicOp::NEQ: return LogicOp::EQ;
        default: 
            throw std::invalid_argument("Unknown logic operator in get_opposite()");
    }
}

// Printing a logic operator as a symbolic string
inline std::ostream &operator<<(std::ostream &os, LogicOp lop) {
    switch (lop) {
        case LogicOp::LE:  os << "<"; break;
        case LogicOp::LEQ: os << "<="; break;
        case LogicOp::GE:  os << ">"; break;
        case LogicOp::GEQ: os << ">="; break;
        case LogicOp::EQ:  os << "=="; break;
        case LogicOp::NEQ: os << "!="; break;
    }
    return os;
}

// Categorizes an AST node (variable, if-else, while, assignment, etc.)
enum class NodeType {
    VARIABLE, INTEGER, PRE_CON, POST_CON, 
    ARITHM_OP, LOGIC_OP, DECLARATION, 
    ASSIGNMENT, IFELSE, WHILELOOP, SEQUENCE
};

// Printing a node type for debugging
inline std::ostream &operator<<(std::ostream &os, NodeType type) {
    switch (type) {
        case NodeType::VARIABLE:    os << "Variable"; break;
        case NodeType::INTEGER:     os << "Integer"; break;
        case NodeType::PRE_CON:     os << "Pre conditions"; break;
        case NodeType::POST_CON:    os << "Post conditions"; break;
        case NodeType::ARITHM_OP:   os << "Arithmetic Operation"; break;
        case NodeType::LOGIC_OP:    os << "Logic Operation"; break;
        case NodeType::DECLARATION: os << "Declaration"; break;
        case NodeType::ASSIGNMENT:  os << "Assignment"; break;
        case NodeType::IFELSE:      os << "If-Else"; break;
        case NodeType::WHILELOOP:   os << "While-Loop"; break;
        case NodeType::SEQUENCE:    os << "Sequence"; break;
    }
    return os;
}

// Represents an AST node, storing type, a generic value, and any children.
struct ASTNode {
    using VType = std::variant<std::string, int, BinOp, LogicOp>;
    using NodeArray = std::vector<ASTNode>;

    static std::atomic<size_t> counter; // auto-incrementing ID generator
    size_t id;                         // unique ID for debugging or warnings
    NodeType type;                     // e.g., VARIABLE, ASSIGNMENT, ARITHM_OP
    VType value;                       // could be a string (var name), int, or op
    NodeArray children;                // sub-nodes if any (e.g., expression children)

    // Overloads for easy ASTNode creation
    ASTNode() : id(counter++), type(NodeType::INTEGER), value(0) {}
    ASTNode(const std::string &s) : id(counter++), type(NodeType::VARIABLE), value(s) {}
    ASTNode(int n) : id(counter++), type(NodeType::INTEGER), value(n) {}
    ASTNode(BinOp bop, ASTNode left, ASTNode right)
        : id(counter++), type(NodeType::ARITHM_OP), value(bop) {
        children.push_back(left);
        children.push_back(right);
    }
    ASTNode(LogicOp lop, ASTNode left, ASTNode right)
        : id(counter++), type(NodeType::LOGIC_OP), value(lop) {
        children.push_back(left);
        children.push_back(right);
    }
    ASTNode(NodeType t) : id(counter++), type(t), value(0) {}
    ASTNode(NodeType t, const std::string &nm) 
        : id(counter++), type(t), value(nm) {}
    ASTNode(NodeType t, const VType &val) 
        : id(counter++), type(t), value(val) {}

    // Helper to print the 'value' union
    static void printVal(const VType &v) {
        std::visit([](auto &&arg) {
            std::cout << arg << std::endl;
        }, v);
    }

    // Recursively prints the node and its children, indenting by depth
    void print(int depth = 0) const {
        std::string indent(depth * 2, ' ');
        std::cout << indent << "ID: " << id 
                  << ", NodeType: " << type 
                  << ", Value: ";
        printVal(value);
        for (auto &c : children) {
            c.print(depth + 1);
        }
    }
};

// Initialize the static counter to 0
inline std::atomic<size_t> ASTNode::counter{0};

#endif
