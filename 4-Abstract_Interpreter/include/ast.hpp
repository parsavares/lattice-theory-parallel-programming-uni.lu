// copyright 2024 Yi-Nung Tsao

#ifndef ABSTRACT_INTERPRETER_AST_HPP
#define ABSTRACT_INTERPRETER_AST_HPP

#include <variant>
#include <cmath>
#include <algorithm> // For std::min and std::max

// Enum for binary operations
enum class BinOp {ADD, SUB, MUL, DIV};
std::ostream& operator<<(std::ostream& os, BinOp op) {
    switch (op) {
        case BinOp::ADD: os << "+"; break;
        case BinOp::SUB: os << "-"; break;
        case BinOp::MUL: os << "*"; break;
        case BinOp::DIV: os << "/"; break;
    }
    return os;
}

// Enum for logical operations
enum class LogicOp {LE, LEQ, GE, GEQ, EQ, NEQ};
std::ostream& operator<<(std::ostream& os, LogicOp lop) {
    switch (lop) {
        case LogicOp::LE: os << "<"; break;
        case LogicOp::LEQ: os << "<="; break;
        case LogicOp::GE: os << ">"; break;
        case LogicOp::GEQ: os << ">="; break;
        case LogicOp::EQ: os << "=="; break;
        case LogicOp::NEQ: os << "!="; break;
    }
    return os;
}

// Enum for node types
enum class NodeType {
    VARIABLE, INTEGER, PRE_CON, POST_CON, ARITHM_OP, LOGIC_OP,
    DECLARATION, ASSIGNMENT, IFELSE, WHILELOOP, SEQUENCE
};
std::ostream& operator<<(std::ostream& os, NodeType type) {
    switch (type) {
        case NodeType::VARIABLE: os << "Variable"; break;
        case NodeType::INTEGER: os << "Integer"; break;
        case NodeType::PRE_CON: os << "Pre conditions"; break;
        case NodeType::POST_CON: os << "Post conditions"; break;
        case NodeType::ARITHM_OP: os << "Arithmetic Operation"; break;
        case NodeType::LOGIC_OP: os << "Logic Operation"; break;
        case NodeType::DECLARATION: os << "Declaration"; break;
        case NodeType::ASSIGNMENT: os << "Assignment"; break;
        case NodeType::IFELSE: os << "If-Else"; break;
        case NodeType::WHILELOOP: os << "While-Loop"; break;
        case NodeType::SEQUENCE: os << "Sequence"; break;
    }
    return os;
}

// Interval class for range-based analysis
class Interval {
public:
    int lower; // Lower bound of the interval
    int upper; // Upper bound of the interval

    Interval(int l, int u) : lower(l), upper(u) {}

    // Check if the interval contains a specific value
    bool contains(int value) const {
        return value >= lower && value <= upper;
    }

    // Check if the interval contains zero
    bool containsZero() const {
        return contains(0);
    }

    // Addition of two intervals
    Interval operator+(const Interval& other) const {
        return Interval(lower + other.lower, upper + other.upper);
    }

    // Subtraction of two intervals
    Interval operator-(const Interval& other) const {
        return Interval(lower - other.upper, upper - other.lower);
    }

    // Multiplication of two intervals
    Interval operator*(const Interval& other) const {
        return Interval(std::min({lower * other.lower, lower * other.upper,
                                  upper * other.lower, upper * other.upper}),
                        std::max({lower * other.lower, lower * other.upper,
                                  upper * other.lower, upper * other.upper}));
    }

    // Division of two intervals (raises an error if the divisor contains zero)
    Interval operator/(const Interval& other) const {
        if (other.containsZero()) {
            throw std::runtime_error("Division by zero interval detected!");
        }
        return Interval(std::min({lower / other.upper, lower / other.lower,
                                  upper / other.lower, upper / other.upper}),
                        std::max({lower / other.upper, lower / other.lower,
                                  upper / other.lower, upper / other.upper}));
    }

    // Print interval
    friend std::ostream& operator<<(std::ostream& os, const Interval& interval) {
        os << "[" << interval.lower << ", " << interval.upper << "]";
        return os;
    }
};

// AST Node structure
struct ASTNode {
    using VType = std::variant<std::string, int, BinOp, LogicOp>;
    using ASTNodes = std::vector<ASTNode>;

    NodeType type;
    VType value;
    ASTNodes children;

    ASTNode() : type(NodeType::INTEGER), value(0) {}
    ASTNode(const std::string& name) : type(NodeType::VARIABLE), value(name) {}
    ASTNode(const int num) : type(NodeType::INTEGER), value(num) {}
    ASTNode(BinOp bop, ASTNode left, ASTNode right)
        : type(NodeType::ARITHM_OP), value(bop) {
            children.push_back(left);
            children.push_back(right);
    }
    ASTNode(LogicOp lop, ASTNode left, ASTNode right)
        : type(NodeType::LOGIC_OP), value(lop) {
            children.push_back(left);
            children.push_back(right);
    }
    ASTNode(NodeType t) : type(t) {}
    ASTNode(NodeType t, const std::string& name) : type(t), value(name) {}
    ASTNode(NodeType t, const VType& value) : type(t), value(value) {}

    static void printVariant(const VType& value) {
        std::visit([](const auto& v) {
            std::cout << v << std::endl;
        }, value);
    }

    void print(int depth = 0) const {
        std::string indent(depth * 2, ' ');
        std::cout << indent << "NodeType: " << type << ", Value: ";
        printVariant(value);
        for (const auto& child : children) {
            child.print(depth + 1);
        }
    }
};

#endif
