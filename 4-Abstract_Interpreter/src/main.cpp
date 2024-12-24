#include <fstream>
#include <sstream>
#include <climits> // For INT32_MIN and INT32_MAX

#include "parser.hpp"
#include "ast.hpp"

// Function to check for division by zero in the AST
void checkDivisionByZero(const ASTNode& node) {
    if (node.type == NodeType::ARITHM_OP && std::get<BinOp>(node.value) == BinOp::DIV) {
        const auto& denominator = node.children[1];

        // Check if the denominator is zero or contains zero in its range
        if (denominator.type == NodeType::INTEGER && std::get<int>(denominator.value) == 0) {
            std::cerr << "Warning: Division by zero detected!" << std::endl;
        }
        // Add checks for interval-based values if intervals are implemented
    }

    // Recursively check all children nodes
    for (const auto& child : node.children) {
        checkDivisionByZero(child);
    }
}

// Function to check for overflow in arithmetic operations
void checkOverflow(const ASTNode& node) {
    if (node.type == NodeType::ARITHM_OP) {
        const auto& op = std::get<BinOp>(node.value);
        const auto& left = node.children[0];
        const auto& right = node.children[1];

        // Only check for overflow if both children are integer literals
        if (left.type == NodeType::INTEGER && right.type == NodeType::INTEGER) {
            int leftVal = std::get<int>(left.value);
            int rightVal = std::get<int>(right.value);

            // Check for overflow based on the operation
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

    // Recursively check all children nodes
    for (const auto& child : node.children) {
        checkOverflow(child);
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "usage: " << argv[0] << " tests/00.c" << std::endl;
        return 1;
    }

    // Open the input file
    std::ifstream f(argv[1]);
    if (!f.is_open()) {
        std::cerr << "[ERROR] cannot open the test file `" << argv[1] << "`." << std::endl;
        return 1;
    }

    // Read the contents of the file into a string
    std::ostringstream buffer;
    buffer << f.rdbuf();
    std::string input = buffer.str();
    f.close();

    // Parse the input program and print the AST
    std::cout << "Parsing program `" << argv[1] << "`..." << std::endl;
    AbstractInterpreterParser AIParser;
    ASTNode ast = AIParser.parse(input);
    ast.print();

    // Check for division-by-zero errors
    checkDivisionByZero(ast);

    // Check for overflow errors
    checkOverflow(ast);

    return 0;
}
