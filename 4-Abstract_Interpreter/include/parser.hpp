#ifndef ABSTRACT_INTERPRETER_PARSER_HPP
#define ABSTRACT_INTERPRETER_PARSER_HPP

#include "peglib.h"
#include <assert.h>
#include <iostream>
#include "ast.hpp"

class FancyParser {
    using SV = peg::SemanticValues;

public:
    ASTNode parse(const std::string &input) {
        peg::parser grammar(R"(
            Program     <- Statements*
            Statements  <- DeclareVar / Assignment / Increment / IfElse / WhileLoop / Block / PreCon / PostCon / Comment
            Integer     <- < [+-]? [0-9]+ >
            Identifier  <- < [a-zA-Z_][a-zA-Z0-9_]* >
            SeqOp       <- '+' / '-'
            PreOp       <- '*' / '/'
            LogicOp     <- '<=' / '>=' / '==' / '!=' / '<' / '>'
            DeclareVar  <- 'int' Identifier ('=' Integer / ',' Identifier)* ';'
            PreCon      <- '/*!npk' Identifier 'between' Integer 'and' Integer '*/'
            PostCon     <- 'assert' '(' Expression ')' ';'
            Assignment  <- Identifier '=' Expression ';'
            Increment   <- Identifier '++' ';'
            Block       <- ('void main' '(' ')')? '{' Statements* '}'
            IfElse      <- 'if' '(' Expression ')' (Block / Statements) ('else' (Block / Statements))?
            WhileLoop   <- 'while' '(' Expression ')' (Block / Statements)
            Expression  <- Term ((SeqOp / LogicOp) Term)*
            Term        <- Factor (PreOp Factor)*
            Factor      <- '-' Factor / Integer / Identifier / '(' Expression ')'
            ~Comment    <- '//' [^\n\r]* [ \n\r\t]*
            %whitespace <- [ \n\r\t]*
        )");

        assert(static_cast<bool>(grammar) == true);

        // Binding actions
        grammar["Program"] = [this](const SV &sv) { return makeProgram(sv); };
        grammar["Integer"] = [](const SV &sv) { return ASTNode(sv.token_to_number<int>()); };
        grammar["Identifier"] = [](const SV &sv) { return ASTNode(sv.token_to_string()); };
        grammar["SeqOp"] = [this](const SV &sv) { return makeSeqOp(sv); };
        grammar["PreOp"] = [this](const SV &sv) { return makePreOp(sv); };
        grammar["LogicOp"] = [this](const SV &sv) { return makeLogicOp(sv); };
        grammar["DeclareVar"] = [this](const SV &sv) { return makeDeclaration(sv); };
        grammar["PreCon"] = [this](const SV &sv) { return makePreConstraint(sv); };
        grammar["PostCon"] = [this](const SV &sv) { return makePostConstraint(sv); };
        grammar["Assignment"] = [this](const SV &sv) { return makeAssign(sv); };
        grammar["Increment"] = [this](const SV &sv) { return makeIncrement(sv); };
        grammar["Block"] = [this](const SV &sv) { return makeBlock(sv); };
        grammar["IfElse"] = [this](const SV &sv) { return makeIfElse(sv); };
        grammar["WhileLoop"] = [this](const SV &sv) { return makeWhileLoop(sv); };
        grammar["Expression"] = [this](const SV &sv) { return makeExpression(sv); };
        grammar["Term"] = [this](const SV &sv) { return makeTerm(sv); };
        grammar["Factor"] = [this](const SV &sv) { return makeFactor(sv); };

        grammar.set_logger([](size_t l, size_t c, const std::string &msg, const std::string &) {
            std::cerr << l << ":" << c << ": " << msg << "\n";
        });

        ASTNode root;
        if (grammar.parse(input.c_str(), root)) {
            std::cout << "Parsing DONE!" << std::endl;
        } else {
            std::cerr << "Parsing FAIL!" << std::endl;
        }
        return root;
    }

private:
    ASTNode makeProgram(const SV &sv) {
        if (sv.size() == 1) {
            return std::any_cast<ASTNode>(sv[0]);
        } else {
            ASTNode root;
            for (size_t i = 0; i < sv.size(); ++i) {
                try {
                    root.children.push_back(std::any_cast<ASTNode>(sv[i]));
                } catch (std::bad_any_cast &) {
                    // ignoring comments or non-AST
                    continue;
                }
            }
            return root;
        }
    }

    ASTNode makeDeclaration(const SV &sv) {
        ASTNode declNode(NodeType::DECLARATION, std::string("int"));
        for (size_t i = 0; i < sv.size(); ++i) {
            declNode.children.push_back(std::any_cast<ASTNode>(sv[i]));
        }
        return declNode;
    }

    ASTNode makePreConstraint(const SV &sv) {
        ASTNode preNode(NodeType::PRE_CON, std::string("PreCon"));
        ASTNode var(std::any_cast<ASTNode>(sv[0]));

        // LB => LogicOp::GEQ
        ASTNode geNode(NodeType::LOGIC_OP, LogicOp::GEQ);
        geNode.children.push_back(std::any_cast<ASTNode>(sv[1]));
        geNode.children.push_back(var);

        // UB => LogicOp::LEQ
        ASTNode leNode(NodeType::LOGIC_OP, LogicOp::LEQ);
        leNode.children.push_back(std::any_cast<ASTNode>(sv[2]));
        leNode.children.push_back(var);

        preNode.children.push_back(geNode);
        preNode.children.push_back(leNode);
        return preNode;
    }

    ASTNode makePostConstraint(const SV &sv) {
        ASTNode postNode(NodeType::POST_CON, std::string("PostCon"));
        ASTNode expr(std::any_cast<ASTNode>(sv[0]));
        postNode.children.push_back(expr);
        return postNode;
    }

    ASTNode makeSeqOp(const SV &sv) {
        ASTNode node(NodeType::ARITHM_OP);
        std::string op = sv.token_to_string();
        if (op == "+") node.value = BinOp::ADD;
        else if (op == "-") node.value = BinOp::SUB;
        return node;
    }

    ASTNode makePreOp(const SV &sv) {
        ASTNode node(NodeType::ARITHM_OP);
        std::string op = sv.token_to_string();
        if (op == "*") node.value = BinOp::MUL;
        else if (op == "/") node.value = BinOp::DIV;
        return node;
    }

    ASTNode makeLogicOp(const SV &sv) {
        ASTNode node(NodeType::LOGIC_OP);
        std::string content = sv.token_to_string();
        if (content == "<")  node.value = LogicOp::LE;
        else if (content == ">")  node.value = LogicOp::GE;
        else if (content == "<=") node.value = LogicOp::LEQ;
        else if (content == ">=") node.value = LogicOp::GEQ;
        else if (content == "==") node.value = LogicOp::EQ;
        else if (content == "!=") node.value = LogicOp::NEQ;
        return node;
    }

    ASTNode makeExpression(const SV &sv) {
        if (sv.size() == 1) {
            return std::any_cast<ASTNode>(sv[0]);
        } 
        else if (sv.size() == 3) {
            ASTNode op = std::any_cast<ASTNode>(sv[1]);
            ASTNode expr(op.type, op.value);
            expr.children.push_back(std::any_cast<ASTNode>(sv[0]));
            expr.children.push_back(std::any_cast<ASTNode>(sv[2]));
            return expr;
        } 
        else {
            // multiple ops
            ASTNode firstOp = std::any_cast<ASTNode>(sv[1]);
            ASTNode expr(NodeType::ARITHM_OP, firstOp.value);
            expr.children.push_back(std::any_cast<ASTNode>(sv[0]));

            ASTNode subOp;
            size_t i = 3;
            for (; i < sv.size(); i += 2) {
                subOp = std::any_cast<ASTNode>(sv[i]);
                subOp.children.push_back(std::any_cast<ASTNode>(sv[i - 1]));
                if (i + 2 < sv.size()) {
                    expr.children.push_back(subOp);
                }
            }
            subOp.children.push_back(std::any_cast<ASTNode>(sv[i - 1]));
            expr.children.push_back(subOp);
            return expr;
        }
    }

    ASTNode makeTerm(const SV &sv) {
        if (sv.size() == 1) {
            return std::any_cast<ASTNode>(sv[0]);
        } 
        else if (sv.size() == 3) {
            ASTNode node(NodeType::ARITHM_OP, std::any_cast<ASTNode>(sv[1]).value);
            node.children.push_back(std::any_cast<ASTNode>(sv[0]));
            node.children.push_back(std::any_cast<ASTNode>(sv[2]));
            return node;
        } 
        else {
            ASTNode term(NodeType::ARITHM_OP, std::any_cast<ASTNode>(sv[1]).value);
            term.children.push_back(std::any_cast<ASTNode>(sv[0]));
            size_t i = 3;
            for (; i < sv.size(); i += 2) {
                ASTNode opNode(NodeType::ARITHM_OP, std::any_cast<ASTNode>(sv[i]).value);
                term.children.push_back(opNode);
                term.children.push_back(std::any_cast<ASTNode>(sv[i - 1]));
            }
            term.children.push_back(std::any_cast<ASTNode>(sv[i - 1]));
            return term;
        }
    }

    ASTNode makeFactor(const SV &sv) {
        if (sv.choice() == 0) {
            // unary minus
            ASTNode sign(NodeType::ARITHM_OP, std::string("-"));
            sign.children.push_back(ASTNode(0));
            sign.children.push_back(std::any_cast<ASTNode>(sv[0]));
            return sign;
        } else {
            return std::any_cast<ASTNode>(sv[0]);
        }
    }

    ASTNode makeAssign(const SV &sv) {
        ASTNode node(NodeType::ASSIGNMENT, std::string("="));
        node.children.push_back(std::any_cast<ASTNode>(sv[0]));
        node.children.push_back(std::any_cast<ASTNode>(sv[1]));
        return node;
    }

    ASTNode makeIncrement(const SV &sv) {
        ASTNode incNode(NodeType::ASSIGNMENT, std::string("="));
        ASTNode varNode = std::any_cast<ASTNode>(sv[0]);

        ASTNode plusOp(NodeType::ARITHM_OP, std::string("+"));
        plusOp.children.push_back(varNode);
        plusOp.children.push_back(ASTNode(1));

        incNode.children.push_back(varNode);
        incNode.children.push_back(plusOp);
        return incNode;
    }

    ASTNode makeBlock(const SV &sv) {
        if (sv.size() == 1) {
            return std::any_cast<ASTNode>(sv[0]);
        } 
        else {
            ASTNode seq(NodeType::SEQUENCE, std::string(";"));
            for (size_t i = 0; i < sv.size(); ++i) {
                try {
                    seq.children.push_back(std::any_cast<ASTNode>(sv[i]));
                } 
                catch (std::bad_any_cast &) {
                    continue;
                }
            }
            return seq;
        }
    }

    ASTNode makeIfElse(const SV &sv) {
        ASTNode ifelse(NodeType::IFELSE, std::string("IfElse"));
        for (size_t i = 0; i < sv.size(); ++i) {
            ASTNode interim;
            if (i == 0) interim = ASTNode(NodeType::IFELSE, std::string("Condition"));
            else if (i == 1) interim = ASTNode(NodeType::IFELSE, std::string("If-Body"));
            else if (i == 2) interim = ASTNode(NodeType::IFELSE, std::string("Else-Body"));
            ASTNode c = std::any_cast<ASTNode>(sv[i]);
            interim.children.push_back(c);
            ifelse.children.push_back(interim);
        }
        return ifelse;
    }

    ASTNode makeWhileLoop(const SV &sv) {
        ASTNode whileNode(NodeType::WHILELOOP, std::string("WhileLoop"));
        for (size_t i = 0; i < sv.size(); ++i) {
            ASTNode interim;
            if (i == 0) interim = ASTNode(NodeType::WHILELOOP, std::string("Condition"));
            else if (i == 1) interim = ASTNode(NodeType::WHILELOOP, std::string("While-Body"));
            ASTNode c = std::any_cast<ASTNode>(sv[i]);
            interim.children.push_back(c);
            whileNode.children.push_back(interim);
        }
        return whileNode;
    }
};

#endif
