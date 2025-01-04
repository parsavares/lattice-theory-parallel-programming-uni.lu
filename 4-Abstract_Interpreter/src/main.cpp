#include <fstream>
#include <sstream>

#include "parser.hpp"
#include "ast.hpp"
#include "ValueInterval.hpp"
#include "DisjointRangeSet.hpp"
#include "InvariantStore.hpp"
#include "abstract_interpreter.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "usage: " << argv[0] << " tests/00.c" << std::endl;
        return 1;
    }
    std::ifstream f(argv[1]);
    if (!f.is_open()) {
        std::cerr << "[ERROR] cannot open file `" << argv[1] << "`." << std::endl;
        return 1;
    }

    std::ostringstream buf;
    buf << f.rdbuf();
    std::string input = buf.str();
    f.close();

    std::cout << "Parsing `" << argv[1] << "`..." << std::endl;
    FancyParser fParser;
    ASTNode rootAst = fParser.parse(input);
    rootAst.print();

    FancyAnalyzer analyzer;
    analyzer.constructSolverSystem(rootAst);

    analyzer.solveSystem();
    std::cout << std::endl;

    analyzer.printSystemState();
    std::cout << std::endl;

    analyzer.printSystemNotes();
    return 0;
}
