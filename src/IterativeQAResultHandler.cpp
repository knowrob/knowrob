// BOOST
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
// KnowRob
#include <knowrob/KnowledgeBase.h>

using namespace knowrob;
namespace po = boost::program_options;

class IterativeQAResultHandler : public QueryResultHandler {
public:
    std::string errorMessage;
    std::list<std::string> solutions_;
    IterativeQAResultHandler(const std::string &queryString)
            : has_stop_request_(false)
    {
        numSolutions_ = 0;
        errorMessage = "";
        currentQuery_ = queryString;
    }

    static std::string substitutionToJSONString(const std::shared_ptr<Substitution> &omega) {
        auto ss = std::stringstream();
        uint32_t i=0;
        ss << "{\n";
        for(const auto &pair : *omega) {
            if(i++ > 0) ss << ",\n";
            // Convert the value to string
            std::stringstream second_stream;
            second_stream << (*pair.second);
            std::string value = second_stream.str();
            if ( value.front() == '\'' ) { // if the value starts with a single quote
                value.erase( 0, 1 );
                value.erase( value.size() - 1 );
            }
            ss << '\"' << pair.first.name() << "\":" << '\"' << value << "\"";
        }
        ss << '}';
        return ss.str();
    }

// Override QueryResultHandler
    bool pushQueryResult(const QueryResultPtr &solution) override {
        if(solution->substitution()->empty()) {
            solutions_.push_back("True.");
        }
        else {
            solutions_.push_back(substitutionToJSONString(solution->substitution()));
        }
        numSolutions_ += 1;
        return !has_stop_request_;
    }

    bool has_more_solutions() const {
        return numSolutions_ > 0;
    }

    std::string next_solution() {
        std::string solution = solutions_.front();
        solutions_.pop_front();
        numSolutions_ -= 1;
        return solution;
    }

    bool has_error() const {
        if (errorMessage.empty()) {
            return false;
        } else {
            return true;
        }

    }

    const std::string& error() const {
        return errorMessage;
    }

    void pushSolution(const std::string &solution) {
        solutions_.push_back(solution);
    }

    void setError(const std::string& error) {
        errorMessage = error;
    }

    void finish() {
        // TODO: What to do during finish?
    }

    int getNumSolutions() const {
        return numSolutions_;
    };


protected:
    std::atomic<bool> has_stop_request_;
    int numSolutions_;
    std::string currentQuery_;
};
