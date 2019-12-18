#include "bica_graph/Types.hpp"

namespace bica_graph
{

std::vector<std::string> tokenize(const std::string & text, const std::string & delim)
{
  std::vector<std::string> ret;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = text.find(delim, start);
    ret.push_back(text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : (end + delim.length() - 1) + 1);
  }
  return ret;
}
bool operator==(const Node & op1, const Node & op2) 
{
  return op1.name == op2.name && op1.type == op2.type;
}

bool operator==(const Edge & op1, const Edge & op2) 
{
  return op1.content == op2.content && op1.type == op2.type;
}

}  // namespace bica_graph
