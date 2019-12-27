// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>

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
