#include <sstream>
#include <iostream>
#include <fstream>

#include "utils.hpp"

namespace routing {

Point2i CardinalDirectionToVector(const std::string& dir)
{
  if (dir == "'N'") {
    return Point2i(0, 1);
  } else if (dir == "'E'") {
    return Point2i(1, 0);
  } else if (dir == "'S'") {
    return Point2i(0, -1);
  } else if (dir == "'W'") {
    return Point2i(-1, 0);
  } else {
    throw std::runtime_error("Unrecognized cardinal direction.");
  }
}

std::vector<std::string> ParseTuple(const std::string& input_str)
{
  // Remove spaces, parens, and square brackets.
  std::string tupstr = input_str;
  std::vector<char> remove_chars = { ')', '(', ' ', '[', ']' };
  for (const char ch : remove_chars) {
    tupstr.erase(std::remove(tupstr.begin(), tupstr.end(), ch), tupstr.end());
  }

  // Split the input string around commas.
  std::vector<std::string> tokens;
  std::stringstream ss(tupstr);
  std::string tmp;
  while (std::getline(ss, tmp, ',')) {
    tokens.emplace_back(tmp);
  }

  return tokens;
}

std::string ConvertPathToString(const std::vector<Point2i>& path)
{
  std::string output = "[";
  for (size_t i = 0; i < path.size(); ++i) {
    const Point2i& pt = path.at(i);

    char buf[4 + std::to_string(pt.x).size() + std::to_string(pt.y).size()];
    int size = sprintf(buf, "(%d, %d)", path.at(i).x, path.at(i).y);
    output += std::string(buf);

    // Add comma between coords if not the last one.
    if (i != path.size() - 1) { output += ", "; }
  }
  output += "]";
  return output;
}

}
