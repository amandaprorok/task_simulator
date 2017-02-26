/*-------------------------------------------------------------------
# Copyright (C) 2012 Sibi <sibi@psibi.in>
#
# This file is part of csv-parser.
#
# csv-parser is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# csv-parser is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with csv-parser.  If not, see <http://www.gnu.org/licenses/>.
#
# Author:   Sibi <sibi@psibi.in>
--------------------------------------------------------------------*/

#include "csv_parser.h"

#include <assert.h>
#include <sstream>

CSVParser::CSVParser() : num_columns_(0) {}

bool CSVParser::Open(const std::string& filename, bool has_header) {
  csv_file_.open(filename.c_str());
  if (!csv_file_.is_open()) {
    return false;
  }
  if (csv_file_.eof()) {
    num_columns_ = 0;
    return true;
  }
  // Count number of columns.
  // This is a very dumb implementation.
  getline(csv_file_, current_line_);
  int num_columns_ = 1;
  for (std::string::const_iterator it = current_line_.begin(); it < current_line_.end(); it++) {
    num_columns_ += (*it == ',');
  }
  if (!has_header) {
    csv_file_.clear();
    csv_file_.seekg(0, std::ios::beg);
  }
  return true;
}

void CSVParser::Close() {
  csv_file_.close();
}

bool CSVParser::Next() {
  if (csv_file_.eof()) {
    return false;
  }
  getline(csv_file_, current_line_);
  return true;
}

namespace {
std::vector<std::string> &Split(const std::string &s, char delim, std::vector<std::string> &elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> Split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  Split(s, delim, elems);
  return elems;
}
}  // namespace.

std::vector<std::string> CSVParser::Fields() {
  return Split(current_line_, ',');
}

std::vector<float> CSVParser::NumberFields() {
  std::vector<float> v;
  for (const std::string& token : Fields()) {
    float f;
    sscanf(token.c_str(), "%f", &f);
    v.push_back(f);
  }
  return v;
}
