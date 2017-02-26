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

#ifndef _CSV_PARSER_H__
#define _CSV_PARSER_H__

#include <fstream>
#include <string>
#include <vector>

class CSVParser {
 public:
  CSVParser();

  // Opens a file.
  bool Open(const std::string& filename, bool has_header = false);

  // Closes the file.
  void Close();

  // Get number of columns.
  int NumColumns() { return num_columns_; }

  // Advance line.
  bool Next();

  // Get fields.
  std::vector<std::string> Fields();
  std::vector<float> NumberFields();

 private:
  std::ifstream csv_file_;
  int num_columns_;
  std::string current_line_;
};

#endif
