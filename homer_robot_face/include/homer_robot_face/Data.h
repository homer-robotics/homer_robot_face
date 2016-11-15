/*******************************************************************************
 *  TalkingHead - A talking head for robots
 *  Copyright (C) 2012 AG Aktives Sehen <agas@uni-koblenz.de>
 *                     Universitaet Koblenz-Landau
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA 02110-1301  USA or see <http://www.gnu.org/licenses/>.
 *******************************************************************************/

#ifndef TALKING_HEAD_INCLUDE_DATA_H_
#define TALKING_HEAD_INCLUDE_DATA_H_

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * @class  Data
 * @brief  Describes Datatypes for reading Configfiles
 * @author Julian Giesen (R18)
 */

class Data
{
public:

    /** Constructor */
    explicit Data(const std::string& data = "") : data_(data) {}

    /** Destructor */
    ~Data(){}

    /** @brief Converts text to int. */
    operator int() const
    {
        std::stringstream ss(data_);
        int val = 0;
        ss >> val;

        if (ss.fail())
        {
            throw std::runtime_error("Cannot convert to int.");
        }
        return val;
    }

    /** @brief Converts text to char. */
    operator const char*() const
    {
        return data_.c_str();
    }

    /** @brief Returns text as string. */
    operator std::string() const
    {
        return data_;
    }

    /** @brief Converts text to vector<float>. */
    operator std::vector<float>() const
    {
        std::string str1 = "";
        std::string str2 = "";
        std::string str3 = "";

        size_t sep = data_.find_first_of(",");
        size_t sep1 = data_.find_last_of(",");


        if (sep != std::string::npos)
        {
            str1 = data_.substr(0, sep);
            str2 = data_.substr(sep + 1, sep1 - 1);
            str3 = data_.substr(sep1 + 1, data_.length() - 1);
        }

        std::stringstream ss1(str1);
        float val1 = 0.0;
        ss1 >> val1;
        std::stringstream ss2(str2);
        float val2 = 0.0;
        ss2 >> val2;
        std::stringstream ss3(str3);
        float val3 = 0.0;
        ss3 >> val3;

        if (ss1.fail() || ss2.fail() || ss3.fail())
        {
            throw std::runtime_error("Cannot convert to float.");
        }
        float values[] = {val1, val2, val3};
        std::vector<float> vec(values, values + sizeof(values) / sizeof(float));

        return vec;
    }

    /** @brief Converts text to bool. */
    operator bool() const
    {
        return data_ == "true" || data_ == "True" || data_ == "TRUE";
    }

private:

   std::string data_;

};

#endif // TALKING_HEAD_INCLUDE_DATA_H_
