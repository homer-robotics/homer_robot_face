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

#ifndef TALKING_HEAD_INCLUDE_CONFIG_H_
#define TALKING_HEAD_INCLUDE_CONFIG_H_

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include "Data.h"

/**
 * @class  Config
 * @brief  Loads a Configfile
 * @author Julian Giesen (R18)
 */

class Config
{
public:

    /** Constructor */
    Config( const char* filename, const char fieldSeparator = ':' )
    {
        std::fstream file( filename, std::ios::in );

        if( file )
        {
            std::string line;

            while( getline( file, line ) )
            {
                size_t sep = line.find_first_of( fieldSeparator );

                if (sep != std::string::npos)
                {
                    std::string key   = trim( line.substr( 0, sep ) );
                    std::string value = trim( line.substr( sep + 1 ) );

                    if( !key.empty() && !value.empty() )
                    {
                        myConfigItems[key] = static_cast<Data>( value );
                    }
                    else
                    {
                        throw std::runtime_error( "Error within configuration file." );
                    }
                }
                else
                {
                    throw std::runtime_error( "Error within configuration file." );
                }
            }
        }
        else
        {
            throw std::runtime_error( "Cannot open config file." );
        }
    }

    /** Destructor */
    ~Config(){}

    /** @return reads the configfile and returns the items */
    Data get( const std::string& key ) const
    {
        std::map<std::string, Data>::const_iterator it = myConfigItems.find( key );

        if ( it != myConfigItems.end() )
        {
            return it->second;
        }
        else
        {
            throw std::runtime_error( "Cannot find config item." );
        }
    }

    friend std::ostream& operator<<( std::ostream& os, const Config& config )
    {
      for( std::map<std::string, Data>::const_iterator it = config.myConfigItems.begin();
           it != config.myConfigItems.end();
           ++it )
      {
         const std::string& key = it->first;
         const std::string  val = it->second;

         std::cout << key << "\t" << val << std::endl;
      }

      return os;
    }

private:

   /// @return trims leading and trailing spaces and returns the prepared String
   std::string trim( std::string str )
   {
      size_t pos = str.find_first_not_of( " \t\n" );

      if ( pos != std::string::npos )
      {
         str.erase( 0, pos );
      }

      pos = str.find_last_not_of( " \t\n" );

      if ( pos != std::string::npos )
      {
         str.erase( pos + 1 );
      }

      return str;
   }

   std::map<std::string, Data> myConfigItems;
};

#endif // TALKING_HEAD_INCLUDE_CONFIG_H_
