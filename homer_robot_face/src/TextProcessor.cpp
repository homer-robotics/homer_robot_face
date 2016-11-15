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

#include <homer_robot_face/TextProcessor.h>

#include <string>

TextProcessor::TextProcessor() :
      publish_smiley_( false )
{
    smileys_.push_back(">:");
    smileys_.push_back(":)");
    smileys_.push_back(":(");
    smileys_.push_back(":O");
    smileys_.push_back(":o");
    smileys_.push_back(":!");
    smileys_.push_back(":&");

    punctuation_characters_.push_back( ":" );
    punctuation_characters_.push_back( ")" );
    punctuation_characters_.push_back( "(" );
    punctuation_characters_.push_back( "'" );
    punctuation_characters_.push_back( "\"" );
    punctuation_characters_.push_back( "}" );
    punctuation_characters_.push_back( "{" );
    punctuation_characters_.push_back( "§" );
    punctuation_characters_.push_back( "!" );
    punctuation_characters_.push_back( "?" );
    punctuation_characters_.push_back( "`" );
    punctuation_characters_.push_back( "´" );
    punctuation_characters_.push_back( "-" );
    punctuation_characters_.push_back( ";" );
    punctuation_characters_.push_back( "€" );
    punctuation_characters_.push_back( "°" );
    punctuation_characters_.push_back( "<" );
    punctuation_characters_.push_back( ">" );
}

TextProcessor::~TextProcessor(){ }

std::string TextProcessor::trimSpaces( std::string text )
{
    std::string tmp_text = text;

    size_t startpos = tmp_text.find_first_not_of(" \t");
    size_t endpos = tmp_text.find_last_not_of(" \t");

    if(( std::string::npos == startpos ) || ( std::string::npos == endpos))
    {
        tmp_text = "";
    }
    else
    {
        tmp_text = tmp_text.substr( startpos, endpos-startpos+1 );
    }

    return tmp_text;
}

std::string TextProcessor::clearSmileys( std::string text )
{
    std::string tmp_text = text;

    size_t i_smiley = std::string::npos;

    for( unsigned int j = 0; j < smileys_.size(); j++ )
    {
        for( unsigned int i = 0; i < tmp_text.length(); i++ )
        {
            i_smiley = tmp_text.find( smileys_.at( j ), 0 );
            if( i_smiley != std::string::npos )
            {
                tmp_text.erase(i_smiley, smileys_.at( j ).length());
            }
        }
    }

    return tmp_text;

}

std::string TextProcessor::prepareText( std::string text, textcase tcase  )
{
    std::string tmp_text = text;

    tmp_text = trimSpaces(tmp_text);
    if( tcase == GENERATE )
    {
        for( unsigned int j = 0; j < smileys_.size(); j++ )
        {
            size_t find_smiley = tmp_text.find( smileys_.at( j ) );
            if( find_smiley != std::string::npos )
            {
                publish_smiley_ = true;
            }
        }
        size_t find_dot = tmp_text.find( "." );
        if( find_dot != std::string::npos )
        {
            publish_smiley_ = true;
        }
    }
    tmp_text = clearSmileys(tmp_text);

    if( tcase == SYNTHESIZE || tcase == GENERATE )
    {
        size_t i_symbol = std::string::npos;

        for( unsigned int j = 0; j < punctuation_characters_.size(); j++ )
        {
        for( unsigned int i = 0; i < tmp_text.length(); i++ )
        {
                i_symbol = tmp_text.find( punctuation_characters_.at( j ), 0 );
                if( i_symbol != std::string::npos )
                {
                    tmp_text.erase(i_symbol, punctuation_characters_.at( j ).length());
                }
            }
        }
    }

    if( tcase == GENERATE || tcase == DISPLAY )
    {
        size_t find_dot = tmp_text.find_first_of(".", 0);
        if( find_dot != std::string::npos )
        {
            tmp_text.erase( find_dot, 1 );
        }
    }

    tmp_text = trimSpaces( tmp_text );

    if( tmp_text == ".")
    {
        tmp_text = "";
    }

    return tmp_text;
}

bool TextProcessor::getSmiley()
{
    return publish_smiley_;
}

void TextProcessor::setSmiley( bool value )
{
    publish_smiley_ = value;
}
