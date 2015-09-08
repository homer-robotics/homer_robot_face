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

#ifndef TALKING_HEAD_INCLUDE_TEXTOUTDISPLAY_H_
#define TALKING_HEAD_INCLUDE_TEXTOUTDISPLAY_H_

#include "RotationLabel.h"
#include "TextProcessor.h"

#include <QWidget>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <vector>

/**
 * @class  TextOutDisplay
 * @brief  Displays the currently spoken sentence
 * @author Julian Giesen (R16)(R18), David Gossow (R12)
 */
class TextOutDisplay : public QWidget
{

  Q_OBJECT

  public:

    /** Constructor */
    TextOutDisplay( int min_height = 0, int font_size=15, bool user_input = false, int text_rotation = 0, QWidget* parent = 0 );

    /** Destructor */
    ~TextOutDisplay();

    void callbackText( const std_msgs::String::ConstPtr& msg );

    void callbackTalkingFinished( const std_msgs::String::ConstPtr& msg );

  public slots:

    /// @brief use setText("") for a delayed text reset
    void setText( std::string text );

    void clearText();

  private:

    std::string                 text_;

    RotationLabel               *text_out_label_;
    QFont                       font_;

    QTimer                      *reset_timer_;

    std::vector<std::string>    smileys_;

    bool                        user_input_;

    TextProcessor               text_processor_;
};

#endif // TALKING_HEAD_INCLUDE_TEXTOUTDISPLAY_H_
