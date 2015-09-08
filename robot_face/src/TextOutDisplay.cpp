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

#include "TextOutDisplay.h"
#include "RotationLabel.h"

#include <QLabel>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <string>

TextOutDisplay::TextOutDisplay(int min_height, int font_size, bool user_input, int text_rotation, QWidget* parent) :
    QWidget( parent )
{
  user_input_ = user_input;

  QBoxLayout* layout = new QVBoxLayout( this );

  // Text output
  font_.setPixelSize(font_size);
  font_.setBold(true);

  text_out_label_ = new RotationLabel( this );
  text_out_label_->setFont( font_ );
  text_out_label_->setMinimumSize( 100, min_height );
  text_out_label_->setAlignment( Qt::AlignCenter );
  text_out_label_->setWordWrap( true );
  text_out_label_->rotateText( text_rotation );

  layout->addWidget( text_out_label_ );

  setLayout( layout );

  reset_timer_ = new QTimer( this );  // create internal timer
  connect( reset_timer_, SIGNAL ( timeout() ), SLOT ( clearText() ) );
  reset_timer_->start( 1000 / 25  );

  setVisible( false );

  text_ = "";
}

TextOutDisplay::~TextOutDisplay()
{
    if( text_out_label_ ) delete text_out_label_;
    if( reset_timer_ ) delete reset_timer_;
}

void TextOutDisplay::clearText()
{
  if( text_ != "" )
  {
    setText( text_ );
    if( user_input_ )
    {
        text_ = "";
    }
  }
  else
  {
    setText( "" );
    text_out_label_->setText( "" );
    setVisible( false );
  }
}

void TextOutDisplay::setText( std::string text )
{
  if ( text == "" )
  {
    reset_timer_->start ( 1000 / 25  );
  }
  else
  {
      if( user_input_ )
      {
          font_.setPixelSize( 18 );
          text_out_label_->setFont(font_);
          setVisible( true );
          text_out_label_->setText( text.c_str() );
          reset_timer_->start( 10000 );
      }
      else
      {
          if( text.length() >= 80 )
          {
              font_.setPixelSize( 18 );
              text_out_label_->setFont(font_);
          }
          else
          {
              font_.setPixelSize( 25 );
              text_out_label_->setFont(font_);
          }
          setVisible( true );
          text_out_label_->setText( text.c_str() );
          reset_timer_->start ( 10000  );
      }
  }
}

void TextOutDisplay::callbackText( const std_msgs::String::ConstPtr& msg )
{
    std::string out = "";

    //    out = "You said: ";

    out.append(msg->data);
    text_ = text_processor_.prepareText( out, TextProcessor::DISPLAY );
    if( user_input_ )
    {
       reset_timer_->start( text_.length() * 10  );
    }
}

void TextOutDisplay::callbackTalkingFinished( const std_msgs::String::ConstPtr& msg )
{
    text_ = "";
    reset_timer_->start( text_.length() * 100  );
}
