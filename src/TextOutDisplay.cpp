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

#include <homer_robot_face/TextOutDisplay.h>
#include <homer_robot_face/RotationLabel.h>

#include <QLabel>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <string>

TextOutDisplay::TextOutDisplay(int min_height, int font_size, int type, int text_rotation, QWidget* parent) :
    QWidget( parent )
{
  type_ = type;

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
  connect( this, SIGNAL( timerChanged(int) ), SLOT ( setTimer(int) ) );
  //reset_timer_->start( 40 );

  setVisible( false );

}

TextOutDisplay::~TextOutDisplay()
{
    if( text_out_label_ ) delete text_out_label_;
    if( reset_timer_ ) delete reset_timer_;
}

void TextOutDisplay::clearText()
{
    text_out_label_->setText( "" );
    setVisible( false );
}

void TextOutDisplay::setText( std::string text )
{
  if ( text == "" )
  {
      if(type_ == ::EXP)
      {
          setVisible(false);
          return;
      }
      emit timerChanged(40);
  }
  else
  {
      if( type_ == ::REC)
      {
          font_.setPixelSize( 18 );
          text_out_label_->setFont(font_);
          setVisible( true );
          text_out_label_->setText( text.c_str() );
          emit timerChanged( 2000);
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
          if( type_ != ::EXP)
          {
              emit timerChanged( 10000 );
          }
      }
  }
}

void TextOutDisplay::setTimer(int msec)
{
    reset_timer_->start(msec);
}

void TextOutDisplay::callbackText( const std_msgs::String::ConstPtr& msg )
{
    setText( text_processor_.prepareText( msg->data, TextProcessor::DISPLAY ));
}

void TextOutDisplay::callbackTalkingFinished( const std_msgs::String::ConstPtr& msg )
{
    emit timerChanged(  300 );
}
