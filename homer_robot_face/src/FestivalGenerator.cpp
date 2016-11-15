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

#include <string>

#include <homer_robot_face/FestivalGenerator.h>

FestivalGenerator::FestivalGenerator ( QObject* parent ) :
    QObject( parent ),
    synth_phonemes_( false ),
    synth_words_( false ),
    file_ready_( false ),
    punctuation_characters_( 0 )
{
    timer_ = new QTimer ( this );  // create internal timer
    connect ( timer_, SIGNAL ( timeout() ), SLOT ( run() ) );
    connect( this, SIGNAL( timerChanged(int) ), SLOT( setTimer(int) ) );
    timer_->start( 1000.0 / 25 );
}

FestivalGenerator::~FestivalGenerator ()
{
    if( timer_ ) delete timer_;
    festival_tidy_up();
}

void FestivalGenerator::run()
{
    if( synth_phonemes_ && synth_words_ )
    {
        festival_wait_for_spooler();

        synthPhonemes( text_for_synth_ );
        synthWords( text_for_synth_ );

        file_ready_ = true;

        synth_phonemes_ = false;
        synth_words_ = false;

        text_processor_.setSmiley( false );
    }

    if( text_processor_.getSmiley() )
    {
        file_ready_ = true;

        text_processor_.setSmiley( false );
    }
}

void FestivalGenerator::callbackSynth( const std_msgs::String::ConstPtr& msg )
{
    text_for_synth_ = text_processor_.prepareText( msg->data, TextProcessor::GENERATE );
    if( text_for_synth_.length() > 0 )
    {
        synth_phonemes_ = true;
        synth_words_ = true;
        if(!file_ready_)
            emit timerChanged( 1000.0 / 25 );
    }
}

void FestivalGenerator::callbackTalkingFinished( const std_msgs::String::ConstPtr& msg )
{
    file_ready_ = false;
    emit timerChanged( 1000.0 / 25 );
}

void FestivalGenerator::synthPhonemes( std::string text )
{
    ostringstream command;
    command << "(set! utt1 (SynthText \"" << text << "\"))";
    festival_eval_command( command.str().c_str() );
    festival_eval_command( "(utt.save.segs utt1 \"phonemes.txt\"))" );
}

void FestivalGenerator::synthWords( std::string text )
{
    ostringstream command;
    command << "(set! utt2 (SynthText \"" << text << "\"))";
    festival_eval_command( command.str().c_str() );
    festival_eval_command( "(utt.save.words utt2 \"words.txt\"))" );
    emit timerChanged( 100000 );
}

bool FestivalGenerator::isFileReady()
{
    if( file_ready_ )
    {
        return true;
    }
    return false;
}

void FestivalGenerator::setFileReady( bool value )
{
    file_ready_ = value;
}

void FestivalGenerator::setTimer(int msec)
{
    timer_->start(msec);
}
