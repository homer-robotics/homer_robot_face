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

#ifndef TALKING_HEAD_INCLUDE_FESTIVALSYNTHESIZER_H_
#define TALKING_HEAD_INCLUDE_FESTIVALSYNTHESIZER_H_

#include <homer_robot_face/TextProcessor.h>

#include <festival.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <fstream>
#include <ostream>
#include <string>
#include <vector>

class pa_simple;

/**
 * @class FestivalSynthesizer
 * @brief Provides text-to-speech sythesis via Festival
 * @author Julian Giesen (R16)(R18)
 */


class FestivalSynthesizer
{
    public:

        /** Constructor */
        FestivalSynthesizer();

        /** Destructor */
        ~FestivalSynthesizer();

        void callbackSynth( const std_msgs::String::ConstPtr& msg );

        void run();

    private:

        pa_simple*                  pa_simple_;

        bool                        festival_initialized_;
        bool                        synth_speech_;
        std::string                 text_for_synth_;
        std::vector<std::string>    punctuation_characters_;
        std::vector<std::string>    smileys_;
        TextProcessor               text_processor_;

        /// Define your custom ROS subscribers, callbacks and publishers here
        ros::Publisher              talking_finished_publisher_;
        ros::Publisher              speech_out_status_publisher_;
        ros::Subscriber             subscriber_;

        void initFestival();

        /// @brief Synthesize speech for given text
        void synthSpeech ( std::string text );
};

#endif // TALKING_HEAD_INCLUDE_FESTIVALSYNTHESIZER_H_
