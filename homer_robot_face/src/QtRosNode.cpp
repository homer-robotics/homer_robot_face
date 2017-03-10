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

#include <homer_robot_face/MainWindow.h>
#include <homer_robot_face/QtRosNode.h>
#include <homer_robot_face/TalkingHead.h>

QtRosNode::QtRosNode(int argc, char *argv[], const char* node_name, MainWindow* main_window, QApplication* app)
{
  node_handle_ = new ros::NodeHandle;
  main_window_ = main_window;
  application_ = app;

  text_out_node_handle_ = new ros::NodeHandle;
  text_out_node_handle_->setCallbackQueue( &text_out_callback_queue_ );

  tf_node_handle_ = new ros::NodeHandle;
  tf_node_handle_->setCallbackQueue( &tf_callback_queue_ );
  tf_text_ = "talking_finished";

  subscribeTopics();

  quit_from_gui = false;
  talking_finished_ = true;
}

QtRosNode::~QtRosNode()
{
  if( node_handle_ ) delete node_handle_;
  if( text_out_node_handle_) delete text_out_node_handle_;
  if( tf_node_handle_ ) delete tf_node_handle_;
  if( main_window_ ) delete main_window_;
  if( application_ ) delete application_;
}

void QtRosNode::subscribeTopics()
{
  //talking head subscribers
  talking_finished_subscriber_ = node_handle_->subscribe<std_msgs::String>( "/robot_face/talking_finished", 10, &QtRosNode::callbackTalkingFinished, this );
  emotion_subscriber_ = text_out_node_handle_->subscribe<std_msgs::String>( "/robot_face/text_out", 10, &TalkingHead::callbackTextForEmotion , main_window_->getFaceWidget() );
  face_talking_finished_subscriber_ = text_out_node_handle_->subscribe<std_msgs::String>( "/robot_face/talking_finished", 10, &TalkingHead::callbackResetAnimation, main_window_->getFaceWidget());
  //face_image_stream_show_subscriber_ = node_handle_->subscribe<std_msgs::Int8>( "/robot_face/image_stream", 1000, &TalkingHead::callbackShowStream, main_window_->getFaceWidget() );
  face_image_stream_show_subscriber_ = node_handle_->subscribe<homer_robot_face::DisplayImage>( "/robot_face/ImageDisplay", 10, &TalkingHead::callbackShowStream, main_window_->getFaceWidget() );
  face_image_display_show_subscriber_ = node_handle_->subscribe<homer_robot_face::DisplayImageFile>( "/robot_face/ImageFileDisplay", 10, &TalkingHead::callbackShowImage, main_window_->getFaceWidget() );

  // text output subscribers
  text_out_subscriber_ = text_out_node_handle_->subscribe<std_msgs::String>( "/robot_face/text_out", 10, &TextOutDisplay::callbackText, main_window_->getTextWidget(MainWindow::OUT) );
  text_talking_finished_subscriber_ = node_handle_->subscribe<std_msgs::String>( "/robot_face/talking_finished", 10, &TextOutDisplay::callbackTalkingFinished, main_window_->getTextWidget(MainWindow::OUT) );
  user_input_subscriber_ = node_handle_->subscribe<std_msgs::String>( "/recognized_speech", 10, &TextOutDisplay::callbackText, main_window_->getTextWidget(MainWindow::REC) );

  // expected Input subscriber
  expected_input_subscriber_ = node_handle_->subscribe<std_msgs::String>( "/robot_face/expected_input", 10, &TextOutDisplay::callbackText, main_window_->getTextWidget(MainWindow::EXP) );
  
  // speak out subscriber
  synth_subscriber_ = text_out_node_handle_->subscribe<std_msgs::String>( "/robot_face/text_out", 10, &FestivalGenerator::callbackSynth, main_window_->getGenerator() );
  generator_talking_finished_subscriber_ = tf_node_handle_->subscribe<std_msgs::String>( "/robot_face/talking_finished", 10, &FestivalGenerator::callbackTalkingFinished, main_window_->getGenerator() );

  // image subscribers
  image_stream_subscriber_= node_handle_->subscribe<homer_robot_face::DisplayImage>( "/robot_face/ImageDisplay", 5, &ImageDisplay::callbackImageDisplay, main_window_->getImageStream() );
  image_file_display_subscriber_ =  node_handle_->subscribe<homer_robot_face::DisplayImageFile>( "/robot_face/ImageFileDisplay", 10, &ImageDisplay::callbackImageFileDisplay, main_window_->getImageStream() );
}

void QtRosNode::quitNow()
{
  quit_from_gui = true;
}

void QtRosNode::run()
{
  ros::Rate loop_rate(20);

  while (ros::ok() && !quit_from_gui)
  {
    if( talking_finished_ )
      text_out_callback_queue_.callOne(ros::WallDuration());
    
    if( tf_text_ == "talking_finished")
      tf_callback_queue_.callOne(ros::WallDuration());

    if( main_window_->getGenerator()->isFileReady() )
    {
      main_window_->getFaceWidget()->callbackVisemes();

      talking_finished_ = false;

      if( tf_text_ == "smiley")
      {
        tf_callback_queue_.callOne(ros::WallDuration());
        talking_finished_ = true;
      }
    }


//#define ROCKIN_HACK
#ifdef ROCKIN_HACK
	//turn off mic while talking // RoCKIn hack
	std::string mic_volume_command = "";
	if(talking_finished_)
	{
		//ROS_INFO_STREAM("[HACK] Reactivating mic");
		mic_volume_command = "amixer set Capture 100%";
	}
	else
	{
		//ROS_INFO_STREAM("[HACK] Muting mic");
		mic_volume_command = "amixer set Capture 0%";
	}
	int change_mic_volume_result = std::system(mic_volume_command.c_str());
	//end turn off mic while talking
#endif

    main_window_->updateGeometry();

    loop_rate.sleep();

    ros::spinOnce();
  }

  if (!ros::ok())
  {
    application_->exit();
    emit rosQuits();
    ROS_INFO("ROS-Node Terminated\n");
  }
}

void QtRosNode::callbackTalkingFinished(const std_msgs::String::ConstPtr& msg)
{
  tf_text_ = "talking_finished";
  talking_finished_ = true;
}
