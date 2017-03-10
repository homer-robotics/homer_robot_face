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

#ifndef TALKING_HEAD_INCLUDE_QTROSNODE_H_
#define TALKING_HEAD_INCLUDE_QTROSNODE_H_

#include <QApplication>
#include <QObject>
#include <QThread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>

#include <string.h>

class MainWindow;

/**
 * @class  QtRosNode
 * @brief  Handles the ROS connection
 * @author Julian Giesen (R16)(R18)
 */

class QtRosNode : public QThread {

  Q_OBJECT

  public:
  /// Note: The constructor will block until connected with roscore
  /// Instead of ros::spin(), start this thread with the start() method
  /// to run the event loop of ros
  QtRosNode(int argc, char *argv[], const char* node_name, MainWindow* main_window, QApplication* app);

  ~QtRosNode();

  ros::NodeHandle* getNodeHandle(){ return node_handle_; }

  /** @brief This method contains the ROS event loop. Feel free to modify */
  void run();

  /** @brief returning the window that displays the face stuff */
  const MainWindow * getMainWindow(){ return main_window_;}

  void callbackTalkingFinished( const std_msgs::String::ConstPtr& msg );

  public slots:
  /// Connect to aboutToQuit signals, to stop the thread
  void quitNow();

  signals:
  /// Triggered if ros::ok() != true
  void rosQuits();

  private:

  bool                quit_from_gui;
  bool                talking_finished_;

  ros::NodeHandle*    node_handle_;
  ros::NodeHandle*    text_out_node_handle_;
  ros::NodeHandle*    tf_node_handle_;

  ros::CallbackQueue  text_out_callback_queue_;
  ros::CallbackQueue  tf_callback_queue_;

  ros::Subscriber     emotion_subscriber_;
  ros::Subscriber     generator_talking_finished_subscriber_;
  ros::Subscriber     face_talking_finished_subscriber_;
  ros::Subscriber     text_talking_finished_subscriber_;
  ros::Subscriber     text_out_subscriber_;
  ros::Subscriber     user_input_subscriber_;
  ros::Subscriber     expected_input_subscriber_;
  ros::Subscriber     synth_subscriber_;
  ros::Subscriber     talking_finished_subscriber_;
  ros::Subscriber     image_stream_subscriber_;
  ros::Subscriber     image_stream_show_subscriber_;
  ros::Subscriber     face_image_stream_show_subscriber_;
  ros::Subscriber     face_image_display_show_subscriber_;
  ros::Subscriber     image_subscriber_;
  ros::Subscriber     image_show_subscriber_;
  ros::Subscriber     image_file_display_subscriber_;

  image_transport::Subscriber image_sub_;
  ros::Subscriber     image_mat_subscriber;
  image_transport::Subscriber face_mat_image_display_show_subscriber_;


  std::string         tf_text_;

  MainWindow*         main_window_;
  QApplication*       application_;

  /// subscribe to topics here
  void subscribeTopics();
  };

#endif // TALKING_HEAD_INCLUDE_QTROSNODE_H_
