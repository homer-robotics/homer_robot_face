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

#ifndef TALKING_HEAD_INCLUDE_TALKINGHEAD_H_
#define TALKING_HEAD_INCLUDE_TALKINGHEAD_H_

#include <OgreAnimation.h>
#include <OgreCamera.h>
#include <OgreConfigFile.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreMaterialManager.h>
#include <OgreMeshManager.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>
#include <OgreWindowEventUtilities.h>

#include <QtWidgets>

//messages to react to
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <homer_robot_face/DisplayImage.h>
#include <homer_robot_face/DisplayImageFile.h>

#include <iostream>
#include <istream>
#include <map>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


/**
 * @class  TalkingHead
 * @brief  Displays a TalkingHead
 * @author Julian Giesen (R16)(R18)
 */

class TalkingHead : public QWidget, public Ogre::FrameListener, public Ogre::WindowEventListener
  {
  Q_OBJECT

  public:

  /** QWidget Constructor */
  TalkingHead( QWidget *parent = 0, std::string mesh_string = "", std::vector< std::vector<float> > material_vector = std::vector< std::vector<float> >(), int window_rotation = 0 );

  /** Destructor */
  virtual ~TalkingHead();

  /** @brief Creates Ogre::Scene and manages Renderingloop */
  void initOgreSystem( void );

  int getWindowRotation() const {return window_rotation_;}

  /** @brief is called when a String-msg arrives and activates speech synth. / animation. */
  void callbackVisemes();
  void callbackTextForEmotion( const std_msgs::String::ConstPtr& msg );
  void callbackResetAnimation( const std_msgs::String::ConstPtr& msg);
  void callbackShowStream(const homer_robot_face::DisplayImage::ConstPtr& image_msg );
  void callbackShowImage( const homer_robot_face::DisplayImageFile::ConstPtr& msg );
  void callbackShowMatImage( const sensor_msgs::ImageConstPtr& msg );

  public slots:

  void updateOverlay();

  void clearFace();

  void setTimer(int msec);

  signals:
  void faceCleared();

  void timerChanged(int msec);

  protected:

  /** @brief Qt related */
  virtual void paintEvent( QPaintEvent* event );
  virtual void resizeEvent( QResizeEvent* event );
  virtual void showEvent( QShowEvent* event );
  virtual void moveEvent( QMoveEvent* event );

  private:

  /// File Access
  std::vector<Ogre::String> 				phones_;
  std::vector<Ogre::String>                               words_;

  /// Ogre::Scene
  Ogre::Root*                                             root_;
  Ogre::Camera*        					camera_;
  Ogre::Viewport*                                         viewport_;
  Ogre::SceneManager*                                     scene_manager_;
  bool                 					shut_down_;
  Ogre::RenderWindow*  					window_;

  /// PoseAnimation
  Ogre::MeshPtr 						mesh_;
  std::string                                             mesh_string_;
  std::vector< std::vector<float> >                       material_vector_;
  Ogre::MaterialPtr                                       material_;
  Ogre::PoseList						pose_list_;
  int							num_sub_meshes_;
  Ogre::AnimationStateSet* 				anim_set_;
  Ogre::AnimationState* 					mouth_animation_state_;
  Ogre::String 						mouth_animation_state_name_;
  Ogre::AnimationState*                                   eyebrows_animation_state_;
  Ogre::String                                            eyebrows_animation_state_name_;
  std::vector<Ogre::VertexAnimationTrack*>                vertex_animation_tracks_;
  Ogre::VertexPoseKeyFrame*                               manual_key_frame_;
  bool                                                    visemes_arrived_;
  bool                                                    words_arrived_;
  std::map<float, Ogre::String>                           word_map_;
  std::map<float, Ogre::String>                           viseme_map_;
  /// map of phonemes with correspondent visemes
  std::map<Ogre::String, Ogre::String>                    phonemes_;
  std::string                                             text_for_emotion_;

  int                                                     window_rotation_;
  QTimer*                                                 redraw_timer_;
  bool                                                    is_visible_;
  unsigned int                                            show_time_;
  bool                                                    init_;

  /// Emotion related
  std::vector<std::string>                                smileys_;
  bool                                                    angry_;
  bool                                                    smile_;
  bool                                                    sad_;
  bool                                                    rest_;
  bool                                                    surprised_;
  bool                                                    disgusted_;
  bool                                                    afraid_;

  /// Ogre::FrameListener
  virtual bool frameRenderingQueued( const Ogre::FrameEvent& evt );

  /** @brief Loads a Mesh with Poses/ShapeKeys and creates an Animation for the Poses. */
  void createAnimations( std::string mesh_file );

  /** @brief initialises the AnimationState */
  void initAnimationStates();

  /** @brief initialises the map of phonemes with correspondent visemes. */
  void initPhoneMap();

  /** @return creates a map with the actual visemes and timestamps to visualize and returns it. */
  std::map<float, Ogre::String> createVisemeMap();

  /** @return creates a map with words and timestamps for emotion pipeline and returns it. */
  std::map<float, Ogre::String> createWordMap();

  /** @brief creates the Keyframes for Mouth and enables them for Animation */
  void playTalkAnimation();

  /** @brief updates the poses */
  void updatePoses( Ogre::String pose_name, float weight );

  /** @brief change material colors for each submesh */
  void changeMaterialColor();

  };

#endif // TALKING_HEAD_INCLUDE_TALKINGHEAD_H_
