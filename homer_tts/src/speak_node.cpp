#include <homer_tts/speak_node.h>

void Speak::publishFeedback(float progress, std::string feedback)
{
  homer_tts::SpeakFeedback speak_feedback;
  speak_feedback.progress = progress / homer_tts::SpeakFeedback::STEPS;
  speak_feedback.feedback = feedback;
  m_as_.publishFeedback(speak_feedback);
}

void Speak::talkingFinishedCallback(const std_msgs::String::ConstPtr& msg)
{
  if (m_state == State::SPEAKING)
  {
    homer_tts::SpeakResult result;
    m_as_.setSucceeded(result);
    m_state = State::IDLE;
  }
}

void Speak::speakCallback()
{
  m_goal = m_as_.acceptNewGoal();
  ROS_DEBUG_STREAM("[SPEAK_ACTION] Got action goal" << *m_goal);
  if (m_goal->text == "")
  {
    ROS_INFO_STREAM("got empty speak goal - will not execute");
    homer_tts::SpeakResult result;
    m_as_.setSucceeded(result);
    return;
  }
  std_msgs::String msg;
  msg.data = m_goal->text;
  m_text_out_pub.publish(msg);
  m_state = State::SPEAKING;
  publishFeedback(homer_tts::SpeakFeedback::SPEAKING, "Action goal "
                                                          "received");
}

Speak::Speak(ros::NodeHandle n, std::string name) : m_as_(n, "speak", false), m_action_name_(name)
{
  m_state = State::IDLE;
  m_nh_ = n;

  m_as_.registerGoalCallback(boost::bind(&Speak::speakCallback, this));
  m_as_.start();

  m_talking_finished_sub =
      m_nh_.subscribe<std_msgs::String>("robot_face/talking_finished", 3, &Speak::talkingFinishedCallback, this);
  m_text_out_pub = m_nh_.advertise<std_msgs::String>("/robot_face/text_out", 3);

  ROS_DEBUG_STREAM("Speak Action Node initialized");
}

Speak::~Speak()
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speak_node");
  ros::NodeHandle n;
  Speak speak_node(n, ros::this_node::getName());
  ros::spin();
  return 0;
}
