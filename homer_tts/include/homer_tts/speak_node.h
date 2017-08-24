#ifndef SPEAK_NODE_H_C5VS8RBE
#define SPEAK_NODE_H_C5VS8RBE

// ROS
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <homer_tts/SpeakAction.h>
#include <homer_tts/SpeakActionFeedback.h>
#include <homer_tts/SpeakActionResult.h>

#include <actionlib/server/simple_action_server.h>

enum State
{
    IDLE,
    SPEAKING
};

typedef actionlib::SimpleActionServer<homer_tts::SpeakAction> Server;

class Speak
{
	protected:
		Server m_as_;
		std::string m_action_name_;

	public:
		Speak(ros::NodeHandle n, std::string name);
		~Speak();

	private:
		// Member Variables
		ros::NodeHandle m_nh_;
		homer_tts::SpeakGoal::ConstPtr m_goal;

		void speakCallback();

		void publishFeedback(float progress, std::string feedback);

		void talkingFinishedCallback(const std_msgs::String::ConstPtr& msg);

        State m_state;

		ros::Subscriber m_talking_finished_sub;
        ros::Publisher m_text_out_pub;

		ros::NodeHandle nh;
};

#endif /* end of include guard: SPEAK_NODE_H_C5VS8RBE */
