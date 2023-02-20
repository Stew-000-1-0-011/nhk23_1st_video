#include <optional>

#include <ros/ros.h>

#include <pluginlib/class_list_macros.hpp>
#include <nodelet/nodelet.h>

#include <adhoc_canplugins_onehalf/CanFrame.h>
#include <omni4/Twist2D.h>

#include <logicool.hpp>

namespace momotarou_node
{
	constexpr double rotation_speed = 3.14;
	constexpr double linear_speed_factor = 1.5;

	class MomotarouNodeInner final
	{
		CRSLib::Logicool logicool;

		// その時点でどうであるか
		bool collector_nail_open{true};
		bool load_nail_open{true};


		ros::Publisher collector_nail_pub{};
		ros::Publisher load_nail_pub{};
		ros::Publisher body_twist_velocity_pub{};
		ros::Timer pub_tim{};

		MomotarouNodeInner(MomotarouNodeInner&&) = delete;
		MomotarouNodeInner& operator=(MomotarouNodeInner&&) = delete;

	public:
		MomotarouNodeInner(ros::NodeHandle& nh):
			logicool{nh, "joy"}
		{
			collector_nail_pub = nh.advertise<adhoc_canplugins_onehalf::CanFrame>("can501", 1);
			load_nail_pub = nh.advertise<adhoc_canplugins_onehalf::CanFrame>("can500", 1);
			body_twist_velocity_pub = nh.advertise<omni4::Twist2D>("body_twist_velocity", 1);

			pub_tim = nh.createTimer(ros::Duration(1.0 / 1000), &MomotarouNodeInner::pub_tim_callback, this);
		}

	private:
		void pub_tim_callback(const ros::TimerEvent&)
		{
			using Buttons = CRSLib::Logicool::Buttons;
			using Axes = CRSLib::Logicool::Axes;

			omni4::Twist2D body_twist_velocity{};

			if(logicool.is_being_pushed(Buttons::lb))
			{
				body_twist_velocity.z += rotation_speed;
			}
			else if(logicool.is_being_pushed(Buttons::rb))
			{
				body_twist_velocity.z -= rotation_speed;
			}

			body_twist_velocity.x = linear_speed_factor * logicool.get_axis(Axes::l_stick_LR);
			body_twist_velocity.y = linear_speed_factor * logicool.get_axis(Axes::cross_UD);

			body_twist_velocity_pub.publish(body_twist_velocity);

			if(logicool.is_pushed_down(Buttons::x, ros::Duration(0.5)))
			{
				adhoc_canplugins_onehalf::CanFrame frame{};
				frame.data[0] = !collector_nail_open;
				collector_nail_open = !collector_nail_open;
				frame.dlc = 1;

				collector_nail_pub.publish(frame);
			}
			
			if(logicool.is_pushed_down(Buttons::y, ros::Duration(0.5)))
			{
				adhoc_canplugins_onehalf::CanFrame frame{};
				frame.data[0] = !load_nail_open;
				load_nail_open = !load_nail_open;
				frame.dlc = 1;

				load_nail_pub.publish(frame);
			}
		}
	};

	class MomotarouNode final : public nodelet::Nodelet
	{
		std::optional<MomotarouNodeInner> inner{};

		void onInit() override
		{
			auto nh = getNodeHandle();
			inner.emplace(nh);
		}
	};
}

PLUGINLIB_EXPORT_CLASS(momotarou_node::MomotarouNode, nodelet::Nodelet);