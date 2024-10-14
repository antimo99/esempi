#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "temperatura_interfaces/action/media.hpp"
#include "temperatura_interfaces/action/traj.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task_node");

  using Media = temperatura_interfaces::action::Media;
  using Traj = temperatura_interfaces::action::Traj;

  //definisco le configurazioni in cui spostarsi
  //float qa[]={0, 0, 0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_4};
  //float qb[]={0, 0, 0, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_4};
  //float qc[]={M_PI_2, 0, 0, -M_PI_2, -M_PI_2, M_PI, -M_PI_4};

  //inizializzo due action client: uno per la action traj e uno per la action media
  auto joint_traj_action_client =
      rclcpp_action::create_client<Traj>(node, "joint_traj_action");
  
  auto calcola_media_action_client =
      rclcpp_action::create_client<Media>(node, "calcola_media_action");
  
  //attendo che i server siano attivi
  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");
  joint_traj_action_client->wait_for_action_server();
  calcola_media_action_client->wait_for_action_server();
  RCLCPP_INFO(node->get_logger(), "Servers UP");
  
  //mi sposto nelle 3 configurazioni e in ognuna di esse effettuo il calcolo della media
  {
    Traj::Goal joint_goal;
    for(int i=0; i<3; i++)
    {
        if(i==0)
            joint_goal.qf={0, 0, 0, -M_PI_2, M_PI_2, M_PI_2, -M_PI_4}; //qa
        else if(i==1)
            joint_goal.qf={0, 0, 0, -M_PI_2, -M_PI_2, M_PI_2, -M_PI_4}; //qb
        else if(i==2)
            joint_goal.qf={M_PI_2, 0, 0, -M_PI_2, -M_PI_2, M_PI, -M_PI_4}; //qc

        joint_goal.time=rclcpp::Duration::from_seconds(2);

            // chiamo l'azione e aspetto che termini
            auto future_goal_handle1 = joint_traj_action_client->async_send_goal(joint_goal);
            if (rclcpp::spin_until_future_complete(node, future_goal_handle1) != rclcpp::FutureReturnCode::SUCCESS)
            {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
            return -1;
            }

            auto future_result1 = joint_traj_action_client->async_get_result(future_goal_handle1.get());
            if (rclcpp::spin_until_future_complete(node, future_result1) != rclcpp::FutureReturnCode::SUCCESS)
            {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, NO RESULT");
            return -1;
            }
            // se arrivo quì l'azione è terminata, controllo se è terminata con successo

            // check dello stato dell'azione, se non ho errori lo stato deve essere SUCCEEDED
            if (future_result1.get().code != rclcpp_action::ResultCode::SUCCEEDED)
            {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, JOINT TRAJECTORY NOT SUCCEEDED");
            return -1;
            }


            Media::Goal attivazione;
            attivazione.active=true;

            // chiamo l'azione e aspetto che termini
            auto future_goal_handle2 = calcola_media_action_client->async_send_goal(attivazione);
            if (rclcpp::spin_until_future_complete(node, future_goal_handle2) != rclcpp::FutureReturnCode::SUCCESS)
            {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
            return -1;
            }

            auto future_result2 = calcola_media_action_client->async_get_result(future_goal_handle2.get());
            if (rclcpp::spin_until_future_complete(node, future_result2) != rclcpp::FutureReturnCode::SUCCESS)
            {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, NO RESULT");
            return -1;
            }

            // check dello stato dell'azione, se non ho errori lo stato deve essere SUCCEEDED
            if (future_result2.get().code != rclcpp_action::ResultCode::SUCCEEDED)
            {
            RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, JOINT TRAJECTORY NOT SUCCEEDED");
            return -1;
            }

            //stampo la media
            RCLCPP_INFO_STREAM(node->get_logger(), "La media è "<<future_result2.get().result->media);
    }

  }
  
  rclcpp::shutdown();
  return 0;
}