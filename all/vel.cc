#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <unistd.h>

int main(int _argc, char **_argv)
{
  // Load gazebo as a client
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the velodyne topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("~/my_velodyne/vel_cmd");

  // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();

  // Create a a vector3 message
  gazebo::msgs::Vector3d msg;

  // Send a 3D vector
  // You can set the velocity, position, etc.
  gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), std::atof(_argv[2]), std::atof(_argv[3])));

  // Send the message
  pub->Publish(msg);

  // for (double i = 0; true; i += 0.01) {
  //   // Send a 3D vector
  //   // You can set the velocity, position, etc.
  //   gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]), std::atof(_argv[2]) + i, std::atof(_argv[3])));

  //   // Send the message
  //   pub->Publish(msg);

  //   usleep(100 * 1000);
  // }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
