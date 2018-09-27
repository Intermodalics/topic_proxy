#include <topic_proxy/server.h>

int main(int argc, char **argv)
{
  // add __tcpros_server_port to remappings
  std::string tcpros_server_port_argv = "__tcpros_server_port:=" + boost::lexical_cast<std::string>(topic_proxy::g_default_port);
  std::vector<char *> new_argv;
  new_argv.reserve(argc + 1);
  new_argv.assign(&argv[0], &argv[argc]);
  new_argv.push_back(const_cast<char *>(tcpros_server_port_argv.c_str()));
  argc = new_argv.size();
  argv = new_argv.data();

  ros::init(argc, argv, "topic_proxy_server");
  topic_proxy::Server server;
  ROS_INFO("Created topic_proxy server listening on %s:%u", server.getHost().c_str(), server.getTCPPort());
  ros::spin();
  return 0;
}
