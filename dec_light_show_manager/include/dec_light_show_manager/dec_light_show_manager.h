/*
 * dec_light_show_manager.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_MANAGER_H_
#define DEC_LIGHT_SHOW_MANAGER_H_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include <dec_light_show_manager/dec_mutex.h>
#include <dec_light_show_manager/dec_light_show.h>
#include <dec_light_show_manager/dec_light_show_factory.h>
#include <dec_light_show_manager/dec_light_show_stack.h>
#include <dec_light_show_manager/dec_light_show_simulation.h>
#include <dec_light_show_manager/dec_light_show_visualization.h>

#include <dec_light_show_msgs/SwitchLightShowStack.h>

namespace dec_light_show_manager
{

class DecLightShowManager
{

public:
  DecLightShowManager();
  virtual ~DecLightShowManager();

  /*!
   * @return True on success, otherwise False
   */
  bool initialize();

  /*! This runs the manager
   */
  void run();

  /*!
   * @return True on success, otherwise False
   */
  bool update();

  /*!
   * Assigns next_light_show_stack_ by name, and sets the switching_light_shows_ flag
   * @param light_show_stack_names
   * @return True on succes, otherwise False
   */
  bool switchLightShowStack(const std::vector<std::string>& light_show_stack_names);

  /**
   * Reloads all light show parameters and reinitializes light shows
   */
  void reloadLightShows();

  /**
   * Returns the names of all available light show stacks
   * @param names
   */
  void getLightShowStackNames(std::vector<std::string>& names) const;

  /**
   * Gets a pointer to a light show by name
   */
  bool getLightShowByName(const std::string& name, boost::shared_ptr<DecLightShow>& light_show);

private:

  bool initialized_;

  boost::shared_ptr<DecLightShowFactory> light_show_factory_;
  typedef std::tr1::unordered_map<std::string, boost::shared_ptr<DecLightShow> > LightShowMap;
  typedef std::tr1::unordered_map<std::string, boost::shared_ptr<DecLightShowStack> > LightShowStackMap;
  LightShowMap light_shows_;
  std::vector<boost::shared_ptr<DecLightShow> > light_show_list_;
  LightShowStackMap light_show_stacks_;
  std::vector<boost::shared_ptr<DecLightShowStack> > light_show_stack_list_;

  int num_light_show_stacks_;
  int num_light_shows_;

  bool light_shows_running_;
  std::vector<std::string> light_show_stack_names_;

  std::vector<boost::shared_ptr<DecLightShowStack> > active_light_show_stacks_; /**< Currently running list of stacks, ordered */
  std::vector<boost::shared_ptr<DecLightShowStack> > next_light_show_stacks_;   /**< List of stacks which will run after the stack switch */

  std::vector<int> stack_active_list_; /**< vector of bool: currently active stacks */
  std::vector<int> stack_next_active_list_; /**< vector of bool: stacks active after the switch */
  std::vector<int> stack_insert_list_; /**< vector of bool: Stacks to insert */
  std::vector<int> stack_insert_list_tmp_;

  std::vector<int> light_show_active_list_; /**< vector of bool: light_shows currently active */
  std::vector<int> light_show_next_active_list_; /**< vector of bool: light_shows going to be active after switch */
  std::vector<int> light_show_stop_list_; /**< vector of bool: light_shows to stop */
  std::vector<int> light_show_start_list_; /**< vector of bool: light_shows to start */

  bool switching_light_shows_;
  bool switching_light_shows_success_;

  dec_mutex switching_light_shows_mutex_;
  dec_cond switching_light_shows_cond_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle local_node_handle_;

  boost::shared_ptr<DecLightShowData> light_show_data_;

  std::vector<std::string> stack_groups_;

  bool readLightShows();
  bool readLightShowStacks();

  ros::ServiceServer switch_light_show_stack_service_;

  bool getLightShowStackByName(const std::string& name, boost::shared_ptr<DecLightShowStack>& light_show);
  void publishLightShowStatus(int code);
  bool switchLightShowStackService(dec_light_show_msgs::SwitchLightShowStack::Request& request,
                                      dec_light_show_msgs::SwitchLightShowStack::Response& response);
  bool loadLightShows();

  /**
   * Expects active_light_show_stacks_, stack_active_list_, and stack_insert_list_ to be valid.
   * Updates light_show_start_list_, light_show_stop_list_, next_light_show_stacks_, and stack_active_list_
   * @return
   */
  bool processLightShowSwitch();

  ros::Time start_ros_time_;
  double start_ros_time_in_sec_;

  void debugLightShowList(const std::vector<int>& light_shows);
  void debugLightShowStackList(const std::vector<int>& light_shows);

  bool simulation_mode_;
  boost::shared_ptr<DecLightShowSimulation> dec_light_show_simulation_;
  bool copySensorInformationFromStructure();
  bool copyLightDataToStructure();

  boost::shared_ptr<DecLightShowVisualization> dec_light_show_visualization_;

  void setDefaultStacks();

};

}

#endif /* DEC_LIGHT_SHOW_MANAGER_H_ */
