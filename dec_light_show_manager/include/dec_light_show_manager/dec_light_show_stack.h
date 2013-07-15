/*
 * dec_light_show_stack.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_LIGHT_SHOW_STACK_H_
#define DEC_LIGHT_SHOW_STACK_H_

#include <vector>
#include <tr1/unordered_map>
#include <boost/shared_ptr.hpp>
#include <dec_light_show_manager/dec_light_show.h>

namespace dec_light_show_manager
{

class DecLightShowStack
{

public:
  DecLightShowStack(const std::string& name, const int id, const std::vector<int>& groups);
  virtual ~DecLightShowStack();

  bool start();
  bool update();
  bool stop();

  /**
   * Add the given light show to the bottom of the stack
   * @param light_show Light show to add
   */
  void addLightShow(boost::shared_ptr<DecLightShow> light_show);

  /**
   * Checks if this stack contains the named light_show.
   * @param name (in) name of the light show
   * @param light_show (out) pointer to the light show if it exists
   * @return True if the this stack contains the light_show, False otherwise
   */
  bool hasLightShow(const std::string& name, boost::shared_ptr<DecLightShow>& light_show) const;

  /**
   * Checks if this stack contains the light show by name
   * @param name (in) name of the light show
   * @return True if the this stack contains the light show, False otherwise
   */
  bool hasLightShow(const std::string& name) const;

  /*!
   * @return All light shows of the stack
   */
  std::vector<boost::shared_ptr<DecLightShow> > const& getLightShows() const;

  /**
   * Gets a "vector of int" indexed by light show id
   * Does not clear out light shows
   * @param light_shows
   */
  void getLightShows(std::vector<int>& light_shows) const;

  /*!
   * @return Name of the light show
   */
  std::string getName() const;
  /*!
   * @return Id of the light show
   */
  int getId() const;

  bool conflicts(boost::shared_ptr<DecLightShowStack> other) const;
  bool belongsToGroup(int group_id) const;

private:
  std::string name_;
  int id_;
  std::vector<int> groups_; /**< realy a vector of bools indicating membership in stack groups */
  std::vector<boost::shared_ptr<DecLightShow> > light_shows_;

  typedef std::tr1::unordered_map<std::string, boost::shared_ptr<DecLightShow> > LightShowMap;
  LightShowMap light_show_map_;

};

}

#endif /* DEC_LIGHT_SHOW_STACK_H_ */
