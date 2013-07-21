/*
 * dec_circular_buffer.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_CIRCULAR_BUFFER_H_
#define DEC_CIRCULAR_BUFFER_H_

// system includes
#include <vector>
#include <ros/ros.h>
#include <stdint.h>
#include <dec_utilities/assert.h>

#include <algorithm>
#include <boost/circular_buffer.hpp>

namespace dec_light_shows
{

/** \brief A realtime safe circular (ring) buffer.
 */
template<typename T>
  class DecCircularBuffer
  {

  private:
    DecCircularBuffer();

  public:
    DecCircularBuffer(int size, const T& default_val) :
        counter_(0), cb_(size)
    {
      ROS_ASSERT(cb_.capacity() > 0);
      for (unsigned int i = 0; i < cb_.capacity(); i++)
      {
        cb_.push_back(default_val);
      }
    }

    void push_back(const T& item)
    {
      if (counter_ < cb_.size())
      {
        cb_[counter_] = item;
      }
      else
      {
        cb_.push_back(item);
      }
      counter_++;
    }
    void push_front(const T& item)
    {
      cb_.push_front(item);
      counter_++;
    }

    void clear()
    {
      counter_ = 0;
    }

    T& front()
    {
      return cb_.front();
    }

    T& back()
    {
      if (counter_ < cb_.size())
      {
        return cb_[counter_];
      }
      else
      {
        return cb_.back();
      }
    }

    bool get(std::vector<T>& data)
    {
      if (data.size() != cb_.size())
      {
        return false;
      }
      typename boost::circular_buffer<T>::const_iterator ci;
      int i = 0;
      for (ci = cb_.begin(); ci != cb_.end(); ++ci)
      {
        data[i] = *ci;
        i++;
      }
      return true;
    }
    unsigned int size()
    {
      return std::min(counter_, (unsigned int)cb_.size());
    }

    unsigned int capacity()
    {
      return (unsigned int)cb_.capacity();
    }

    bool empty()
    {
      return cb_.empty();
    }

    T& at(size_t index)
    {
      return cb_.at(index);
    }

    T& operator[](size_t index)
    {
      return cb_[index];
    }

  private:
    unsigned int counter_; // <! special counter to keep track of first N times through
    boost::circular_buffer<T> cb_;
  };

}

#endif /* DEC_CIRCULAR_BUFFER_H_ */
