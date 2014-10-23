#ifndef CROSSING_DETECTOR_WRAPPER_UTILS_H
#define CROSSING_DETECTOR_WRAPPER_UTILS_H

#include <string>

#include <boost/python.hpp>

#include <ros/serialization.h>

namespace rs = ros::serialization;

// Extracted from https://gist.github.com/avli/b0bf77449b090b768663.
template<class T>
struct vector_to_python
{
  static PyObject* convert(const std::vector<T>& vec)
  {
    boost::python::list* l = new boost::python::list();
    for(std::size_t i = 0; i < vec.size(); i++)
      (*l).append(vec[i]);

    return l->ptr();
  }
};

/* Read a ROS message from a serialized string.
  */
template <typename M>
M from_python(const std::string str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  rs::IStream stream(buffer.get(), serial_size);
  M msg;
  rs::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string.
*/
  template <typename M>
std::string to_python(const M& msg)
{
  size_t serial_size = rs::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  rs::OStream stream(buffer.get(), serial_size);
  rs::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i)
  {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

#endif // CROSSING_DETECTOR_WRAPPER_UTILS_H


