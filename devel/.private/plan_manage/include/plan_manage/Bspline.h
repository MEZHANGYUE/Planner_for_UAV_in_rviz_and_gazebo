// Generated by gencpp from file plan_manage/Bspline.msg
// DO NOT EDIT!


#ifndef PLAN_MANAGE_MESSAGE_BSPLINE_H
#define PLAN_MANAGE_MESSAGE_BSPLINE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace plan_manage
{
template <class ContainerAllocator>
struct Bspline_
{
  typedef Bspline_<ContainerAllocator> Type;

  Bspline_()
    : order(0)
    , traj_id(0)
    , start_time()
    , knots()
    , pos_pts()
    , yaw_pts()
    , yaw_dt(0.0)  {
    }
  Bspline_(const ContainerAllocator& _alloc)
    : order(0)
    , traj_id(0)
    , start_time()
    , knots(_alloc)
    , pos_pts(_alloc)
    , yaw_pts(_alloc)
    , yaw_dt(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _order_type;
  _order_type order;

   typedef int64_t _traj_id_type;
  _traj_id_type traj_id;

   typedef ros::Time _start_time_type;
  _start_time_type start_time;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _knots_type;
  _knots_type knots;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Point_<ContainerAllocator> >> _pos_pts_type;
  _pos_pts_type pos_pts;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _yaw_pts_type;
  _yaw_pts_type yaw_pts;

   typedef double _yaw_dt_type;
  _yaw_dt_type yaw_dt;





  typedef boost::shared_ptr< ::plan_manage::Bspline_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::plan_manage::Bspline_<ContainerAllocator> const> ConstPtr;

}; // struct Bspline_

typedef ::plan_manage::Bspline_<std::allocator<void> > Bspline;

typedef boost::shared_ptr< ::plan_manage::Bspline > BsplinePtr;
typedef boost::shared_ptr< ::plan_manage::Bspline const> BsplineConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::plan_manage::Bspline_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::plan_manage::Bspline_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::plan_manage::Bspline_<ContainerAllocator1> & lhs, const ::plan_manage::Bspline_<ContainerAllocator2> & rhs)
{
  return lhs.order == rhs.order &&
    lhs.traj_id == rhs.traj_id &&
    lhs.start_time == rhs.start_time &&
    lhs.knots == rhs.knots &&
    lhs.pos_pts == rhs.pos_pts &&
    lhs.yaw_pts == rhs.yaw_pts &&
    lhs.yaw_dt == rhs.yaw_dt;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::plan_manage::Bspline_<ContainerAllocator1> & lhs, const ::plan_manage::Bspline_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace plan_manage

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::plan_manage::Bspline_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plan_manage::Bspline_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plan_manage::Bspline_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plan_manage::Bspline_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_manage::Bspline_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_manage::Bspline_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::plan_manage::Bspline_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b352d4f7278a546180de67cbe6793e49";
  }

  static const char* value(const ::plan_manage::Bspline_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb352d4f7278a5461ULL;
  static const uint64_t static_value2 = 0x80de67cbe6793e49ULL;
};

template<class ContainerAllocator>
struct DataType< ::plan_manage::Bspline_<ContainerAllocator> >
{
  static const char* value()
  {
    return "plan_manage/Bspline";
  }

  static const char* value(const ::plan_manage::Bspline_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::plan_manage::Bspline_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 order\n"
"int64 traj_id\n"
"time start_time\n"
"\n"
"float64[] knots\n"
"geometry_msgs/Point[] pos_pts\n"
"\n"
"float64[] yaw_pts\n"
"float64 yaw_dt\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::plan_manage::Bspline_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::plan_manage::Bspline_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.order);
      stream.next(m.traj_id);
      stream.next(m.start_time);
      stream.next(m.knots);
      stream.next(m.pos_pts);
      stream.next(m.yaw_pts);
      stream.next(m.yaw_dt);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Bspline_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::plan_manage::Bspline_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::plan_manage::Bspline_<ContainerAllocator>& v)
  {
    s << indent << "order: ";
    Printer<int32_t>::stream(s, indent + "  ", v.order);
    s << indent << "traj_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.traj_id);
    s << indent << "start_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.start_time);
    s << indent << "knots[]" << std::endl;
    for (size_t i = 0; i < v.knots.size(); ++i)
    {
      s << indent << "  knots[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.knots[i]);
    }
    s << indent << "pos_pts[]" << std::endl;
    for (size_t i = 0; i < v.pos_pts.size(); ++i)
    {
      s << indent << "  pos_pts[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.pos_pts[i]);
    }
    s << indent << "yaw_pts[]" << std::endl;
    for (size_t i = 0; i < v.yaw_pts.size(); ++i)
    {
      s << indent << "  yaw_pts[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.yaw_pts[i]);
    }
    s << indent << "yaw_dt: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_dt);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLAN_MANAGE_MESSAGE_BSPLINE_H
