/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TURTLESIM_TURTLE_H
#define TURTLESIM_TURTLE_H

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/ros.h>
# include <boost/shared_ptr.hpp>

# include <turtlesim/Pose.h>
# include <turtlesim/PoseArray.h>
# include <geometry_msgs/Twist.h>
# include <turtlesim/SetPen.h>
# include <turtlesim/TeleportRelative.h>
# include <turtlesim/TeleportAbsolute.h>
# include <turtlesim/Color.h>
#endif

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

#include <map>

#define PI 3.14159265

namespace turtlesim
{

class Turtle; //forward declrartion for typedefs

typedef boost::shared_ptr<Turtle> TurtlePtr;
typedef std::map<std::string, TurtlePtr> M_Turtle;

class Turtle
{
public:
  Turtle(const ros::NodeHandle& nh, const QImage& turtle_image, const QPointF& pos, float orient, float view_distance, bool with_collision, bool draw_name, float goal_radius, float total_radius);

  bool update(M_Turtle& turtles, double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height);
  void paint(QPainter &painter);

  Pose getPose(qreal canvas_width, qreal canvas_height);

  bool isCollidable();

private:
  void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
  bool setPenCallback(turtlesim::SetPen::Request&, turtlesim::SetPen::Response&);
  bool teleportRelativeCallback(turtlesim::TeleportRelative::Request&, turtlesim::TeleportRelative::Response&);
  bool teleportAbsoluteCallback(turtlesim::TeleportAbsolute::Request&, turtlesim::TeleportAbsolute::Response&);

  void rotateImage();

  ros::NodeHandle nh_;

  QImage turtle_image_;
  QImage turtle_rotated_image_;

  QPointF pos_;
  qreal orient_;

  qreal lin_vel_;
  qreal ang_vel_;
  bool pen_on_;
  QPen pen_;

  ros::Subscriber velocity_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher other_pose_pub_;
  ros::Publisher view_distance_pub_;
  ros::Publisher color_pub_;
  ros::ServiceServer set_pen_srv_;
  ros::ServiceServer teleport_relative_srv_;
  ros::ServiceServer teleport_absolute_srv_;

  ros::WallTime last_command_time_;

  float meter_;

  float view_distance_;
  bool views_other_;
  bool with_collision_; //if true other can collide with it and it also checks for collision
  float goal_radius_;
  float total_radius_;

  bool draw_name_; //flag if the namespace/name is drawn too.

  struct TeleportRequest
  {
    TeleportRequest(float x, float y, qreal _theta, qreal _linear, bool _relative)
    : pos(x, y)
    , theta(_theta)
    , linear(_linear)
    , relative(_relative)
    {}

    QPointF pos;
    qreal theta;
    qreal linear;
    bool relative;
  };
  typedef std::vector<TeleportRequest> V_TeleportRequest;
  V_TeleportRequest teleport_requests_;
};
}

#endif
