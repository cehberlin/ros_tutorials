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

#include "turtlesim/turtle_frame.h"

#include <QPointF>

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

#define DEFAULT_VIEW_DISTANCE 5

namespace turtlesim
{

TurtleFrame::TurtleFrame(QWidget* parent, Qt::WindowFlags f, int frame_width, int frame_height)
: QFrame(parent, f)
, path_image_(frame_width, frame_height, QImage::Format_ARGB32)
, path_painter_(&path_image_)
, frame_count_(0)
, id_counter_(0)
{
  setFixedSize(frame_width, frame_height);
  setWindowTitle("TurtleSim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  QVector<QString> turtles;
  turtles.append("box-turtle.png");
  turtles.append("robot-turtle.png");
  turtles.append("sea-turtle.png");
  turtles.append("diamondback.png");
  turtles.append("electric.png");
  turtles.append("fuerte.png");
  turtles.append("groovy.png");
  turtles.append("hydro.svg");
  turtles.append("indigo.svg");
  turtles.append("jade.png");

  QString images_path = (ros::package::getPath("turtlesim") + "/images/").c_str();
  for (int i = 0; i < turtles.size(); ++i)
  {
    QImage img;
    img.load(images_path + turtles[i]);
    turtle_images_.append(img);
  }

  meter_ = turtle_images_[0].height();

  clear();

  clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
  reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
  spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
  spawn_grad_srv_ = nh_.advertiseService("spawn_grad", &TurtleFrame::spawnGradCallback, this);
  spawn_img_srv_ = nh_.advertiseService("spawn_img", &TurtleFrame::spawnImgCallback, this);
  draw_ellipse_srv_ = nh_.advertiseService("draw_ellipse", &TurtleFrame::drawEllipseCallback, this);

  kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);

  ROS_INFO("Starting turtlesim with node name %s", ros::this_node::getName().c_str()) ;

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;

  bool with_initial_turtle;
  nh_.param("with_initial_turtle", with_initial_turtle, true);
  if(with_initial_turtle){
    spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  }

  nh_.param("draw_name", this->draw_name_, true);

  // spawn all available turtle types
  if(FALSE)
  {
    for(int index = 0; index < turtles.size(); ++index)
    {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool TurtleFrame::spawnGradCallback(turtlesim::SpawnGrad::Request& req, turtlesim::SpawnGrad::Response& res)
{
  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta, req.goal_radius, req.total_radius);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool TurtleFrame::spawnImgCallback(turtlesim::SpawnImg::Request& req, turtlesim::SpawnImg::Response& res)
{
  QString images_path = (ros::package::getPath("turtlesim") + "/images/" + req.img_name).c_str();
  QImage img;
  img.load(images_path);

  std::string name = spawnTurtle(req.name, req.x, req.y, req.theta, img, req.with_collision);
  if (name.empty())
  {
    ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
    return false;
  }

  res.name = name;

  return true;
}

bool TurtleFrame::drawEllipseCallback(turtlesim::DrawEllipse::Request& req, turtlesim::DrawEllipse::Response& res)
{
  // update gradient vector
  ellipse = req.ellipses;

  res.name = "ellipse";

  update(); // trigger paint event

  return true;
}


bool TurtleFrame::killCallback(turtlesim::Kill::Request& req, turtlesim::Kill::Response&)
{
  M_Turtle::iterator it = turtles_.find(req.name);
  if (it == turtles_.end())
  {
    ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

bool TurtleFrame::hasTurtle(const std::string& name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index){
  return spawnTurtle(name, x, y, angle, turtle_images_[index], true, 0, 0);
}


std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, float goal_radius, float total_radius){
  return spawnTurtle(name, x, y, angle, turtle_images_[rand() % turtle_images_.size()], true, goal_radius, total_radius);
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, QImage& img, bool with_collision){
    return spawnTurtle(name, x, y, angle, img, with_collision, 0.0, 0.0);
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, QImage& img, bool with_collision = true, float goal_radius = 0.0, float total_radius = 0.0)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  }
  else
  {
    if (hasTurtle(real_name))
    {
      return "";
    }
  }

  float view_distance = DEFAULT_VIEW_DISTANCE;

  nh_.param("view_distance", view_distance, view_distance );

  TurtlePtr t(new Turtle(ros::NodeHandle(real_name), img, QPointF(x, height_in_meters_ - y), angle, view_distance, with_collision, this->draw_name_, goal_radius, total_radius));
  turtles_[real_name] = t;
  update();

  ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

  nh_.param("background_r", r, r);
  nh_.param("background_g", g, g);
  nh_.param("background_b", b, b);

  path_image_.fill(qRgb(r, g, b));
  update();
}

void TurtleFrame::onUpdate()
{
  ros::spinOnce();

  updateTurtles();

  if (!ros::ok())
  {
    close();
  }
}

void TurtleFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_);

  for(std::vector<turtlesim::Ellipse >::iterator it=ellipse.begin(); it != ellipse.end(); ++it)
  {
    QPointF pCircle = QPoint((*it).x * meter_, (height_in_meters_ - (*it).y) * meter_);

    QPen pen = painter.pen();
    pen.setWidth((*it).width);
    pen.setColor(QColor((*it).r, (*it).g, (*it).b));

    if((*it).solid == true)
        pen.setStyle(Qt::PenStyle(Qt::SolidLine));
    else if((*it).solid == false)
        pen.setStyle(Qt::PenStyle(Qt::DashLine));

    painter.setPen(pen);

    // draw Gradient
    painter.drawEllipse(pCircle, (*it).radius *meter_, (*it).radius *meter_);
  }

  // draw turtles
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.isZero())
  {
    last_turtle_update_ = ros::WallTime::now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(turtles_, 0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}


bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}
