/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "ActorPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

// ros actor groundtruth publisher definition


ros::NodeHandle n;

ros::Publisher actorposePub = n.advertise<nav_msgs::Odometry>("actorpose", 1000);
ros::Publisher actorposePubNWU = n.advertise<nav_msgs::Odometry>("actorposeNWU", 1000);

ros::Rate loop_rate(10);
nav_msgs::Odometry actorPose;

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  int argc; char **argv;
  // ros::init(argc, argv,"actorpose_node"); //initialize actor pose node

  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity = 1.4;
  this->lastUpdate = 0;
  this->start = this->world->SimTime().Double();

  // if (this->sdf && this->sdf->HasElement("target"))
  //   this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  // else
    this->target = ignition::math::Vector3d(0, 0, walking_height);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void ActorPlugin::ChooseNewTarget()
{
  duration = ( this->world->SimTime().Double() - this->start ) ;
  ignition::math::Vector3d newTarget(this->target);
  if (duration < 5)
  {
    newTarget.X(0) ;
    newTarget.Y(0) ;
    newTarget.Z(walking_height) ;
    if ((newTarget - this->target).Length() < 1.0)
    {
       this->velocity = 0.01;
    }
  }
  else
  {
  while ((newTarget - this->target).Length() < 1.0)
  {
    this->velocity  = 1.0;
    // this->velocity  = (ignition::math::Rand::DblUniform(0.5,1.5));
    // newTarget.X(ignition::math::Rand::DblUniform(-10, 10));
    // newTarget.Y(ignition::math::Rand::DblUniform(-10, 10));
    newTarget.X(x_waypoints[counter % x_waypoints.size()]);
    newTarget.Y(y_waypoints[counter % y_waypoints.size()]);

    if(this->USE_HEIGHT_MAP){
      newTarget.Z(1.0+heightMapZ(newTarget.X(),newTarget.Y()));
      }
    else {
      newTarget.Z(walking_height);
    }
    // newTarget.Y(0);
    // newTarget.Z(1.1);
    // printf("x:%f,y:%f,z:%f\n",newTarget.X(),newTarget.Y(),newTarget.Z());
    counter  = counter + 1;

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
}
  this->target = newTarget;
}

/////////////////////////////////////////////////
void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

double ActorPlugin::heightMapZ(double x, double y)
{

   // getting the pointer to the HeightmapShape
   // physics::WorldPtr world = this->model->GetWorld();
   physics::ModelPtr model = this->world->ModelByName("ground_texture");
   physics::CollisionPtr collision = model->GetLink("link")->GetCollision("collision");
   physics::HeightmapShapePtr heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());

   // coordinate transform from regular Word (x,y) to the HeightmapShape (index_x,index_y)
   ignition::math::Vector3d size = heightmap->Size();
   ignition::math::Vector2i vc = heightmap->VertexCount();
   int index_x = (  ( (x + size.X()/2)/size.X() ) * vc.X() - 1 ) ;
   int index_y = (  ( (-y + size.Y()/2)/size.Y() ) * vc.Y() - 1 ) ;
   //
   // //  getting the height :
   double z =  heightmap->GetHeight( index_x , index_y ) ;
   // double z = 0.1;
   // //
   return z;
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();
  ignition::math::Vector3d applied_control = this->target - pose.Pos();

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 1)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  // this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  // if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  // {
  //   pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
  //       yaw.Radian()*0.001);
  // }
  // else
  // {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  // }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(-20.0, std::min(20.0, pose.Pos().X())));
  pose.Pos().Y(std::max(-20.0, std::min(20.0, pose.Pos().Y())));
  if (this->USE_HEIGHT_MAP){
    pose.Pos().Z(1.0+heightMapZ(pose.Pos().X(),pose.Pos().Y()));
  }
  else{
    pose.Pos().Z(walking_height);
  }

  // pose.Pos().Y(0);
  // pose.Pos().Z(std::max(-30.0, std::min(30.0, pose.Pos().Z())));

  //


  //ros publish groundtruth position
  actorPose.pose.pose.position.x =  pose.Pos().Y(); //X is -Y HACK - coordinate transforms
  actorPose.pose.pose.position.y =  pose.Pos().X(); //Y is X HACK - coordinate transforms
  actorPose.pose.pose.position.z = -pose.Pos().Z(); // Z is -Z
  actorPose.pose.pose.orientation.x = pose.Rot().X();
  actorPose.pose.pose.orientation.y = pose.Rot().Y();
  actorPose.pose.pose.orientation.z = pose.Rot().Z();
  actorPose.pose.pose.orientation.w = pose.Rot().W();
  actorPose.header.stamp = ros::Time::now();
  actorPose.header.frame_id = "world";

  applied_control = pos * this->velocity;
  //ros publish groundtruth position
  actorPose.twist.twist.linear.x =  applied_control.Y(); //X is Y HACK - coordinate transforms
  actorPose.twist.twist.linear.y =  applied_control.X(); //Y is X HACK - coordinate transforms
  actorPose.twist.twist.linear.z =  -applied_control.Z(); // Z is -Z


  actorposePub.publish(actorPose);
  actorPose.pose.pose.position.x =  pose.Pos().X();
  actorPose.pose.pose.position.y =  pose.Pos().Y();
  actorPose.pose.pose.position.z =  pose.Pos().Z();
  actorposePubNWU.publish(actorPose);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
