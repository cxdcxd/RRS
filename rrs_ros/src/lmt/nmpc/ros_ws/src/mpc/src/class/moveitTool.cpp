#include<moveitTool.h>
#include<chrono>
MoveitTool::MoveitTool(ros::NodeHandle n):nh(n)
{
  // initialization
  // robot model
//  ROS_INFO("Start moveittool instance initilization");
  robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
  robot_model = robot_model_loader.getModel();
  arm_jmg = robot_model->getJointModelGroup("arm");
  model_frame = robot_model->getModelFrame();

  global_frame = "origin";
  // end effector link
  eef_link = robot_model->getLinkModel("j2s7s300_end_effector");

//  link_model_ptrs = robot_model->getLinkModelsWithCollisionGeometry();
  link_model_ptrs = arm_jmg->getLinkModels();
  // ignore the last 6 links which are fingers.   // last link that is end effector virtual link
  link_model_ptrs.erase(link_model_ptrs.begin()+8,link_model_ptrs.end());
  // may be also good to ignore the first base link
  link_model_ptrs.erase(link_model_ptrs.begin(),link_model_ptrs.begin()+1);
  joint_names = arm_jmg->getVariableNames();
  joint_num = joint_names.size();
//  ROS_INFO("Valid link of the robot:");
//  for(auto link:link_model_ptrs)
//    std::cout << link->getName() << std::endl;

  // initialize robot current state of the planning scene(some default values)
  current_state = std::make_shared<robot_state::RobotState>(robot_model);
  current_state->setToDefaultValues();


  //build a  cylinder collision object for robot link76 and link54
  link_shape = std::make_shared<shapes::Cylinder>(0.05,0.3);
  std::shared_ptr<fcl::CollisionGeometryd> link_geometry;
  buildGeometry(link_shape,link_geometry);
  auto crobot76 = new fcl::CollisionObjectd(link_geometry);
  crobot_ptrs.push_back(crobot76);
//  std::cout <<"Construct collision object for link7 and link6 " << std::endl;
  auto crobot54 = new fcl::CollisionObjectd(link_geometry);
  crobot_ptrs.push_back(crobot54);
//  std::cout <<"Construct collision object for link5 and link4 " << std::endl;


  // cylinder32 is longer
  link_shape32 = std::make_shared<shapes::Cylinder>(0.05,0.5);
  buildGeometry(link_shape32,link_geometry);
  auto crobot32 = new fcl::CollisionObjectd(link_geometry);
  crobot_ptrs.push_back(crobot32);
  std::cout <<"Construct collision object for link3 and link2 " << std::endl;
  // initialize centroids and z-axes for three cylinders.
  centroids.resize(3);
  x_axes.resize(3);
  y_axes.resize(3);
  z_axes.resize(3);



//  // collision for robot
//  for(const robot_model::LinkModel* link:link_model_ptrs)
//  {
//    // ignore the j2s7s300 link 1, since it is base link and can not move
//    if(link->getName()=="j2s7s300_link_1")
//      continue;
//    std::cout <<"Construct collision object for link "<<link->getName() << std::endl;
//    shapes::ShapeConstPtr link_shape = link->getShapes()[0];
//    std::shared_ptr<fcl::CollisionGeometryd> link_geometry;
//    buildGeometry(link_shape,link_geometry);
//    auto crobot = new fcl::CollisionObjectd(link_geometry);
//    crobot_ptrs.push_back(crobot);
//  }

  // initialize marker related;
  // publisher
    marker_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   ros::WallDuration sleep_t(0.5);
   // while(marker_publisher.getNumSubscribers() < 1)
   // {
   //   sleep_t.sleep();
   // }


  // distance marker
  line_strip.header.frame_id = model_frame;
  line_strip.id = 0;
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.005;
  line_strip.color.a = 1.0;
  line_strip.color.b = 1.0;


  // obstacle marker
  obstacle_marker.header.frame_id = model_frame;
  obstacle_marker.color.b = 1.0;
  obstacle_marker.color.a = 0.7;
  obstacle_marker.id = 0;

  obstacle_marker.action = visualization_msgs::Marker::ADD;

  // distance reequest
  dist_request.enable_nearest_points = true;
  dist_request.enable_signed_distance = true;



  ROS_INFO("Moveit tool Initialization completed.");
}

void MoveitTool::updateJointState(const std::vector<double>& joint_values,bool update_co,bool update_cylinder_pose)
{
//  auto t1 = std::chrono::high_resolution_clock::now();
  current_state->setJointGroupPositions(arm_jmg,joint_values);
  // update robot collision object pose
  if (update_co)
  {
      auto link_transform7 = current_state->getGlobalLinkTransform("j2s7s300_link_7");
      Eigen::Affine3d collision_transform = link_transform7*Eigen::Translation3d(0,0,-0.05);
      const auto& cylinder1_translation = collision_transform.translation();
      const auto& cylinder1_rotation = collision_transform.rotation();

      crobot_ptrs[0]->setTransform(cylinder1_rotation,cylinder1_translation);


//     obstacle_marker.color.a = 0.5;
//     obstacle_marker.color.g = 1.0;
//     tf::poseEigenToMsg(collision_transform,obstacle_marker.pose);
//     shapes::constructMarkerFromShape(link_shape.get(),obstacle_marker);
//     obstacle_marker.ns="collision76";
//     marker_publisher.publish(obstacle_marker);
      auto link_transform5 = current_state->getGlobalLinkTransform("j2s7s300_link_5");
      const auto& cylinder2_translation = link_transform5.translation();
      const auto& cylinder2_rotation = link_transform5.rotation();
      crobot_ptrs[1]->setTransform(cylinder2_rotation,cylinder2_translation);
//     tf::poseEigenToMsg(link_transform5,obstacle_marker.pose);
//     obstacle_marker.ns="collision54";
//     marker_publisher.publish(obstacle_marker);


      auto link_transform3 = current_state->getGlobalLinkTransform("j2s7s300_link_3");
      const auto& cylinder3_translation = link_transform3.translation();
      const auto& cylinder3_rotation = link_transform3.rotation();
      crobot_ptrs[2]->setTransform(cylinder3_rotation,cylinder3_translation);
//     tf::poseEigenToMsg(link_transform3,obstacle_marker.pose);
//     shapes::constructMarkerFromShape(link_shape32.get(),obstacle_marker);
//     obstacle_marker.ns="collision32";
//     marker_publisher.publish(obstacle_marker);

     // auto t2 = std::chrono::high_resolution_clock::now();
     // auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
     // std::cout << "duration:" << duration << std::endl;
//     obstacle_marker.color.a = 1;
//     obstacle_marker.color.g = 0;

      if(update_cylinder_pose)
      {
        centroids={cylinder1_translation,cylinder2_translation,cylinder3_translation};
        x_axes={cylinder1_rotation.block<3,1>(0,0),cylinder2_rotation.block<3,1>(0,0),cylinder3_rotation.block<3,1>(0,0)};
        y_axes={cylinder1_rotation.block<3,1>(0,1),cylinder2_rotation.block<3,1>(0,1),cylinder3_rotation.block<3,1>(0,1)};
        z_axes={cylinder1_rotation.block<3,1>(0,2),cylinder2_rotation.block<3,1>(0,2),cylinder3_rotation.block<3,1>(0,2)};
      }
  }
}



const Eigen::Isometry3d& MoveitTool::getEEFTransform() const
{
  return current_state->getGlobalLinkTransform(eef_link);
}

const Eigen::MatrixXd MoveitTool::getJacobian(const bool use_quat) const
{
  Eigen::MatrixXd jacobian_matrix;
  Eigen::Vector3d ref_point_pos = Eigen::Vector3d::Zero();
  current_state->getJacobian(arm_jmg,eef_link,ref_point_pos,jacobian_matrix,use_quat);
  return jacobian_matrix;
}
void MoveitTool::getjacobian(const int link_index, Eigen::MatrixXd &jacobian,
                             const Eigen::Vector3d &ref_point_pos
                             ) const
{
  int index_in_robot = 5-link_index*2;
  auto link_transform = current_state->getGlobalLinkTransform(link_model_ptrs[index_in_robot]);
  Eigen::Vector3d local_pos0 = ref_point_pos - link_transform.translation();
  Eigen::Vector3d local_pos7 = link_transform.linear().transpose()*local_pos0;
  current_state->getJacobian(arm_jmg,link_model_ptrs[index_in_robot],local_pos7,jacobian,false);
}


void MoveitTool::buildGeometry(const shapes::ShapeConstPtr& obj_shape,std::shared_ptr<fcl::CollisionGeometryd>& co)
{
  if (obj_shape->type == shapes::PLANE)  // shapes that directly produce CollisionGeometry
  {
    // handle cases individually
    switch (obj_shape->type)
    {
      case shapes::PLANE:
      {
        const shapes::Plane* p = static_cast<const shapes::Plane*>(obj_shape.get());
        // fcl plane
        co = std::make_shared<fcl::Planed>(p->a, p->b, p->c, p->d);
      }
      break;
    default:
      break;
    }

  }
  else
  {
    switch(obj_shape->type){
      case shapes::MESH:
      {
        auto g = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
        const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(obj_shape.get());
        if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
        {
          std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
          for (unsigned int i = 0; i < mesh->triangle_count; ++i)
            tri_indices[i] = fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

          std::vector<fcl::Vector3d> points(mesh->vertex_count);
          for (unsigned int i = 0; i < mesh->vertex_count; ++i)
            points[i] = fcl::Vector3d(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

          g->beginModel();
          g->addSubModel(points, tri_indices);
          g->endModel();

        }
        co = g;
      }
      break;

      case shapes::BOX:
      {
         const shapes::Box* s = static_cast<const shapes::Box*>(obj_shape.get());
         const double* size = s->size;
         co = std::make_shared<fcl::Boxd>(size[0], size[1], size[2]);
      }
      break;

      case shapes::SPHERE:
      {
        const shapes::Sphere* s = static_cast<const shapes::Sphere*>(obj_shape.get());
        co = std::make_shared<fcl::Sphered>(s->radius);
      }
      break;

      case shapes::CYLINDER:
      {
        const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(obj_shape.get());
        co = std::make_shared<fcl::Cylinderd>(s->radius, s->length);
      }
      break;

      case shapes::CONE:
      {
        const shapes::Cone* s = static_cast<const shapes::Cone*>(obj_shape.get());
         co = std::make_shared<fcl::Coned>(s->radius, s->length);
      }
      break;

      case shapes::OCTREE:
      {
        const shapes::OcTree* g = static_cast<const shapes::OcTree*>(obj_shape.get());
        co = std::make_shared<fcl::OcTreed>(g->octree);
      }
      break;

      default:
        ROS_ERROR("This shape type (%d) is not supported using FCL yet", (int)obj_shape->type);
        co = NULL;

    }


  }
  if(co)
  {
    co->computeLocalAABB();
  }





}

void MoveitTool::addObstacle(const std::string& obj_name,const shapes::ShapeConstPtr& obj_shape, const Eigen::Affine3d& obj_transform, const bool plot)
{
  // construct collision object
  std::shared_ptr<fcl::CollisionGeometryd> co_geometry;
  buildGeometry(obj_shape,co_geometry);
  auto co = std::make_shared<fcl::CollisionObjectd>(co_geometry, obj_transform.rotation(),obj_transform.translation());
  co_ptrs.insert(std::pair< std::string,std::shared_ptr<fcl::CollisionObjectd> >(obj_name,co));
  shape_ptrs.insert(std::pair<std::string,shapes::ShapeConstPtr>(obj_name,obj_shape));

  if(plot)
  {
    // send obstacle update msg to rviz planning scene
    obstacle_marker.action = visualization_msgs::Marker::ADD;
    obstacle_marker.ns = obj_name;
    tf::poseEigenToMsg(obj_transform,obstacle_marker.pose);
    shapes::constructMarkerFromShape(obj_shape.get(),obstacle_marker);
    marker_publisher.publish(obstacle_marker);
    ROS_INFO_STREAM(obj_name <<" added.");
  }
}


void MoveitTool::updateObstacle(const std::string& obj_name,const Eigen::Affine3d& obj_transform, const bool plot)
{
  // update fcl collision object stored in co_ptrs;
  co_ptrs[obj_name]->setTransform(obj_transform.rotation(),obj_transform.translation());
  // update obstacles in rviz
  if(plot)
  {
    obstacle_marker.ns = obj_name;
    obstacle_marker.action = visualization_msgs::Marker::MODIFY;
    tf::poseEigenToMsg(obj_transform,obstacle_marker.pose);
    shapes::constructMarkerFromShape(shape_ptrs[obj_name].get(),obstacle_marker);
    marker_publisher.publish(obstacle_marker);
  }
}

void MoveitTool::removeObstacle(const std::string& obj_name)
{

  // remove fcl collision object stored in co_ptrs;
  co_ptrs.erase(obj_name);


  // remove obstacles in rviz and shape stored in shape_ptrs;
  //  shape_ptrs.erase(obj_name);
  //  obstacle_marker.ns = obj_name;
  //  obstacle_marker.action = visualization_msgs::Marker::DELETE;
  //  marker_publisher.publish(obstacle_marker);
}

int MoveitTool::getObstacleNum()
{
  return co_ptrs.size();
}


bool MoveitTool::defaultDistanceFunction(fcl::CollisionObjectd *o1, fcl::CollisionObjectd *o2, void *cdata_, double &dist)
{
  DistanceData* cdata = static_cast<DistanceData*>(cdata_);
  auto& request = cdata->request;
  auto& result = cdata->result;
  fcl::DistanceResultd temp;
  fcl::distance(o1, o2, request, temp);
  result.insert(std::pair<fcl::CollisionObjectd*,fcl::DistanceResultd>(o1,temp));
  return cdata->done;
}

void MoveitTool::getDistsAndPoints(std::vector< std::vector<double> >& dists,
                       std::vector< std::vector<Eigen::Vector3d> >& points1,
                       std::vector< std::vector<Eigen::Vector3d> >& points2)
{
  int co_ptrs_size = co_ptrs.size();
  dists.resize(co_ptrs_size);
  points1.resize(co_ptrs_size);
  points2.resize(co_ptrs_size);

  for(auto& co:co_ptrs)
  {

//    distance_data.result.clear();
//    manager->distance(co.second.get(),&distance_data,defaultDistanceFunction);
    std::vector<double> dists_co(crobot_ptrs.size());
    std::vector<Eigen::Vector3d> points1_co(crobot_ptrs.size());
    std::vector<Eigen::Vector3d> points2_co(crobot_ptrs.size());
    for(int i=0;i<crobot_ptrs.size();i++)
    {
      fcl::DistanceResultd dist_result;
      fcl::distance(crobot_ptrs[i], co.second.get(),dist_request,dist_result);
      fcl::Vector3d point1 = dist_result.nearest_points[0];
      fcl::Vector3d point2 = dist_result.nearest_points[1];
      if(co.second->getObjectType()!=fcl::OT_GEOM)
      {
        if(dist_result.min_distance>=0)
        {
          fcl::Vector3d temp_point = point1;
          point1 = point2;
          point2 = temp_point;
        }
        else
        {
          fcl::CollisionRequestd collision_req;
          collision_req.enable_contact=true;
          collision_req.num_max_contacts=100;
          fcl::CollisionResultd collision_res;

          fcl::collide(co.second.get(),crobot_ptrs[i],collision_req,collision_res);
          size_t index = static_cast<size_t>(-1);
          double max_pen_depth = std::numeric_limits<double>::min();
          for(auto i=0u;i<collision_res.numContacts();i++)
          {
            const auto& contact = collision_res.getContact(i);
            if(max_pen_depth<contact.penetration_depth)
            {
              max_pen_depth=contact.penetration_depth;
              index=i;
            }
          }
          dist_result.min_distance=-max_pen_depth;
          const auto& contact = collision_res.getContact(index);
          point1=contact.pos;
        }
      }
//      if(i==0)
//      {
//        std::cout << "min dist"<<std::endl;
//        std::cout << dist_result.min_distance << std::endl;
//        std::cout << "point1:"<<std::endl;
//        std::cout << point1 << std::endl;
//        std::cout << "point2:"<<std::endl;
//        std::cout << point2 << std::endl;
//        std::cout << "----" << std::endl;
//        line_strip.ns = co.first+"To"+link_model_ptrs[4-i*2]->getName();
//        line_strip.points.clear();
//        p.x = point1[0];
//        p.y = point1[1];
//        p.z = point1[2];
//        line_strip.points.push_back(p);
//        p.x = point2[0];
//        p.y = point2[1];
//        p.z = point2[2];
//        line_strip.points.push_back(p);
//        marker_publisher.publish(line_strip);
//      }

      points1_co[i] = {point1[0],point1[1],point1[2]};
      points2_co[i] = {point2[0],point2[1],point2[2]};
      dists_co[i] = dist_result.min_distance;
    }

    int index_in_co = std::distance(co_ptrs.begin(),std::find(co_ptrs.begin(),co_ptrs.end(),co));
    dists[index_in_co] = dists_co;
    points1[index_in_co] = points1_co;
    points2[index_in_co] = points2_co;
  }
}

double MoveitTool::selfCollisionDistAndPoints(Eigen::Vector3d& point1, Eigen::Vector3d& point2)
{
  fcl::DistanceResultd dist_result;
  fcl::distance(crobot_ptrs[0], crobot_ptrs[2],dist_request,dist_result);
  const auto& fcl_point1 = dist_result.nearest_points[0];
  const auto& fcl_point2 = dist_result.nearest_points[1];

  point1 = {fcl_point1[0],fcl_point1[1],fcl_point1[2]};
  point2 = {fcl_point2[0],fcl_point2[1],fcl_point2[2]};
  return dist_result.min_distance;
}




MoveitTool::~MoveitTool()
{
  for(auto crobot:crobot_ptrs)
    delete crobot;
}
