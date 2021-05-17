for(auto& res:distance_data.result)
{

//       1.In fcl 0.5, nearest points positions are in the coordinate frame of respective objects joinning in
//       in distance calculation, thus, positions need to be applied with rotation and translation to transform
//       it to world frame.
//       2. In fcl 0.5 there is fcl::Vec3d/Vec3f instead of Vector3d/Vector3f

//      fcl::Vec3f point1 = res.first->getRotation()*res.second.nearest_points[0] + res.first->getTranslation();
//      fcl::Vec3f point2 = co.second->getRotation()*res.second.nearest_points[1] + co.second->getTranslation();


 fcl::Vector3d point1 = res.second.nearest_points[0];
 fcl::Vector3d point2 = res.second.nearest_points[1];
 int index_in_crobot = std::distance(crobot_ptrs.begin(),std::find(crobot_ptrs.begin(),crobot_ptrs.end(),res.first));


  publish the distance line for link7
 if(i == 0)
 {
   line_strip.ns = co.first+"To"+joint_names[6];
   line_strip.points.clear();
   p.x = point1[0];
   p.y = point1[1];
   p.z = point1[2];
   line_strip.points.push_back(p);
   p.x = point2[0];
   p.y = point2[1];
   p.z = point2[2];
   line_strip.points.push_back(p);
   marker_publisher.publish(line_strip);
 }
  store the data in the given reference
 points1_co[index_in_crobot] = {point1[0],point1[1],point1[2]};
 points2_co[index_in_crobot] = {point2[0],point2[1],point2[2]};
 dists_co[index_in_crobot] = res.second.min_distance;
}
