/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
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

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <ros/ros.h>

#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>



using namespace std;
using namespace octomap;

int main(int argc, char** argv) {

  cout << endl;
  cout << "generating example map" << endl;

  shared_ptr<OcTree> tree1= make_shared<OcTree>(0.1);  // create empty tree with resolution 0.1


  // insert some measurements of occupied cells

  for (int x=-20; x<20; x++) {
    for (int y=-20; y<20; y++) {
      for (int z=-20;z<20; z++) {
        point3d endpoint ((float) x*0.025f, (float) y*0.025f, (float) z*0.025f);
        tree1->updateNode(endpoint, true); // integrate 'occupied' measurement
      }
    }
  }
  shared_ptr<fcl::OcTreed> treed1 = make_shared<fcl::OcTreed>(tree1);
//  shared_ptr<fcl::CollisionGeometryd> tree_ptr(treed);
  Eigen::Translation3d translation1(0,0,0);
  Eigen::Quaterniond quater;
  quater.w()=1.0;
  quater.x()=0.0;
  quater.y()=0.0;
  quater.z()=0.0;
  Eigen::Affine3d tf = translation1*quater.toRotationMatrix();
  Eigen::Isometry3d tf1;
  tf1.linear()= tf.linear();
  tf1.translation() = tf.translation();

  shared_ptr<fcl::Sphered> sphere_ptr=make_shared<fcl::Sphered>(0.5);
  Eigen::Translation3d translation2(0.7,0.7,0);
  tf = translation2*quater.toRotationMatrix();
  Eigen::Isometry3d tf2;
  tf2.linear()= tf.linear();
  tf2.translation() = tf.translation();

  fcl::detail::GJKSolver_libccdd gjk_solver;
  fcl::CollisionRequestd req;
  req.enable_contact=true;
  req.num_max_contacts=100000;
  fcl::CollisionResultd res;

  fcl::DistanceRequestd dis_req;
  dis_req.enable_nearest_points=true;
  dis_req.enable_signed_distance=true;
  fcl::DistanceResultd dis_res;

  fcl::detail::OcTreeSolver<fcl::detail::GJKSolver_libccdd> oct_solver(&gjk_solver);
  oct_solver.OcTreeShapeIntersect(treed1.get(),*sphere_ptr,tf1,tf2,req,res);
  size_t index = static_cast<size_t>(-1);
  double max_pen_depth = numeric_limits<double>::min();
  for(auto i=0u;i<res.numContacts();i++)
  {
    const auto& contact = res.getContact(i);
    if(max_pen_depth<contact.penetration_depth)
    {
      max_pen_depth=contact.penetration_depth;
      index=i;
    }
  }
  cout << "min distance: " << -max_pen_depth << endl;
  cout << "contact normal: " << endl;
  cout << res.getContact(index).normal << endl;
}
