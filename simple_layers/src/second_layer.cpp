#include<simple_layers/second_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SecondLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace simple_layer_namespace
{

SecondLayer::SecondLayer() {}

void SecondLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SecondLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void SecondLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void SecondLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SecondLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  unsigned int mx;
  unsigned int my;
  /*if(worldToMap(mark_x, mark_y, mx, my)){
    setCost(mx, my, LETHAL_OBSTACLE);
  }*/

  for(int j = -3;j<4;j++)
  {
	for(int i= -10;i<10;i++)
	{
	 	if(worldToMap(robot_x+i, robot_y+j, mx, my)){
    		setCost(mx, my, (unsigned char)(100));
  	}


	}
  }

  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}

void SecondLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);


}

} // end namespace
