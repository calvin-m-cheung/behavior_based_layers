#include<simple_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

GridLayer::GridLayer() {}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  //double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  
  double mark_x = robot_x, mark_y = robot_y;
  

  unsigned int mx;
  unsigned int my;
  /*if(worldToMap(mark_x, mark_y, mx, my)){
    //setCost(mx, my, LETHAL_OBSTACLE);
    setCost(mx, my, 200);
  }*/

//j,y is left right of vehicle
  for(int j = -1;j<2;j++)
  {
        for(int i= 1;i<11;i++)
        {
                if(worldToMap(robot_x+i, robot_y+j, mx, my)){
                setCost(mx, my, (unsigned char)(250));
        }

        
        }
  }

  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;


  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

} // end namespace
