#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <cmath>
#include <string>

namespace see_csom {

    double interpolate( double val, double y0, double x0, double y1, double x1 ) {
        return (val-x0)*(y1-y0)/(x1-x0) + y0;
    }

    double base( double val ) {
        if ( val <= -0.75 ) return 0;
        else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
        else if ( val <= 0.25 ) return 1.0;
        else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
        else return 0.0;
    }

    double red( double gray ) {
        return base( gray - 0.5 );
    }
    double green( double gray ) {
        return base( gray );
    }
    double blue( double gray ) {
        return base( gray + 0.5 );
    }
    
    std_msgs::ColorRGBA JetMapColor(float gray) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      color.r = red(gray);
      color.g = green(gray);
      color.b = blue(gray);
      return color;
    }

    std_msgs::ColorRGBA SemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 1:
          color.r = 1;
          color.g = 0;
          color.b = 0;
          break;
        case 2:
          color.r = 70.0/255;
          color.g = 130.0/255;
          color.b = 180.0/255;
          break;
        case 3:
          color.r = 218.0/255;
          color.g = 112.0/255;
          color.b = 214.0/255;
          break;
        default:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          break;
      }

      return color;
    }

    // 这里修改颜色
    std_msgs::ColorRGBA SemanticKITTISemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 1: // "unlabeled", and others ignored
          color.r = 0.0 / 255;
          color.g = 0.0 / 255;
          color.b = 0.0 / 255;
          break;
        case 2:  // car
          color.r = 245.0 / 255;
          color.g = 150.0 / 255;
          color.b = 150.0 / 255;
          break;
        case 3:  // bicycle
          color.r = 245.0 / 255;
          color.g = 230.0 / 255;
          color.b = 100.0 / 255;
          break;
        case 4:  // motorcycle
          color.r = 150.0 / 255;
          color.g = 60.0 / 255;
          color.b = 30.0 / 255;
          break;
        case 5:  // truck
          color.r = 180.0 / 255;
          color.g = 30.0 / 255;
          color.b = 80.0 / 255;
          break;
        case 6:  // other-vehicle
          color.r = 255.0 / 255;
          color.g = 0.0 / 255;
          color.b = 0.0 / 255;
          break;
        case 7:  // person
          color.r = 200.0 / 255;
          color.g = 40.0 / 255;
          color.b = 1;
          break;
        case 8:  // bicyclist
          color.r = 200.0 / 255;
          color.g = 40.0 / 255;
          color.b = 1;
          break;
        case 9:  // motorcyclist
          color.r = 90.0 / 255;
          color.g = 30.0 / 255;
          color.b = 150.0 / 255;
          break;
        case 10:  // road
          color.r = 1;
          color.g = 0;
          color.b = 1;
          break;
        case 11: // parking
          color.r = 1;
          color.g = 150.0 / 255;
          color.b = 1;
          break;
        case 12: // sidewalk
          color.r = 75.0 / 255;
          color.g = 0;
          color.b = 75.0 / 255;
          break;
        case 13: // other-ground
          color.r = 75.0 / 255;
          color.g = 0;
          color.b = 175.0 / 255;
          break;
        case 14: // building
          color.r = 0;
          color.g = 200.0 / 255;
          color.b = 1;
          break;
        case 15: // fence
          color.r = 0.0 / 255;
          color.g = 175.0 / 255;
          color.b = 0;
          break;
        case 16: // vegetation
          color.r = 0;
          color.g = 175.0 / 255;
          color.b = 0;
          break;
        case 17: // trunk
          color.r = 0;
          color.g = 60.0 / 255;
          color.b = 135.0 / 255;
          break;
        case 18: // terrain
          color.r = 80.0 / 255;
          color.g = 240.0 / 255;
          color.b = 150.0 / 255;
          break;
        case 19: // pole
          color.r = 150.0 / 255;
          color.g = 240.0 / 255;
          color.b = 1;
          break;
        case 20: // traffic-sign
          color.r = 0;
          color.g = 0;
          color.b = 1;
          break;
        default:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          break;
      }
      return color;
    }

    // 这里修改颜色
    std_msgs::ColorRGBA SemanticKITTIBagSemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

       switch (c) {
        case 0:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          color.a = 0.01;
          break;
        case 1:  // unlabeled
          color.r = 0;
          color.g = 0;
          color.b = 0;
          break;
        case 2:  // outlier
          color.r = 0;
          color.g = 0;
          color.b = 1;
          break;
        case 3:  // car
          color.r = 245.0 / 255;
          color.g = 150.0 / 255;
          color.b = 100.0 / 255;
          break;
        case 4:  // bicycle
          color.r = 245.0 / 255;
          color.g = 230.0 / 255;
          color.b = 100.0 / 255;
          break;
        case 5:  // bus
          color.r = 250.0 / 255;
          color.g = 80.0 / 255;
          color.b = 100.0 / 255;
          break;
        case 6:  // motorcycle
          color.r = 150.0 / 255;
          color.g = 60.0 / 255;
          color.b = 30.0 / 255;
          break;
        case 7:  // on-rails
          color.r = 1;
          color.g = 0;
          color.b = 0;
          break;
        case 8:  // truck
          color.r = 180.0 / 255;
          color.g = 30.0 / 255;
          color.b = 80.0 / 255;
          break;
        case 9:  // other-vehicle
          color.r = 1;
          color.g = 0;
          color.b = 0;
          break;
        case 10: // person
          color.r = 30.0 / 255;
          color.g = 30.0 / 255;
          color.b = 1;
          break;
        case 11: // bicyclist
          color.r = 200.0 / 255;
          color.g = 40.0 / 255;
          color.b = 255.0 / 255;
          break;
        case 12: // motorcyclist
          color.r = 90.0 / 255;
          color.g = 30.0 / 255;
          color.b = 150.0 / 255;
          break;
        case 13: // road
          color.r = 1;
          color.g = 0;
          color.b = 1;
          break;
        case 14: // parking
          color.r = 1;
          color.g = 150.0 / 255;
          color.b = 1;
          break;
        case 15: // sidewalk
          color.r = 75.0 / 255;
          color.g = 0;
          color.b = 75.0 / 255;
          break;
        case 16: // other-ground
          color.r = 75.0 / 255;
          color.g = 0.0 / 255;
          color.b = 175.0 / 255;
          break;
        case 17: //building2
          color.r = 0;
          color.g = 200.0 / 255;
          color.b = 1;
          break;
        case 18: // fence
          color.r = 50.0 / 255;
          color.g = 120.0 / 255;
          color.b = 255;
          break;
        case 19: // other-structure
          color.r = 0;
          color.g = 150.0 / 255;
          color.b = 1;
          break;
         case 20: // lane-marking
          color.r = 170.0 / 255;
          color.g = 1;
          color.b = 150.0 / 255;
          break;
         case 21: // vegetation
          color.r = 0;
          color.g = 175.0 / 255;
          color.b = 0;
          break;
         case 22: // trunk
          color.r = 0;
          color.g = 60.0 / 255;
          color.b = 135.0 / 255;
          break;
         case 23: // terrain
          color.r = 80.0 / 255;
          color.g = 240.0 / 255;
          color.b = 150.0 / 255;
          break;
         case 24: // pole
          color.r = 150.0 / 255;
          color.g = 240.0 / 255;
          color.b = 1;
          break;
         case 25: // traffic-sign
          color.r = 0;
          color.g = 0;
          color.b = 1;
          break;
         case 26: // other-object
          color.r = 1;
          color.g = 1;
          color.b = 50.0 / 255;
          break;
         case 27: // moving-car
          color.r = 245.0 / 255;
          color.g = 150.0 / 255;
          color.b = 100.0 / 255;
          break;
         case 28: // moving-bicyclist
          color.r = 1;
          color.g = 0;
          color.b = 0;
          break;
         case 29: // moving-person
          color.r = 200.0 / 255;
          color.g = 40.0 / 255;
          color.b = 1;
          break;
         case 30: // moving-motorcyclist
          color.r = 30.0 / 255;
          color.g = 30.0 / 255;
          color.b = 1;
          break;
         case 31: // moving-on-rails
          color.r = 90.0 / 255;
          color.g = 30.0 / 255;
          color.b = 150.0 / 255;
          break;
         case 32: // moving-bus
          color.r = 250.0 / 255;
          color.g = 80.0 / 255;
          color.b = 100.0 / 255;
          break;
         case 33: // moving-truck
          color.r = 180.0 / 255;
          color.g = 30.0 / 255;
          color.b = 80.0 / 255;
          break;
         case 34: // moving-other-vehicle
          color.r = 1;
          color.g = 0;
          color.b = 0;
          break;
        default:
          color.r = 0;
          color.g = 0;
          color.b = 0;
          break;
      }
      return color;
    }


 // 这里修改颜色，确实应该.0/255
    std_msgs::ColorRGBA SeniorBagSemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 0:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          color.a = 0.01;
          break;
        case 1:  // 建筑
          color.r = 70.0/ 255;  //70
          color.g = 70.0 / 255;  //70
          color.b = 70.0 / 255;  //70
          break;
        case 2:  // 人行道
          color.r = 244.0/ 255;  //244
          color.g = 35.0 / 255;  //35
          color.b = 232.0 / 255;  //232
          break;
        case 3:  // 道路
          color.r = 128.0/255;  //128
          color.g = 64.0/255 ;  //64
          color.b = 128.0/255 ;  //128
          break;
        case 4:  //  围栏
          color.r = 190.0 / 255;  //190.0
          color.g = 153.0 / 255;  //153.0
          color.b = 153.0 / 255;  // 153.0
          break;
        case 5:  // 围墙
          color.r = 153.0 / 255;  //153.0
          color.g = 153.0 / 255;  //153.0
          color.b = 153.0 / 255;  //153.0
          break;
        case 6:  // 柱子
          color.r = 220.0 / 255;
          color.g = 20.0 / 255;
          color.b = 60.0 / 255;
          break;
        case 7:  // 墙
          color.r = 102.0 / 255;
          color.g = 102.0 / 255;
          color.b = 156.0 / 255;
          break;
        case 8:  // 植物
          color.r = 107.0 / 255;
          color.g = 142.0 / 255;
          color.b = 35.0 / 255;
          break;
        case 9:  // 交通标志
          color.r = 220.0 / 255;
          color.g = 220.0 / 255;
          color.b = 0.0 / 255;
          break;
        case 10: // 火车
          color.r = 0.0 / 255;  //0 
          color.g = 80.0 / 255;  //80
          color.b = 100.0 / 255;  //100
          break;
        case 11: // 绿化带
          color.r = 152.0 / 255;
          color.g = 251.0 / 255;
          color.b = 152.0 / 255;
          break;
        case 12: // 自行车
          color.r = 119.0 / 255;
          color.g = 11.0 / 255;  //11
          color.b = 32.0 / 255;  //32
          break;
        case 13: // 汽车
          color.r = 0.0;
          color.g = 0.0;
          color.b = 142.0 / 255;
          break;
        case 14: // 摩托车
          color.r = 0.0;
          color.g = 0.0;
          color.b = 230.0 / 255;
          break;
        case 15: // 骑手
          color.r = 1.0;
          color.g = 0.0;
          color.b = 0.0;
          break;
        case 16: // 交通灯
          color.r = 250.0 / 255;
          color.g = 170.0 / 255;
          color.b = 30.0 / 255;
          break;
        case 17: // 公交车
          color.r = 0.0;
          color.g = 60.0 / 255;
          color.b = 100.0 / 255;
          break;
        case 18: // 卡车
          color.r = 0.0 / 255;
          color.g = 0.0 / 255;
          color.b = 70.0 / 255;
          break;
        case 19: // 天空
          color.r = 70.0 / 255;
          color.g = 130.0 / 255;
          color.b = 180.0 / 255;
          break;
        default:
          color.r = 0.0;
          color.g = 0.0;
          color.b = 0.0;
          break;
      }
      return color;
    }

    // 这里修改颜色，确实应该.0/255
    std_msgs::ColorRGBA GazeboSemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 0:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          color.a = 0.01;
          break;
        case 1:  // 道路
          color.r = 128.0/ 255;
          color.g = 64.0 / 255;  
          color.b = 128.0 / 255;  
          break;
        case 2:  // 人行道
          color.r = 244.0/ 255;  
          color.g = 35.0 / 255;  
          color.b = 232.0 / 255;  
          break;
        case 3:  // 建筑
          color.r = 70.0/255;  
          color.g = 70.0/255 ; 
          color.b = 70.0/255 ; 
          break;
        case 4:  //  植物
          color.r = 107.0 / 255;  
          color.g = 142.0 / 255; 
          color.b = 35.0 / 255; 
          break;
        case 5:  // 天空
          color.r = 70.0 / 255;     //70
          color.g = 70.0 / 255;    //130
          color.b = 70.0 / 255;     //180
          break;
        case 6:  // 人
          color.r = 220.0 / 255;
          color.g = 20.0 / 255;
          color.b = 60.0 / 255;
          break;
        case 7:  // 车辆
          color.r = 0.0 / 255;
          color.g = 0.0 / 255;
          color.b = 142.0 / 255;
          break;
        case 8:  // 那些混乱的颜色都设置为黑色
          color.r = 0.0/ 255;  //70
          color.g = 0.0 / 255;  //70
          color.b = 0.0 / 255;  //70
          break;
        default:
          color.r = 1.0;
          color.g = 1.0;
          color.b = 1.0;
          color.a = 0.0;
          break;
      }
      return color;
    }

     // 这里修改颜色，确实应该.0/255
    std_msgs::ColorRGBA StanfordMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 0:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          color.a = 0.01;
          break;
        case 1:  // 墙面
          color.r = 128.0/ 255;
          color.g = 128.0 / 255;  
          color.b = 128.0 / 255;  
          break;
        case 2:  // 椅子
          color.r = 200.0/ 255;  
          color.g = 100.0 / 255;  
          color.b = 20.0 / 255;  
          break;
        case 3:  // 桌子
          color.r = 200.0/255;  
          color.g = 20.0/255 ; 
          color.b = 100.0/255 ; 
          break;
        case 4:  //  地板
          color.r = 120.0 / 255;  
          color.g = 20.0 / 255; 
          color.b = 120.0 / 255; 
          break;
        case 5:  // 书架
          color.r = 20.0 / 255;     //70
          color.g = 120.0 / 255;    //130
          color.b = 120.0 / 255;     //180
          break;
        case 6:  // 门
          color.r = 150.0 / 255;
          color.g = 50.0 / 255;
          color.b = 50.0 / 255;
          break;
        case 7:  // 光线
          color.r = 120.0 / 255;
          color.g = 120.0 / 255;
          color.b = 20.0 / 255;
          break;
        case 8:  // 木板
          color.r = 100.0/ 255;  //70
          color.g = 50.0 / 255;  //70
          color.b = 250.0 / 255;  //70
          break;
        case 9:  // 杂物
          color.r = 107.0/ 255;  //70
          color.g = 142.0 / 255;  //70
          color.b = 35.0 / 255;  //70
          break;
        case 10:  // 房顶
          color.r = 20.0/ 255;  //70
          color.g = 200.0 / 255;  //70
          color.b = 200.0 / 255;  //70
          break;
        default:
          color.r = 1.0;
          color.g = 1.0;
          color.b = 1.0;
          color.a = 0.0;
          break;
      }
      return color;
    }

    std_msgs::ColorRGBA NCLTSemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;

      switch (c) {
        case 1:  // water
          color.r = 30.0 / 255;
          color.g = 144.0 / 255;
          color.b = 250.0 / 255;
          break;
        case 2:  // road
          color.r = 250.0 / 255;
          color.g = 250.0 / 255;
          color.b = 250.0 / 255;
          break;
        case 3:  // sidewalk
          color.r = 128.0 / 255;
          color.g = 64.0 / 255;
          color.b = 128.0 / 255;
          //color.r = 250.0/255;
          //color.g = 250.0/255;
          //color.b = 250.0/255;
          break;
        case 4:  // terrain
          color.r = 128.0 / 255;
          color.g = 128.0 / 255;
          color.b = 0;
          break;
        case 5:  // building
          color.r = 250.0 / 255;
          color.g = 128.0 / 255;
          color.b = 0;
          break;
        case 6:  // vegetation
          color.r = 107.0 / 255;
          color.g = 142.0/ 255;
          color.b = 35.0 / 255;
          break;
        case 7:  // car
          color.r = 0;
          color.g = 0;
          color.b = 142.0 / 255;
          break;
        case 8:  // person
          color.r = 220.0 / 255;
          color.g = 20.0 / 255;
          color.b = 60.0 / 255;
          //color.r = 250.0 / 255;
          //color.g = 250.0 / 255;
          //color.b = 250.0 / 255;
          break;
        case 9:  // bike
          color.r = 119.0 / 255;
          color.g = 11.0 / 255;
          color.b = 32.0/ 255;
          break;
        case 10:  // pole
          color.r = 192.0 / 255;
          color.g = 192.0 / 255;
          color.b = 192.0 / 255;
          break;
        case 11:  // stair
          color.r = 123.0 / 255;
          color.g = 104.0 / 255;
          color.b = 238.0 / 255;
          break;
        case 12:  // traffic sign
          color.r = 250.0 / 255;
          color.g = 250.0 / 255;
          color.b = 0;
          break;
        case 13:  // sky
          color.r = 135.0 / 255;
          color.g = 206.0 / 255;
          color.b = 235.0 / 255;
          break;
        default:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          break;
      }
      return color;
    }

    std_msgs::ColorRGBA KITTISemanticMapColor(int c) {
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      
      switch (c) {
        case 1:  // building
          color.r = 128.0 / 255;
          color.g = 0;
          color.b = 0;
          break;
        case 2:  // sky
          color.r = 128.0 / 255;
          color.g = 128.0 / 255;
          color.b = 128.0 / 255;
          break;
        case 3:  // road
          color.r = 128.0 / 255;
          color.g = 64.0  / 255;
          color.b = 128.0 / 255;
          break;
        case 4:  // vegetation
          color.r = 128.0 / 255;
          color.g = 128.0 / 255;
          color.b = 0;
          break;
        case 5:  // sidewalk
          color.r = 0;
          color.g = 0;
          color.b = 192.0 / 255;
          break;
        case 6:  // car
          color.r = 64.0 / 255;
          color.g = 0;
          color.b = 128.0 / 255;
          break;
        case 7:  // pedestrain
          color.r = 64.0 / 255;
          color.g = 64.0 / 255;
          color.b = 0;
          break;
        case 8:  // cyclist
          color.r = 0;
          color.g = 128.0 / 255;
          color.b = 192.0 / 255;
          break;
        case 9:  // signate
          color.r = 192.0 / 255;
          color.g = 128.0 / 255;
          color.b = 128.0 / 255;
          break;
        case 10: // fense
          color.r = 64.0  / 255;
          color.g = 64.0  / 255;
          color.b = 128.0 / 255;
          break;
        case 11: // pole
          color.r = 192.0 / 255;
          color.g = 192.0 / 255;
          color.b = 128.0 / 255;
          break;
        default:
          color.r = 1;
          color.g = 1;
          color.b = 1;
          break;
      }

      return color;
    }


    std_msgs::ColorRGBA heightMapColor(double h) {

        std_msgs::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
            f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
            case 6:
            case 0:
                color.r = v;
                color.g = n;
                color.b = m;
                break;
            case 1:
                color.r = n;
                color.g = v;
                color.b = m;
                break;
            case 2:
                color.r = m;
                color.g = v;
                color.b = n;
                break;
            case 3:
                color.r = m;
                color.g = n;
                color.b = v;
                break;
            case 4:
                color.r = n;
                color.g = m;
                color.b = v;
                break;
            case 5:
                color.r = v;
                color.g = m;
                color.b = n;
                break;
            default:
                color.r = 1;
                color.g = 0.5;
                color.b = 0.5;
                break;
        }

        return color;
    }

    // 一个类，用来发布地图消息
    class MarkerArrayPub {
        typedef pcl::PointXYZ PointType;
        typedef pcl::PointCloud<PointType> PointCloud;
    public:
        MarkerArrayPub(ros::NodeHandle nh, std::string topic, float resolution) : nh(nh),
                                                                                  msg(new visualization_msgs::MarkerArray),
                                                                                  topic(topic),
                                                                                  resolution(resolution),
                                                                                  markerarray_frame_id("/map") {
            pub = nh.advertise<visualization_msgs::MarkerArray>(topic, 1, true);

            msg->markers.resize(2);
            for (int i = 0; i < 2; ++i) {
                msg->markers[i].header.frame_id = markerarray_frame_id;
                msg->markers[i].ns = "map";
                msg->markers[i].id = i;
                msg->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                msg->markers[i].scale.x = resolution * pow(2, i);
                msg->markers[i].scale.y = resolution * pow(2, i);
                msg->markers[i].scale.z = resolution * pow(2, i);
                std_msgs::ColorRGBA color;
                color.r = 0.0;
                color.g = 0.0;
                color.b = 1.0;
                color.a = 1.0;
                msg->markers[i].color = color;
            }
        }

        void insert_point3d(float x, float y, float z, float min_z, float max_z, float size) {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            if (size > 0)
                depth = (int) log2(size / 0.1);

            msg->markers[depth].points.push_back(center);
            if (min_z < max_z) {
                double h = (1.0 - std::min(std::max((z - min_z) / (max_z - min_z), 0.0f), 1.0f)) * 0.8;
                msg->markers[depth].colors.push_back(heightMapColor(h));
            }
        }

        // 清空地图，就是把地图所有消息清空
        void clear_map(float size) {
          int depth = 0;
          if (size > 0)
            depth = (int) log2(size / 0.1);

          msg->markers[depth].points.clear();
          msg->markers[depth].colors.clear();
        }

        // 插入语义3d点
        void insert_point3d_semantics(float x, float y, float z, float size, int c, int dataset) {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            if (size > 0)
                depth = (int) log2(size / 0.1);

            msg->markers[depth].points.push_back(center);
            switch (dataset) {
              case 1:
                msg->markers[depth].colors.push_back(KITTISemanticMapColor(c));
                break;
              case 2:
                msg->markers[depth].colors.push_back(SemanticKITTISemanticMapColor(c));
                break;
              case 3:
              msg->markers[depth].colors.push_back(SemanticKITTIBagSemanticMapColor(c));
              break;
              case 4:
              msg->markers[depth].colors.push_back(SeniorBagSemanticMapColor(c));
              break;
              case 5:
              msg->markers[depth].colors.push_back(GazeboSemanticMapColor(c));
              break;
              case 6:
              msg->markers[depth].colors.push_back(StanfordMapColor(c));
              break;
              default:
                msg->markers[depth].colors.push_back(SemanticMapColor(c));
            }
        }

        void insert_point3d_variance(float x, float y, float z, float min_v, float max_v, float size, float var) {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            if (size > 0)
                    depth = (int) log2(size / 0.1);

            float middle = (max_v + min_v) / 2;
            var = (var - middle) / (middle - min_v);
            //std::cout << var << std::endl; 
            msg->markers[depth].points.push_back(center);
            msg->markers[depth].colors.push_back(JetMapColor(var));

        }

        void insert_point3d(float x, float y, float z, float min_z, float max_z) {
            insert_point3d(x, y, z, min_z, max_z, -1.0f);
        }

        void insert_point3d(float x, float y, float z) {
            insert_point3d(x, y, z, 1.0f, 0.0f, -1.0f);
        }

        void insert_color_point3d(float x, float y, float z, double min_v, double max_v, double v) {
            geometry_msgs::Point center;
            center.x = x;
            center.y = y;
            center.z = z;

            int depth = 0;
            msg->markers[depth].points.push_back(center);

            double h = (1.0 - std::min(std::max((v - min_v) / (max_v - min_v), 0.0), 1.0)) * 0.8;
            msg->markers[depth].colors.push_back(heightMapColor(h));
        }

        void clear() {
            for (int i = 0; i < 10; ++i) {
                msg->markers[i].points.clear();
                msg->markers[i].colors.clear();
            }
        }

        void publish() const {
            msg->markers[0].header.stamp = ros::Time::now();
            pub.publish(*msg);
            // ros::spinOnce();
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        visualization_msgs::MarkerArray::Ptr msg;
        std::string markerarray_frame_id;
        std::string topic;
        float resolution;
    };

}
