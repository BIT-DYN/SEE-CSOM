#ifndef MAP_POINT_TYPE_H
#define MAP_POINT_TYPE_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>


struct PointXYZRGBSemanticsBayesian
{
  PCL_ADD_POINT4D;                  // Preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  union  // Semantic colors
  {
      float data_sem[4];
      struct
      {
        float semantic_color1;
        float semantic_color2;
        float semantic_color3;
        float semantic_color4;
      };
  };
  union  // Confidences
  {
    float data_conf[4];
    struct
    {
      float confidence1;
      float confidence2;
      float confidence3;
      float confidence4;
    };
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBSemanticsBayesian,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, rgb, rgb)
                                   (float, semantic_color1, semantic_color1)
                                   (float, semantic_color2, semantic_color2)
                                   (float, semantic_color3, semantic_color3)
                                   (float, semantic_color4, semantic_color4)
                                   (float, confidence1, confidence1)
                                   (float, confidence2, confidence2)
                                   (float, confidence3, confidence3)
                                   (float, confidence4, confidence4)
)


struct MapPoint
{
  PCL_ADD_POINT4D;                  // Preferred way of adding a XYZ+padding
  union  // Semantic labels
  {
      int data_sem[3];
      struct
      {
        int semantic_label1;
        int semantic_label2;
        int semantic_label3;
      };
  };
  union  // Confidences
  {
    float data_conf[4];
    struct
    {
      float prob1;
      float prob2;
      float prob3;
      float true_prob;
    };
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MapPoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, semantic_label1, semantic_label1)
                                   (float, semantic_label2, semantic_label2)
                                   (float, semantic_label3, semantic_label3)
                                   (float, prob1, prob1)
                                   (float, prob2, prob2)
                                   (float, prob3, prob3)
                                   (float, true_prob, true_prob)
)

#endif
