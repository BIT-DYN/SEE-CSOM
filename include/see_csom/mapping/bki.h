#pragma once

#include "bkioctomap.h"

namespace see_csom {

	/*
     * @brief Bayesian Generalized Kernel Inference on Bernoulli distribution
     * @param dim dimension of data (2, 3, etc.)
     * @param T data type (float, double, etc.)
     * @ref Nonparametric Bayesian inference on multivariate exponential families
     */

  /*
  *Bernoulli分布的贝叶斯广义核推理
  *@param dim 数据维度（2、3等）
  *@param T 数据类型（float、double等）
  *多元指数族的@ref非参数贝叶斯推断
  */ 

    template<int dim, typename T>
    class SemanticBKInference {
    public:
        /// Eigen matrix type for training and test data and kernel 训练测试数据的特征矩阵类型及核函数，这里的-1应该是block中gp点的数量
        using MatrixXType = Eigen::Matrix<T, -1, dim, Eigen::RowMajor>;
        using MatrixKType = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>;
        using MatrixDKType = Eigen::Matrix<T, -1, 1>;
        using MatrixYType = Eigen::Matrix<T, -1, 1>;

        // 初始化，需要num_class、 sf2 、 ell
        SemanticBKInference(int nc, T sf2, T ell) : nc(nc), sf2(sf2), ell(ell), trained(false) { }

    // 根据一个MapPoint更新block
    void fusion_block(Block *block, const pcl::PointCloud<MapPoint>::const_iterator it)
    {
        std::vector<float> ybars(nc, 0);
        ybars[it->semantic_label1] = it->prob1;
        ybars[it->semantic_label2] = it->prob2;
        ybars[it->semantic_label3] = it->prob3;
        MatrixYType _y_vec = Eigen::Map<const MatrixYType>(ybars.data(), ybars.size(), 1); // 用来计数的，一列，行数为gp点的数量

        std::vector<float> xs;  //这里面是test_blocks当前这个block的所有的的节点（子块）的xyz
        for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
            point3f p = block->get_loc(leaf_it);
            xs.push_back(p.x());
            xs.push_back(p.y());
            xs.push_back(p.z());
        }
        MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);  //成为了MatrixXType格式， 八行三列

        std::vector<float> block_xyz; // 这个点的xyz坐标
        block_xyz.push_back(it->x);
        block_xyz.push_back(it->y);
        block_xyz.push_back(it->z);
        MatrixXType _x = Eigen::Map<const MatrixXType>(block_xyz.data(), block_xyz.size() / dim, dim);  //一行，三列

        MatrixKType Ks;  
        covSparse(_xs, _x, Ks);  //Ks在这里为八行一列，是每个叶节点到这个语义点的距离，所以数目等于叶节点的数目也就是2的n次方

        MatrixYType _ybar;
        int i=0;
        for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
            SemanticOcTreeNode &node = leaf_it.get_node();
            _ybar = (Ks(i, 0) * _y_vec);  // 稀疏核距离*计数的值，可以得到这个块的语义大小
            for (int r = 0; r < _ybar.rows(); ++r)
                ybars[r] = _ybar(r, 0);
            node.update(ybars);  // 把这个块的每个节点类别信息进行更新
            i++;
        }
    }



        /*
         * @brief Fit BGK Model  适合的BGK模型
         * @param x input vector (3N, row major)  输入向量（3N，行主）
         * @param y target vector (N)   目标向量
         */
        void train(const std::vector<T> &x, const std::vector<T> &y) {
            assert(x.size() % dim == 0 && (int) (x.size() / dim) == y.size());
            MatrixXType _x = Eigen::Map<const MatrixXType>(x.data(), x.size() / dim, dim);
            MatrixYType _y = Eigen::Map<const MatrixYType>(y.data(), y.size(), 1);
            this->y_vec = y;  // 单独又存储了一个
            train(_x, _y);
        }

        /*
         * @brief Fit BGK Model  合适的BGK模型
         * @param x input matrix (NX3)
         * @param y target matrix (NX1)
         */
        void train(const MatrixXType &x, const MatrixYType &y) {
            this->x = MatrixXType(x);    //存储了block中gp点的xyz
            this->y = MatrixYType(y);   //存储了block中gp点的lable
            trained = true;
        }

       
      void predict(const std::vector<T> &xs, std::vector<std::vector<T>> &ybars) {
          assert(xs.size() % dim == 0);
          MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);  //成为了MatrixXType格式
          assert(trained == true);
          MatrixKType Ks;  

          covSparse(_xs, x, Ks);  //获得每个块坐标和gp坐标的x稀疏核函数
          
          ybars.resize(_xs.rows());

          for (int r = 0; r < _xs.rows(); ++r)  
            ybars[r].resize(nc);

          MatrixYType _y_vec = Eigen::Map<const MatrixYType>(y_vec.data(), y_vec.size(), 1); // 用来计数的，一列，行数为gp点的数量
          for (int k = 0; k < nc; ++k) {
            for (int i = 0; i < y_vec.size(); ++i) {
              // 对label标签进行计数
              if (y_vec[i] == k)
                _y_vec(i, 0) = 1;
              else
                _y_vec(i, 0) = 0;
            }
            MatrixYType _ybar;
            _ybar = (Ks * _y_vec);  // 稀疏核距离*计数的值，可以得到这个块的语义大小
            
            for (int r = 0; r < _ybar.rows(); ++r)
              ybars[r][k] = _ybar(r, 0);
          }
      }

      void predict_csm(const std::vector<T> &xs, std::vector<std::vector<T>> &ybars) {
          assert(xs.size() % dim == 0);
          MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);
          assert(trained == true);
          MatrixKType Ks;

          covCountingSensorModel(_xs, x, Ks);
          
          ybars.resize(_xs.rows());
          for (int r = 0; r < _xs.rows(); ++r)
            ybars[r].resize(nc);

            MatrixYType _y_vec = Eigen::Map<const MatrixYType>(y_vec.data(), y_vec.size(), 1);
            for (int k = 0; k < nc; ++k) {
              for (int i = 0; i < y_vec.size(); ++i) {
                if (y_vec[i] == k)
                  _y_vec(i, 0) = 1;
                else
                  _y_vec(i, 0) = 0;
              }
            
            MatrixYType _ybar;
            _ybar = (Ks * _y_vec);
            
            for (int r = 0; r < _ybar.rows(); ++r)
              ybars[r][k] = _ybar(r, 0);
          }
      }

        
    private:
        /*
         * @brief Compute Euclid distances between two vectors.  计算两个向量之间的欧几里德距离。
         * @param x input vector
         * @param z input vecotr
         * @return d distance matrix
         */
        void dist(const MatrixXType &x, const MatrixXType &z, MatrixKType &d) const {
            d = MatrixKType::Zero(x.rows(), z.rows());
            for (int i = 0; i < x.rows(); ++i) {
                d.row(i) = (z.rowwise() - x.row(i)).rowwise().norm();
            }
        }

        /*
         * @brief Matern3 kernel.  Matern3内核。
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         */
        void covMaterniso3(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(1.73205 / ell * x, 1.73205 / ell * z, Kxz);
            Kxz = ((1 + Kxz.array()) * exp(-Kxz.array())).matrix() * sf2;
        }

        /*
         * @brief Sparse kernel.  稀疏内核。
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix  协方差矩阵
         * @ref A sparse covariance function for exact gaussian process inference in large datasets.  参考一个稀疏协方差函数，用于大数据集中精确高斯过程的推理。
         */
        void covSparse(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(x / ell, z / ell, Kxz);   //d/l

          // 这就是论文里的公式10，稀疏核k=sf2 * [ (1/3) * (2 + cos(2 * pi * d/l) * (1 - d/l) + 1/(2*pi) * sin (2 * pi * d/l)]
            // Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
            //       (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * sf2;

            // 采用线性函数
            Kxz = 1.0f - Kxz.array();

            // Clean up for values with distance outside length scale  清除距离超出长度刻度的值
            // Possible because Kxz <= 0 when dist >= ell
            for (int i = 0; i < Kxz.rows(); ++i)
            {
                for (int j = 0; j < Kxz.cols(); ++j)
                    if (Kxz(i,j) < 0.0)
                        Kxz(i,j) = 0.0f;
            }
        }

        void covCountingSensorModel(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
          Kxz = MatrixKType::Ones(x.rows(), z.rows());
        }


        T sf2;    // signal variance 信号方差
        T ell;    // length-scale
        int nc;   // number of classes

        MatrixXType x;   // temporary storage of training data  培训数据的临时存储
        MatrixYType y;   // temporary storage of training labels  培训标签的临时存储
        std::vector<T> y_vec;

        bool trained;    // true if bgkinference stored training data
    };

    typedef SemanticBKInference<3, float> SemanticBKI3f;  // 一个3维语义BKI推理

}
