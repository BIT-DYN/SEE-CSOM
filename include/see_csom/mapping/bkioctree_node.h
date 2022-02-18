#pragma once

#include <vector>
#include <iostream>

namespace see_csom {

    /// Occupancy state: before pruning: FREE, OCCUPIED, UNKNOWN; after pruning: PRUNED 
    //每个节点的状态：剪枝前：空闲、占用、未知；剪枝后：修剪
    enum class State : char {
        FREE, OCCUPIED, UNKNOWN, PRUNED
    };

    /*
     * @brief Inference ouputs and occupancy state.
     *
     * Occupancy has member variables: m_A and m_B (kernel densities of positive
     * and negative class, respectively) and State.
     * Before using this class, set the static member variables first.
     */

    // 输出和占用状态。
    // 占有率有成员变量：m_A和m_B（分别为正类和负类的核密度）和状态。
    // 在使用这个类之前，首先设置静态成员变量。
    class Semantics {
      
      friend class SemanticBKIOctoMap;  //友类，可以直接访问成员

    public:
        /*
         * @brief Constructors and destructor. 构造函数和析构函数。
         */
        Semantics() : ms(std::vector<float>(num_class, prior)), state(State::UNKNOWN), semantics(0) { classified = false; }  //构造一个结点，num_class个概率prior的类

        Semantics(const Semantics &other) : ms(other.ms), state(other.state), semantics(other.semantics) { }  //获得一个节点信息，给到自己

        //重载了=运算符，就是直接复制
        Semantics &operator=(const Semantics &other) {
          ms = other.ms;
          state = other.state;
          semantics = other.semantics;
          return *this;
        }

        ~Semantics() { }

        /*
         * @brief Exact updates for nonparametric Bayesian kernel inference
         * @param ybar kernel density estimate of positive class (occupied)
         * @param kbar kernel density of negative class (unoccupied)
         */
        
        // 非参数贝叶斯核推理的精确更新
        // ybar是 正类（占位）的核密度估计
        // kbar是负类（未占用） 核密度
        void update(std::vector<float>& ybars);

        /// Get probability of occupancy.  得到占用概率
        void get_probs(std::vector<float>& probs) const;

        /// Get variance of occupancy (uncertainty)  //得到占用的变分（不确定行）
	      void get_vars(std::vector<float>& vars) const;

        /// 获取ms 每个语义的概率（数量乘以稀疏核）的一个容器
	      void get_ms(std::vector<float>& ms) const;
        
        /*
         * @brief Get occupancy state of the node.
         * @return occupancy state (see State).
         */
        inline State get_state() const { return state; }  //获得节点状态：占用，空闲，未知，剪枝

        inline int get_semantics() const { return semantics; }   //获得节点的语义状态，就是属于哪一类

        bool classified;  //被分类了吗

    private:
        std::vector<float> ms;   //每个语义的概率（数量乘以稀疏核）的一个容器
        State state;  //状态：占用，空闲，未知，剪枝
        int semantics;  //语义
        static int num_class;      // number of classes
        
        static float sf2;
        static float ell;   // length-scale
        static float prior;  // prior on each class

        static float var_thresh;
        static float free_thresh;     // FREE occupancy threshold
        static float occupied_thresh; // OCCUPIED occupancy threshold
    };

    typedef Semantics SemanticOcTreeNode;
}
