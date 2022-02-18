#include <algorithm>
#include <assert.h>

#include "bkioctree_node.h"

namespace see_csom {

    /// Default static values 默认静态参数
    int Semantics::num_class = 2;
    float Semantics::sf2 = 1.0f;
    float Semantics::ell = 1.0f;
    float Semantics::prior = 0.5f;
    float Semantics::var_thresh = 1000.0f;
    float Semantics::free_thresh = 0.3f;
    float Semantics::occupied_thresh = 0.7f;

    void Semantics::get_probs(std::vector<float>& probs) const {
      assert (probs.size() == num_class);  //如果里面这俩不相等，就终止
      float sum = 0;
      for (auto m : ms)
        sum += m;  
      for (int i = 0; i < num_class; ++i)
        probs[i] = ms[i] / sum;  //得到每个状态的概率
    }

    // 获得变分
    void Semantics::get_vars(std::vector<float>& vars) const {
      assert (vars.size() == num_class);
      float sum = 0;
      for (auto m : ms)
        sum += m;
      for (int i = 0; i < num_class; ++i)
        vars[i] = ((ms[i] / sum) - (ms[i] / sum) * (ms[i] / sum)) / (sum + 1);  //变分的计算公式，可以得到每个类的变分
    }

    void Semantics::get_ms(std::vector<float>& scs) const{
      assert (scs.size() == num_class);
      scs = ms;
    }

    // 根据新进来的没类的概率进行更新
    void Semantics::update(std::vector<float>& ybars) {
      assert(ybars.size() == num_class);
      classified = true;
      for (int i = 0; i < num_class; ++i)
        ms[i] += ybars[i];

      std::vector<float> probs(num_class);
      get_probs(probs);

      semantics = std::distance(probs.begin(), std::max_element(probs.begin(), probs.end()));  //找到里面最大的那个
      if (semantics > num_class-1)
        std::cout << "why?" << std::endl;
      if (semantics == 0)   // 如果最大那个不是0，不管是什么，都是被占用的
        state = State::FREE;
      else
        state = State::OCCUPIED;
    }
}
