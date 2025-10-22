#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORNET__HPP
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORNET__HPP
#if USE_MLP
#include <torch/torch.h>
namespace rm_armor_finder::mlp {

    // 定义模型
    class ArmorNetImpl : public torch::nn::Module {
    public:
        ArmorNetImpl(int num_classes, int size) {
            fc1 = register_module("fc1", torch::nn::Linear(size, 1024));
            fc2 = register_module("fc2", torch::nn::Linear(1024, 512));
            fc3 = register_module("fc3", torch::nn::Linear(512, num_classes));
        }

        torch::Tensor forward(torch::Tensor x) {
            x = x.view({x.size(0), -1});
            x = torch::relu(fc1->forward(x));
            x = torch::relu(fc2->forward(x));
            x = torch::softmax(fc3->forward(x), 1);
            return x;
        }

    private:
        torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
    };
    TORCH_MODULE(ArmorNet);

}// namespace rm_armor_finder::mlp
#endif// USE_MLP
#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORNET__HPP
