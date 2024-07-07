#include <rclcpp/rclcpp.hpp>
// #include <torch/torch.h>
// #include <torch/script.h>
// #include <iostream>
// #include <memory>

int main(int argc, char ** argv)
{
    // call this before anything else ROS-related
    rclcpp::init(argc, argv);

    // add spice here
    auto node = std::make_shared<rclcpp::Node>("arrow_classification");
    // RCLCPP_INFO(node->get_logger(), "Epoch: %lu | Batch: %lu | Loss: %f", epoch, batch_index, loss.item<float>());

    // if (argc != 2) {
    //   std::cerr << "usage: example-app <path-to-exported-script-module>\n";
    //   return -1;
    // }

    // torch::jit::script::Module module;
    // try {
    //   // Deserialize the ScriptModule from a file using torch::jit::load().
    //   module = torch::jit::load(argv[1]);
    // }
    // catch (const c10::Error& e) {
    //   std::cerr << "error loading the model\n";
    //   return -1;
    // }

    // std::cout << "ok\n";

    // // Create a vector of inputs.
    // std::vector<torch::jit::IValue> inputs;
    // inputs.push_back(torch::ones({1, 3, 48, 64}));

    // // Execute the model and turn its output into a tensor.
    // at::Tensor output = module.forward(inputs).toTensor();
    // std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}