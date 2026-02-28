# ros2_learning_parameters_advanced

ROS 2 参数系统进阶演示包，覆盖以下四个核心知识点：

| 知识点 | 实现位置 |
|---|---|
| `declare_parameter` + 描述符（范围约束） | `param_demo_node.cpp` 构造函数 |
| `add_on_set_parameters_callback` 参数校验 | `ParamDemoNode::on_parameters_set()` |
| YAML 参数文件加载 | `config/params.yaml` + launch 文件 |
| `/parameter_events` 话题订阅 | `param_event_listener_node.cpp` |

## 快速开始

```bash
# 构建
cd /root/Ros2Learning/ros2_ws
colcon build --packages-select ros2_learning_parameters_advanced --symlink-install

# 激活环境
source install/setup.bash

# 启动（两个节点同时运行）
ros2 launch ros2_learning_parameters_advanced params_demo.launch.py

# 覆盖 node_label（从命令行）
ros2 launch ros2_learning_parameters_advanced params_demo.launch.py node_label:=my_robot
```

## 动态参数实验

在另一个终端执行以下命令，观察两个节点的输出：

```bash
# 查看所有参数
ros2 param list /param_demo_node

# 查看参数描述符（含范围约束）
ros2 param describe /param_demo_node scale_factor

# 合法修改（回调通过）
ros2 param set /param_demo_node node_label  hello_ros2
ros2 param set /param_demo_node scale_factor 2.5
ros2 param set /param_demo_node verbose true

# 非法修改（回调拒绝）
ros2 param set /param_demo_node node_label  ""      # 空字符串
ros2 param set /param_demo_node scale_factor 0.0    # 必须 > 0
ros2 param set /param_demo_node print_rate_hz 999   # 超出范围 [1,100]
```

## 关键 API 速查

```cpp
// 声明参数（带描述符）
rcl_interfaces::msg::ParameterDescriptor desc;
desc.description = "说明文字";
rcl_interfaces::msg::IntegerRange range;
range.from_value = 1; range.to_value = 100; range.step = 1;
desc.integer_range.push_back(range);
this->declare_parameter("my_param", 1, desc);

// 注册参数回调（必须持有 handle！）
param_cb_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        // ... 校验逻辑 ...
        return result;
    });

// 读取参数
int val = this->get_parameter("my_param").as_int();

// 订阅参数事件
sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", rclcpp::QoS(10), callback);
```

## 注意事项

- `add_on_set_parameters_callback` 返回的 `OnSetParametersCallbackHandle::SharedPtr`
  **必须**保存为成员变量，否则 handle 析构后回调立即失效。
- YAML 文件中参数名必须与 `declare_parameter` 完全一致，否则会因"未声明参数"
  而报错（ROS 2 Humble+ 默认开启严格模式）。
- `/parameter_events` 会收到**所有节点**的参数事件，生产代码中需用
  `event.node` 过滤目标节点。
