# URDF与电机的连接

​	在ROS（Robot Operating System）与Gazebo仿真环境的集成中，gazebo_ros_control插件扮演了一个关键角色。它作为桥梁，连接了机器人模型和控制算法，使得我们能够在Gazebo中模拟机器人的行为。这个插件通过解析URDF（Unified Robot Description Format）文件中的<transmission>标签，并加载相应的硬件接口和控制器管理器来实现这一目标。

​	具体来说，在URDF文件中，<hardwareInterface>标签用于定义机器人关节或执行器的接口类型。这些接口类型包括位置、速度或力矩（effort）接口，它们分别对应于不同的控制模式：基于位置的控制、基于速度的控制以及基于力矩的控制。例如，如果你想控制一个电机的位置，那么你可能会使用PositionJointInterface；如果要控制电机的速度，则可能选择VelocityJointInterface；而如果你希望直接控制施加到电机上的力矩，则可以使用EffortJointInterface。

​	ros::NodeHandle(controller_nh, "pid") 创建了一个新的节点句柄，它在 controller_nh 的基础上添加了 "pid" 子命名空间，这样做会形成类似 controller_nh/pid/... 的参数路径

# 使用override注意事项

​	在C++中，override 关键字用于显式地标记一个成员函数是覆盖（override）基类中的虚函数。为了使用 override，函数的签名必须与基类中的虚函数完全匹配，包括 const 限定符。
在你的代码中，update 函数的签名是：

```cpp
void update(const ros::Time &time, const ros::Duration &period) override;
```

​	这里的 const 限定符表示 update 函数不会修改类的成员变量。如果你去掉 const，函数的签名就不再与基类中的虚函数匹配，因此编译器会报错，不允许使用 override。

### 为什么需要 const？

​	函数签名匹配：基类中的虚函数可能是 const 成员函数，因此派生类中的覆盖函数也必须是 const 的。
语义一致性：const 成员函数表示该函数不会修改对象的状态，这有助于确保代码的正确性和可维护性。

# 在代码中读取关节

```cpp
joint_ = effort_joint_interface->getHandle("rotating_joint");
```
