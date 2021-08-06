# OAK_D Complete Interface for ROS
The OakDInterface class manages the operation of the package

First, the OAK-D is configured using the OakDPipeline class. The desired parameters can be configured in the lib/oakd_pipeline.cpp file (information available at: https://docs.luxonis.com/projects/api/en/latest/references/cpp/)

Images are published using the following classes. Its configuration is done through the oak.launch file:
- OakDTaskMono class (lib/oakd_task_mono.cpp file): Publish Stereo Images
- OakDTaskRectified class (lib/oakd_task_rectified.cpp file): Publish Stereo Rectified Images
- OakDTaskDepth class (lib/oakd_task_depth.cpp file): Publish Depth Images
- OakDTaskColor class (lib/oakd_task_color.cpp file): Publish RGB/BGR Images

For the use of a network with inference in the color camera:
1. Set .blob file path and network configuration in oak.launch file
2. Change in OakDTaskColorNeuralInference class (lib/oakd_task_color_neural_inference.cpp file) the desired label_map

For the use of a network with inference in stereo cameras:
1. Set .blob file path and network configuration in oak.launch file
2. Change in OakDTaskStereoNeuralInference class (lib/oakd_task_stereo_neural_inference.cpp file) the desired label_map

To add new features named TASK:
1. In include/oak_interface/oakd_task.hpp:
   * Add publish_TASK at OakPublishList to enable it in oak.launch
   * Add use_TASK at OakUseList with OAK-D nodes needed
   * Add inx_TASK at OakQueueIndex with OAK-D information needed

2. In lib/oakd_interface.hpp:
   * Add publish_TASK in OakDInterface::read_param to read from oak.launch
   * Add use_TASK in OakDInterface::create_use_list to use all OAK-D nodes needed
   * Add OakDTaskTASK class in OakDInterface::ownSetUp to add it to tasks_list_

3. In oakd_pipeline.cpp:
   * Make the desired configuration
   * Configure streams_queue pipeline

4. Create OakDTaskTASK class that inherits from OakDTask (oakd_task.hpp).
   It has to override the methods:
    - void start(ros::NodeHandle& nh);
    - void run(std::vector<std::shared_ptr<dai::DataOutputQueue>>& streams_queue, 
             OakQueueIndex& queue_index, std_msgs::Header header);
	* Where: streams_queue is OAK-D information queue and queue_index are their index in the queue
    - void stop();
