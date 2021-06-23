#ifndef OAKD_PROCESS
#define OAKD_PROCESS

#include <string>
#include <stdio.h>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>


#define STATE_CREATED 1
#define STATE_READY_TO_START 2
#define STATE_RUNNING 3
#define STATE_PAUSED 4

#define STATE_STARTED 7
#define STATE_NOT_STARTED 8

class OakDProcess
{

public:
  using State = uint8_t;

protected:
  ros::NodeHandle node_handler_robot_process;

protected:
  State current_state;

  // methods
public:
  //! Constructor.
  OakDProcess();

  ~OakDProcess();

  void setUp();

  void start();

  void stop();

  void run();

  State getState();

  void setState(State new_state);

protected:

  virtual void ownSetUp() = 0;

  virtual void ownStart() = 0;

  virtual void ownStop() = 0;

  virtual void ownRun() = 0;
};
#endif

