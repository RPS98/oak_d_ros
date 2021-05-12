#include <oak_interface/oakd_process.h>

OakDProcess::OakDProcess()
{
  current_state = STATE_CREATED;
}

OakDProcess::~OakDProcess(){
}

void OakDProcess::setUp()
{
  ownSetUp();
  setState(STATE_READY_TO_START);
}

void OakDProcess::start()
{
  setState(STATE_RUNNING);
  ownStart();
}

void OakDProcess::stop()
{
  setState(STATE_READY_TO_START);
  ownStop();
}

OakDProcess::State OakDProcess::getState()
{
  return current_state;
}

void OakDProcess::setState(State new_state)
{
  if (new_state == STATE_CREATED || new_state == STATE_READY_TO_START || new_state == STATE_RUNNING ||
      new_state == STATE_PAUSED || new_state == STATE_STARTED || new_state == STATE_NOT_STARTED)
  {
    current_state = new_state;
  }
  else
  {
    ROS_ERROR("In node %s, current state cannot be changed to new state %d", ros::this_node::getName().c_str(),
              new_state);
  }
}

void OakDProcess::run()
{
  if (current_state == STATE_RUNNING)
    ownRun();
}
