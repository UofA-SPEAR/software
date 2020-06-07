The arm has several components (this is probably inaccurate in some way -- please update it if something is wrong)

- Commands are sent to the motors over CAN using an ArrayCommand
- `mapper.py` converts the `/arm/angles` topic and sends it out to CAN
- moveit converts some input topic to `/arm/angles`
- Various arm control GUIs output either `/arm/angles` directly to control the arm without IK, or output moveit's input topic to control the arm with IK

Also,
- rviz reads `/joint_states` to visualize the arm (via `tf` or with a model if you've loaded an arm model)