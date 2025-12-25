<!-- 带摄像头遥操 -->
lerobot-teleoperate \
    --robot.type=dk1_follower \
    --robot.port=/dev/tty.usbmodem00000000050C1 \
    --robot.joint_velocity_scaling=0.5 \
    --robot.cameras="{ 
        context: {type: opencv, index_or_path: 0, width: 1280, height: 720, fps: 30}, 
        wrist: {type: opencv, index_or_path: 1, width: 1280, height: 720, fps: 30}
      }" \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/tty.usbmodem58FA0824311 \
    --display_data=true

<!-- 数采 -->
lerobot-record \
    --robot.type=dk1_follower \
    --robot.port=/dev/tty.usbmodem00000000050C1 \
    --robot.joint_velocity_scaling=1.0 \
    --robot.cameras="{ 
        context: {type: opencv, index_or_path: 0, width: 640, height: 360, fps: 30}, 
        wrist: {type: opencv, index_or_path: 1, width: 640, height: 360, fps: 30}
      }" \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/tty.usbmodem58FA0824311 \
    --display_data=true \
    --dataset.repo_id=$USER/my_dataset \
    --dataset.push_to_hub=false \
    --dataset.num_episodes=50 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=15 \
    --dataset.single_task="My task description."
    --resume=true 

<!-- 推理 -->
lerobot-record  \
  --robot.type=dk1_follower \
  --robot.port=/dev/tty.usbmodem00000000050C1 \
  --robot.joint_velocity_scaling=0.5 \
  --robot.cameras="{ 
      context: {type: opencv, index_or_path: 0, width: 640, height: 360, fps: 30}, 
      wrist: {type: opencv, index_or_path: 1, width: 640, height: 360, fps: 30}
    }"
  --display_data=true \
  --dataset.repo_id=$USER/eval_my_model \
  --dataset.single_task="My task description." \
  --dataset.push_to_hub=false \
  --policy.path=outputs/my_model/checkpoints/last/pretrained_model