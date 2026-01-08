lerobot-record \
    --robot.type=dk1_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.joint_velocity_scaling=1.0 \
    --robot.cameras="{ 
        context: {type: opencv, index_or_path: 2, width: 640, height: 360, fps: 30}
      }" \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/ttyUSB0 \
    --display_data=true \
    --dataset.repo_id=hahazheng/my_dataset \
    --dataset.root=/home/zhq/lerobot/src/lerobot/datasets
    --dataset.push_to_hub=false \
    --dataset.num_episodes=50 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=15 \
    --dataset.single_task="My task description."