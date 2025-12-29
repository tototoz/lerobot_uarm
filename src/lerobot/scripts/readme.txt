# 校准
lerobot-calibrate \
    --robot.type=so100_follower \
    --robot.port=/dev/ttyACM2 \
    --robot.id=so100_follower_black

lerobot-calibrate \
    --teleop.type=so100_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=so100_leader_white

# 遥操  
lerobot-teleoperate \
    --robot.type=so100_follower \
    --robot.port=/dev/ttyACM2 \
    --robot.id=so100_follower_black \
    --teleop.type=so100_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=so100_leader_white \
    --display_data=true

 # 使用摄像头进行遥操 
lerobot-teleoperate \
    --robot.type=so100_follower \
    --robot.port=/dev/ttyACM2 \
    --robot.cameras='{
    top: {"type": "opencv", "index_or_path": 8, "width": 640, "height": 480, "fps": 30},
    laptop: {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}
    }' \
    --robot.id=so100_follower_black \
    --teleop.type=so100_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=so100_leader_white \
    --display_data=true

# 数据采集
lerobot-record \
  --robot.type=so100_follower \
  --robot.port=/dev/ttyACM2 \
  --robot.id=so100_follower_black \
  --robot.cameras='{
    laptop: {"type": "opencv", "index_or_path": 11, "width": 640, "height": 480, "fps": 30}
  }' \
  --teleop.type=so100_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=so100_leader_white \
  --display_data=true \
  --dataset.num_episodes=15 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=10 \
  --dataset.root=/home/xbot/lerobot_new/src/lerobot/datasets/btt \
  --dataset.repo_id=btt/so100-handover-cube \
  --dataset.single_task="btt_experiment" \
  --dataset.push_to_hub=False 


# 数据采集(带深度相机)
lerobot-record \
  --robot.type=so100_follower \
  --robot.port=/dev/ttyACM2 \
  --robot.id=so100_follower_black \
  --robot.cameras='{
    top: {"type": "opencv", "index_or_path": 8, "width": 640, "height": 480, "fps": 30},
    laptop: {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}
  }' \
  --teleop.type=so100_leader \
  --teleop.port=/dev/ttyACM1 \
  --teleop.id=so100_leader_white \
  --display_data=true \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=15 \
  --dataset.reset_time_s=8 \
  --dataset.root=/home/zhq/lerobot/src/lerobot/datasets/btt_expri_22 \
  --dataset.repo_id=btt/so100-catch-cube_expri \
  --dataset.single_task="Please put the yellow cube into the black box." \
  --dataset.push_to_hub=False 

# ceshi
lerobot-record \
  --robot.type=so100_follower \
  --robot.port=/dev/ttyACM1 \
  --robot.id=so100_follower_black \
  --robot.cameras='{
    top: {"type": "opencv", "index_or_path": 4, "width": 640, "height": 480, "fps": 30},
    laptop: {"type": "opencv", "index_or_path": 8, "width": 640, "height": 480, "fps": 30}
  }' \
  --teleop.type=so100_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.id=so100_leader_white \
  --display_data=true \
  --dataset.num_episodes=2 \
  --dataset.episode_time_s=16 











  lerobot-edit-dataset \
    --repo_id hahazheng/expri \
    --push_to_hub false \
    --operation.type merge \
    --operation.repo_ids "['/data/zhq/lerobot/src/lerobot/datasets/expri_one', '/data/zhq/lerobot/src/lerobot/datasets/expri_two']"

hf upload hahazheng/try_hub /home/zhq/lerobot/src/lerobot/datasets/6zi --repo-type=dataset

hf upload hahazheng/bttexp /data/zhq/lerobot/outputs/try_hub --repo-type=model


unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
unset socks_proxy SOCKS_PROXY