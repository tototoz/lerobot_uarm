## Paper

https://tonyzhaozh.github.io/aloha

## Citation

```bibtex
@article{zhao2023learning,
  title={Learning fine-grained bimanual manipulation with low-cost hardware},
  author={Zhao, Tony Z and Kumar, Vikash and Levine, Sergey and Finn, Chelsea},
  journal={arXiv preprint arXiv:2304.13705},
  year={2023}
}
```
accelerate launch 
lerobot-train \
  --dataset.repo_id=datasets/gai0 \
  --dataset.root=/home/zhq/lerobot/src/lerobot/datasets/6zi \
  --policy.type=act \
  --output_dir=outputs/train/act_your_dataset \
  --job_name=act_your_dataset \
  --policy.device=cuda \
  --wandb.enable=false \
  --policy.repo_id=hahazheng/try \
  --policy.push_to_hub=true \
  --steps=20

  export CUDA_VISIBLE_DEVICES=1



accelerate launch /home/zhq/.conda/envs/lerobot/bin/lerobot-train \
  --dataset.repo_id=none \
  --dataset.root=/home/zhq/lerobot/src/lerobot/datasets/gai0 \
  --policy.type=act \
  --output_dir=outputs/train/act_your_dataset \
  --job_name=act_your_dataset \
  --policy.device=cuda \
  --wandb.enable=false \
  --policy.repo_id=none \
  --policy.push_to_hub=false \
  --steps=2000



export HF_ENDPOINT=https://hf-mirror.com
