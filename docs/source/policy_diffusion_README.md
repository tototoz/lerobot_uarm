## Paper

https://diffusion-policy.cs.columbia.edu

## Citation

```bibtex
@article{chi2024diffusionpolicy,
	author = {Cheng Chi and Zhenjia Xu and Siyuan Feng and Eric Cousineau and Yilun Du and Benjamin Burchfiel and Russ Tedrake and Shuran Song},
	title ={Diffusion Policy: Visuomotor Policy Learning via Action Diffusion},
	journal = {The International Journal of Robotics Research},
	year = {2024},
}
```
lerobot-train \
  --dataset.repo_id=datasets/gai0 \
  --dataset.root=/home/zhq/lerobot/src/lerobot/datasets/6zi \
  --policy.type=act \
  --output_dir=outputs/train/act_your_dataset \
  --job_name=act_your_dataset \
  --policy.device=cuda \
  --wandb.enable=false \
  --policy.repo_id=none \
  --policy.push_to_hub=false \
  --steps=2000
