## Paper

https://arxiv.org/abs/2506.01844

## Citation

```bibtex
@article{shukor2025smolvla,
  title={SmolVLA: A Vision-Language-Action Model for Affordable and Efficient Robotics},
  author={Shukor, Mustafa and Aubakirova, Dana and Capuano, Francesco and Kooijmans, Pepijn and Palma, Steven and Zouitine, Adil and Aractingi, Michel and Pascal, Caroline and Russi, Martino and Marafioti, Andres and Alibert, Simon and Cord, Matthieu and Wolf, Thomas and Cadene, Remi},
  journal={arXiv preprint arXiv:2506.01844},
  year={2025}
}
```
lerobot-train \
  --dataset.repo_id=none \
  --dataset.root=/home/zhq/lerobot/src/lerobot/datasets/6zi \
  --policy.type=smolvla \
  --output_dir=outputs/train/smolvla \
  --job_name=act_your_dataset \
  --policy.device=cuda \
  --wandb.enable=false \
  --policy.repo_id=none \
  --policy.push_to_hub=false \
  --steps=2000 \
  --policy.training_time_rtc=true \
  --policy.rtc_max_delay=10
