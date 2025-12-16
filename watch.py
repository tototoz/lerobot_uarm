import pandas as pd

# 读取 parquet 文件
df = pd.read_parquet('/home/zhq/lerobot/src/lerobot/datasets/6zi/data/chunk-000/file-000.parquet')

# 保存为 CSV 文件，不包括索引
df.to_csv('test2.csv', index=False)

# 输出保存成功的信息
print("Data has been saved to 'output.csv'.")
