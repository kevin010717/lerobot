pip install datasets==2.19 # 解决 datasets 版本过高导致的问题
pip install pytest # ==7.4.0 # 解决 pytest 版本过高导致的问题
export HF_ENDPOINT=https://hf-mirror.com # 解决huggingface访问慢的问题


# 数据采集经验
# 1.单手操作
# 2.夹爪回正
# 3.先对准，再夹取
# 4.下探过程缓慢

# todo:
# *用checkpoint推理
# *lerobot 0.4.2迁移
# smolla训练数据格式不匹配，diffusion推理不动
# pi groot数据采集，训练，推理
# 为策略增加prompt输入
# t700机器人数据采集系统搭建                  11.31开工
# vr，视觉遥操系统搭建，关联仿真，产线数据采集，数据增强
# openvla openpi opengalaxea框架
# VLN

# 实验结果：
# 数据：100组
# 策略：act diffusion smolvla
# 任务：抓取包装盒到泡沫垫上
# 结果：act最好，smolvla次之，diffusion最差
# 结论：小数据集下，act更适合；diffusion需要更大数据集
# 后续：增加数据集规模到300组，复现实验
# 实验结果2：
# 数据：300组
# 策略：act diffusion smolvla
# 任务：抓取包装盒到泡沫垫上
# 结果：act最好，smolvla次之，diffusion最差
# 结论：小数据集下，act更适合；diffusion需要更大数据集
# 后续：增加数据集规模到500组，复现实验
# 实验结果3：
# 数据：500组
# 策略：act diffusion smolvla
# 任务：抓取包装盒到泡沫垫上
# 结果：act最好，smolvla次之，diffusion最差
# 结论：小数据集下，act更适合；diffusion需要更大数据集
# 后续：增加数据集规模到1000组，复现实验        
# 实验结果4：
# 数据：1000组
# 策略：act diffusion smolvla
# 任务：抓取包装盒到泡沫垫上
# 结果：act最好，smolvla次之，diffusion最差
# 结论：小数据集下，act更适合；diffusion需要更大数据集
# 后续：增加数据集规模到2000组，复现实验        