pip install datasets==2.19 # 解决 datasets 版本过高导致的问题
pip install pytest # ==7.4.0 # 解决 pytest 版本过高导致的问题
export HF_ENDPOINT=https://hf-mirror.com # 解决huggingface访问慢的问题

sudo apt-get update
sudo apt-get install -y libosmesa6 libosmesa6-dev

sudo apt-get update
sudo apt-get install -y \
  libgl1-mesa-glx \
  libgl1-mesa-dri \
  freeglut3-dev \
  libglu1-mesa \
  libglew-dev

sudo apt-get install mesa-utils
sudo apt-get install libgl1-mesa-dev

glxinfo | grep OpenGL 


# 数据采集经验
# 1.单手操作
# 2.夹爪回正
# 3.先对准，再夹取
# 4.下探过程缓慢

# todo:
# *用checkpoint推理
# *lerobot 0.4.2迁移
# *添加相机支架
# *smolla训练数据格式不匹配，diffusion推理不动
# ---------------------------------- 跑eval脚本 接入仿真环境    gym+mujoco{优先}    libero{gym+mujoco+robosuit}    metaworld{gym+mujoco}   isaac{等lerobot接入}--------------------------------- #
# -------------------------------- 顶部相机安装  等支架 ------------------------------- #
# ------------------------- pi groot数据采集，训练，推理  等服务器 ------------------------- #
# ----------------- t700机器人数据采集系统搭建                  11.31开工 ----------------- #
# *vr，视觉遥操系统搭建，关联仿真，产线数据采集，数据增强
# *openvla openpi opengalaxea框架
# *VLN

# 100组数据，act策略，抓取包装盒到泡沫垫上，成功率85%，
#    ACT抓取任务成功率统计：83%（83/100）
#0   0-*-2-*-*-5-6-*-8-9
#1   0-1-2-*-4-5-*-7-8-9
#2   0-1-2-3-*-*-6-7-8-9
#3   0-1-2-*-4-5-6-7-8-9
#4   0-1-*-3-4-5-6-7-*-9
#5   0-1-2-*-4-5-6-7-8-9-*
#6   0-1-2-3-4-5-6-7-8-9
#7   0-1-2-3-4-5-6-*-*-9
#8   0-1-2-3-4-*-6-7-8-*
#9   0-1-2-3-4-5-*-7-8-9

            metrics
episodes:    25    50    75    100
loss:   
grdn:
lr:
steps:
time:
success rate: