from lerobot.ws.gello_leader import gello_leader
from lerobot.ws import cr5af_follower  # FOR CR5AF
from lerobot.scripts.lerobot_record import record, RecordConfig
if __name__ == "__main__":
    import sys
    from lerobot.utils.utils import init_logging
    import os
    import subprocess
    subprocess.run(["sudo", "chmod", "777", "/dev/ttyACM0"], check=True)
    subprocess.run(["sudo", "chmod", "777", "/dev/ttyUSB0"], check=True)
    # 录制
    sys.argv = [
    "record.py",

    # Robot（CR5 跟随臂 + 相机在 robot 命名空间）
    "--robot.type=cr5af_follower",
    "--robot.port=/dev/ttyACM1",
    "--robot.id=my_awesome_follower_arm",
    '--robot.cameras={"front": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}}',

    # Teleop（主控臂）
    "--teleop.type=gello_leader",
    "--teleop.port=/dev/ttyUSB0",
    "--teleop.id=my_awesome_leader_arm",
    "--teleop.use_degrees=true",

    # Dataset（只本地）
    "--dataset.repo_id=seeedstudio123/test",
    "--dataset.root=/home/robot/lerobot/outputs/11-14/record-test",                    # data root
    "--dataset.single_task=Grab the black cube",
    "--dataset.num_episodes=1",
    "--dataset.episode_time_s=6",
    "--dataset.reset_time_s=10",        # ← 拼写必须是 reset_time_s
    "--dataset.fps=30",
    "--dataset.video=true",
    "--dataset.push_to_hub=false",      # 不推 Hub，且不要提供 repo_id

    # 其他
    "--display_data=false",
    "--play_sounds=true",
    "--resume=false",
]
    # 推理
    # act
#     sys.argv = [
#     "record.py",
#     "--robot.type=cr5af_follower",
#     "--robot.port=/dev/ttyACM1",
#     "--robot.cameras={front: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}",
#     "--robot.id=my_awesome_follower_arm",
#     "--display_data=false",
#     "--dataset.repo_id=seeed/eval_test123",
#     "--dataset.root=/home/robot/lerobot/outputs/eval1111",                                            # data root
#     "--dataset.single_task=Put lego brick into the transparent box",
#     "--policy.path=outputs/act1111/checkpoints/last/pretrained_model",
# ]
    # diffusion
#     sys.argv = [
#     "record.py",
#     "--robot.type=cr5af_follower",
#     "--robot.port=/dev/ttyACM1",
#     "--robot.cameras={front: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}",
#     "--robot.id=my_awesome_follower_arm",
#     "--display_data=false",
#     "--dataset.repo_id=seeed/eval_test123",
#     "--dataset.root=/home/robot/lerobot/outputs/eval1111",                                            # data root
#     "--dataset.single_task=Put lego brick into the transparent box",
#     "--policy.path=outputs/11-14/diffusion/checkpoints/last/pretrained_model",
# ]
    # smolvla
#     sys.argv = [
#     "record.py",
#     "--robot.type=cr5af_follower",
#     "--robot.port=/dev/ttyACM1",
#     "--robot.cameras={front: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}",
#     "--robot.id=my_awesome_follower_arm",
#     "--display_data=false",
#     "--dataset.repo_id=seeed/eval_test123",                                                         # data repo
#     # "--dataset.root=/home/robot/lerobot/outputs/record",                                            # data root
#     "--dataset.single_task=Put lego brick into the transparent box",
#     "--policy.path=outputs/smolvlatest/checkpoints/last/pretrained_model",                              # pretrain model
# ]
    init_logging("/home/robot/lerobot/outputs/11-14/record-log-test.txt")
    record()