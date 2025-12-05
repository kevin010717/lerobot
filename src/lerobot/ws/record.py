from lerobot.ws.gello_leader import gello_leader
from lerobot.ws import cr5af_follower  # FOR CR5AF
from lerobot.scripts.lerobot_record import record, RecordConfig


if __name__ == "__main__":
    import sys

    while True:
        print("\n=== 选择 ===")
        print("0) 录制数据集")
        print("1) act")
        print("2) diffusion")
        print("3) smolvla")
        print("q) 退出程序")
        choice = input("请输入选择 (0/1/2/3/q): ").strip().lower()



        import subprocess
        subprocess.run(["sudo", "chmod", "777", "/dev/ttyACM0"], check=True)
        subprocess.run(["sudo", "chmod", "777", "/dev/ttyUSB0"], check=True)

        if choice == "q":
            print("退出程序")
            break
        elif choice == "1":
                sys.argv = [
                "record.py",
                "--robot.type=cr5af_follower",
                "--robot.port=/dev/ttyACM1",
                "--robot.cameras={camera1: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}",
                "--robot.id=my_awesome_follower_arm",
                "--display_data=false",
                "--dataset.repo_id=seeed/eval_test123",
                "--dataset.root=/home/robot/lerobot/outputs/eval1111",                                            # data root
                "--dataset.single_task=Put lego brick into the transparent box",
                "--policy.path=outputs/11-19/act/checkpoints/last/pretrained_model",
            ]
        elif choice == "2":
            sys.argv = [
            "record.py",
            "--robot.type=cr5af_follower",
            "--robot.port=/dev/ttyACM1",
            "--robot.cameras={camera1: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}",
            "--robot.id=my_awesome_follower_arm",
            "--display_data=false",
            "--dataset.repo_id=seeed/eval_test123",
            "--dataset.root=/home/robot/lerobot/outputs/eval1111",                                            # data root
            "--dataset.single_task=Put lego brick into the transparent box",
            "--policy.path=outputs/11-19/diffusion/checkpoints/last/pretrained_model",
        ]
        elif choice == "3":
            sys.argv = [
            "record.py",
            "--robot.type=cr5af_follower",
            "--robot.port=/dev/ttyACM1",
            "--robot.cameras={camera1: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}", #相机 index_or_path 0 - 3
            "--robot.id=my_awesome_follower_arm",
            "--display_data=false",
            "--dataset.repo_id=seeed/eval_test123",         
            "--dataset.root=/home/robot/lerobot/outputs/eval1111",                                            # data root
            "--dataset.single_task=Put lego brick into the transparent box",
            "--policy.path=outputs/11-19/smolvla/checkpoints/last/pretrained_model",                              # pretrain model
        ]
        elif choice == "0":
            sys.argv = [
            "record.py",

            # Robot（CR5 跟随臂 + 相机在 robot 命名空间） 夹爪
            "--robot.type=cr5af_follower",
            "--robot.port=/dev/ttyACM1",
            "--robot.id=my_awesome_follower_arm",
            # '--robot.cameras={"front": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}}',
            '--robot.cameras={"camera1": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}}',

            # Teleop（主控臂）
            "--teleop.type=gello_leader",
            "--teleop.port=/dev/ttyUSB0",
            "--teleop.id=my_awesome_leader_arm",
            "--teleop.use_degrees=true",

            # Dataset（只本地）
            "--dataset.repo_id=seeedstudio123/test",
            "--dataset.root=/home/robot/lerobot/outputs/11-19/recordtest1",                    # data root
            "--dataset.single_task=Pick up the packaging box and place it onto the foam pad.",
            "--dataset.num_episodes=2",
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
        else:
            print("输入无效，请重新选择")
            continue

        try:
            from lerobot.utils.utils import init_logging
            init_logging()
            record()
            # init_logging("/home/robot/lerobot/outputs/robot_client-log.txt",console_level="DEBUG")
            # while True:
            #     import time
            #     time.sleep(1)
            #     print("启动客户端...")
        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，停止当前策略，返回菜单切换策略...")
            # 继续 while 循环，就能重新选另一个策略
            continue