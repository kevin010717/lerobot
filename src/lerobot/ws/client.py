from lerobot.async_inference.robot_client import async_client
from lerobot.ws import cr5af_follower  # FOR CR5AF

if __name__ == "__main__":
    import sys

    while True:
        print("\n=== 选择策略 ===")
        print("1) act")
        print("2) diffusion")
        print("3) smolvla")
        print("q) 退出程序")
        choice = input("请输入选择 (1/2/3/q): ").strip().lower()



        import subprocess
        subprocess.run(["sudo", "chmod", "777", "/dev/ttyACM0"], check=True)
        subprocess.run(["sudo", "chmod", "777", "/dev/ttyUSB0"], check=True)

        if choice == "q":
            print("退出程序")
            break
        elif choice == "1":
            policy_type = "act"
            pretrained_path = "outputs/11-19/act/checkpoints/last/pretrained_model"
        elif choice == "2":
            policy_type = "diffusion"
            pretrained_path = "outputs/11-19/diffusion/checkpoints/last/pretrained_model"
        elif choice == "3":
            policy_type = "smolvla"
            pretrained_path = "outputs/11-19/smolvla/checkpoints/last/pretrained_model"
        else:
            print("输入无效，请重新选择")
            continue

        # 每次根据策略重写 sys.argv
        sys.argv = [
            "robot_client.py",

            # RobotClientConfig
            "--server_address", "127.0.0.1:8888",
            "--policy_type", policy_type,
            "--pretrained_name_or_path", pretrained_path,
            "--policy_device", "cuda",
            "--actions_per_chunk", "50",
            "--chunk_size_threshold", "0.5",
            "--aggregate_fn_name", "weighted_average",
            "--debug_visualize_queue_size", "true",
            "--fps", "30",

            # RobotConfig ['robot']
            "--robot.type", "cr5af_follower",
            "--robot.port", "/dev/ttyACM1",
            "--robot.id", "my_awesome_follower_arm",
            "--robot.disable_torque_on_disconnect", "true",
            "--robot.use_degrees", "false",
            "--robot.cameras",
            '{"camera1": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}}',
        ]

        print(f"\n>>> 启动策略: {policy_type}, 模型路径: {pretrained_path}")
        try:
            from lerobot.utils.utils import init_logging
            init_logging("/home/robot/lerobot/outputs/robot_client-log.txt")
            # init_logging("/home/robot/lerobot/outputs/robot_client-log.txt",console_level="DEBUG")
            async_client()  # 这里会阻塞直到 client 自己结束/异常/被 Ctrl+C 停止
            # while True:
            #     import time
            #     time.sleep(1)
            #     print("启动客户端...")
        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，停止当前策略，返回菜单切换策略...")
            # 继续 while 循环，就能重新选另一个策略
            continue

