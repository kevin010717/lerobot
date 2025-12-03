if __name__ == "__main__":
    import sys
    from lerobot.utils.utils import init_logging
    from lerobot.scripts.lerobot_train import train
    # ---------------------------------------------------------------------------- #
    #                               在代码里“伪造”命令行参数 act                              #
    # ---------------------------------------------------------------------------- #
    episodes = list(range(25))  # [0,     1, ..., 79]
    print(f"Training on episodes: {episodes}")
    sys.argv = [
        "train.py",
        "--dataset.repo_id=seeedstudio123/test",
        "--dataset.root=/home/robot/lerobot/outputs/11-19/record",   # data root
        f"--dataset.episodes={episodes}",  # 关键这一行
        "--policy.type=act",
        "--output_dir=outputs/11-19/act25",
        "--job_name=act_so101_test",
        "--policy.device=cuda",
        "--wandb.enable=true",
        "--policy.push_to_hub=false",
        "--steps=2000",
        # "--save_freq=200",
        "--batch_size=8",
    ]
    init_logging('outputs/11-19/act-log.txt')
    train()  # 不要传 cfg，@parser.wrap 会用 sys.argv 解析并调用真正的 train(cfg)
    # ---------------------------------------------------------------------------- #
    #                                      50                                      #
    # ---------------------------------------------------------------------------- #
    # episodes = list(range(50))  # [0,     1, ..., 79]
    # print(f"Training on episodes: {episodes}")
    # sys.argv = [
    #     "train.py",
    #     "--dataset.repo_id=seeedstudio123/test",
    #     "--dataset.root=/home/robot/lerobot/outputs/11-19/record",   # data root
    #     f"--dataset.episodes={episodes}",  # 关键这一行
    #     "--policy.type=act",
    #     "--output_dir=outputs/11-19/act50",
    #     "--job_name=act_so101_test",
    #     "--policy.device=cuda",
    #     "--wandb.enable=false",
    #     "--policy.push_to_hub=false",
    #     "--steps=50000",
    #     # "--save_freq=200",
    #     "--batch_size=8",
    # ]
    # init_logging('outputs/11-19/act-log.txt')
    # train()  # 不要传 cfg，@parser.wrap 会用 sys.argv 解析并调用真正的 train(cfg)
    # ---------------------------------------------------------------------------- #
    #                                      75                                      #
    # ---------------------------------------------------------------------------- #
    # episodes = list(range(75))  # [0,     1, ..., 79]
    # print(f"Training on episodes: {episodes}")
    # sys.argv = [
    #     "train.py",
    #     "--dataset.repo_id=seeedstudio123/test",
    #     "--dataset.root=/home/robot/lerobot/outputs/11-19/record",   # data root
    #     f"--dataset.episodes={episodes}",  # 关键这一行
    #     "--policy.type=act",
    #     "--output_dir=outputs/11-19/act75",
    #     "--job_name=act_so101_test",
    #     "--policy.device=cuda",
    #     "--wandb.enable=false",
    #     "--policy.push_to_hub=false",
    #     "--steps=75000",
    #     # "--save_freq=200",
    #     "--batch_size=8",
    # ]
    # init_logging('outputs/11-19/act-log.txt')
    # train()  # 不要传 cfg，@parser.wrap 会用 sys.argv 解析并调用真正的 train(cfg)
    # ---------------------------------------------------------------------------- #
    #                                      100                                     #
    # ---------------------------------------------------------------------------- #
    # episodes = list(range(100))  # [0,     1, ..., 79]
    # print(f"Training on episodes: {episodes}")
    # sys.argv = [
    #     "train.py",
    #     "--dataset.repo_id=seeedstudio123/test",
    #     "--dataset.root=/home/robot/lerobot/outputs/11-19/record",   # data root
    #     f"--dataset.episodes={episodes}",  # 关键这一行
    #     "--policy.type=act",
    #     "--output_dir=outputs/11-19/act100",
    #     "--job_name=act_so101_test",
    #     "--policy.device=cuda",
    #     "--wandb.enable=false",
    #     "--policy.push_to_hub=false",
    #     "--steps=100000",
    #     # "--save_freq=200",
    #     "--batch_size=8",
    # ]
    # init_logging('outputs/11-19/act-log.txt')
    # train()  # 不要传 cfg，@parser.wrap 会用 sys.argv 解析并调用真正的 train(cfg)
    # ---------------------------------------------------------------------------- #
    #                            在代码里“伪造”命令行参数 diffusion                           #
    # ---------------------------------------------------------------------------- #
    # sys.argv = [
    #     "train.py",
    #     "--dataset.root=/home/robot/lerobot/outputs/11-19/record",          # data root
    #     # f"--dataset.episodes={episodes}",  # 关键这一行
    #     "--dataset.repo_id=seeedstudio123/test",                            # data repo
    #     "--output_dir=outputs/11-19/diffusion",                               # outputs model
    #     "--policy.type=diffusion",
    #     "--job_name=act_so101_test",
    #     "--policy.device=cuda",
    #     "--wandb.enable=false",
    #     "--policy.push_to_hub=false",
    #     "--steps=100000",
    #     # "--save_freq=200",
    #     "--batch_size=8",
    # ]
    # init_logging('outputs/11-19/diffusion-log.txt')
    # train()  # 不要传 cfg，@parser.wrap 会用 sys.argv 解析并调用真正的 train(cfg)
    # ---------------------------------------------------------------------------- #
    #                             在代码里“伪造”命令行参数 smolvla                            #
    # ---------------------------------------------------------------------------- #
    # import os
    # os.environ["TOKENIZERS_PARALLELISM"] = "false"
    # os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"

    # sys.argv = [
    #     "train.py",
    #     "--dataset.root=/home/robot/lerobot/outputs/11-19/record",          # data root
    #     # f"--dataset.episodes={episodes}",  # 关键这一行
    #     "--dataset.repo_id=seeedstudio123/test",                            # data repo
    #     "--policy.path=outputs/smolvla_base",                               # model:pretrain 
    #     "--policy.vlm_model_name=outputs/SmolVLM2-500M-Video-Instruct",     # model:vlm
    #     "--output_dir=outputs/11-19/smolvla",                               # outputs model
    #     "--job_name=act_so101_test",
    #     "--policy.device=cuda",
    #     "--wandb.enable=false",
    #     "--policy.push_to_hub=false",
    #     "--steps=100000",
    #     # "--save_freq=200",
    #     "--batch_size=8",
    # ]
    # init_logging('outputs/11-19/smolvla-log.txt')
    # train()  # 不要传 cfg，@parser.wrap 会用 sys.argv 解析并调用真正的 train(cfg)
    # ---------------------------------------------------------------------------- #
    #                            在代码里“伪造”命令行参数 Pi0                           #
    # ---------------------------------------------------------------------------- #
    # sys.argv = [
    #     "train.py",
    #     "--dataset.root=/home/robot/lerobot/outputs/11-14/record-test",          # data root
    #     # f"--dataset.episodes={episodes}",  # 关键这一行
    #     "--dataset.repo_id=seeedstudio123/test",                            # data repo
    #     # "--policy.vlm_model_name=outputs/paligemma-3b-pt-224",            # model:vlm

    #     "--output_dir=outputs/11-14/pi05",                                   # outputs model
    #     "--policy.type=groot",
    #     "--job_name=act_so101_test",
    #     "--policy.device=cuda",
    #     "--wandb.enable=false",
    #     "--policy.push_to_hub=false",
    #     "--steps=20000",
    #     "--save_freq=200",
    #     "--batch_size=8",
    # ]
    # init_logging()
    # train()  # 不要传 cfg，@parser.wrap 会用 sys.argv 解析并调用真正的 train(cfg)
    # ---------------------------------------------------------------------------- #
    #                            在代码里“伪造”命令行参数 Pi0.5                           #
    # ---------------------------------------------------------------------------- #

    # ---------------------------------------------------------------------------- #
    #                            在代码里“伪造”命令行参数 GROOT N1.5                           #
    # ---------------------------------------------------------------------------- #


