from lerobot.scripts.lerobot_eval import eval_main

if __name__ == "__main__":
    import sys
    from lerobot.utils.utils import init_logging

    sys.argv = [
        "eval.py",
        "--policy.path=outputs/11-19/act/checkpoints/last/pretrained_model",
        "--env.type=libero",  #aloha   pusht   libero
        # ❌ 不要加 --env.render=true
        "--eval.batch_size=10",
        "--eval.n_episodes=10",
        # "--eval.max_episodes_rendered=10",  # ✅ 让 eval 保存 10 条视频
        "--policy.use_amp=false",
        "--policy.device=cuda",
        '--rename_map={"observation.images.top": "observation.images.camera1"}',
    ]

    init_logging("/home/robot/lerobot/outputs/eval-log.txt")
    eval_main()
