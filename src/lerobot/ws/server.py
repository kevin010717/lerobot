from lerobot.async_inference.policy_server import serve

if __name__ == "__main__":
    import sys
    from lerobot.utils.utils import init_logging
    init_logging("/home/robot/lerobot/outputs/policy_server-log.txt")
    # init_logging("/home/robot/lerobot/outputs/policy_server-log.txt",console_level="DEBUG")
    sys.argv = [
        "policy_server.py",
        "--host=0.0.0.0",
        "--port=8888",
        "--fps=30",
        # "--inference_latency=0",
        # "--obs_queue_timeout=1",
    ]
    serve()  # run the client
