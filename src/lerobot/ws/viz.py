if __name__ == "__main__":
    from lerobot.scripts.lerobot_dataset_viz import visualize_dataset
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    dataset = LeRobotDataset("seeedstudio123/test","/home/robot/lerobot/outputs/11-19/record")
    visualize_dataset(dataset, mode="local", web_port=9090, ws_port=9087, save=0,episode_index=1)