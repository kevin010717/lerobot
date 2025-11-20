class RobotClient:
    prefix = "robot_client"
    logger = get_logger(prefix)

    def __init__(self, config: RobotClientConfig):
        """根据配置初始化机器人 client。"""

        self.config = config
        # 从 RobotConfig 创建具体机器人实例（软硬件抽象）
        self.robot = make_robot_from_config(config.robot)
        self.robot.connect()

        # 把机器人自身的观测 key 映射成 LeRobot 统一的 feature 名（observation.images.xxx 等）
        lerobot_features = map_robot_keys_to_lerobot_features(self.robot)

        # server 地址
        self.server_address = config.server_address

        # 组织好要发给 server 的 RemotePolicyConfig（policy 类型、路径、device 等）
        self.policy_config = RemotePolicyConfig(
            config.policy_type,
            config.pretrained_name_or_path,
            lerobot_features,
            config.actions_per_chunk,
            config.policy_device,
        )
        # 建立 gRPC channel
        self.channel = grpc.insecure_channel(
            self.server_address, grpc_channel_options(initial_backoff=f"{config.environment_dt:.4f}s")
        )
        self.stub = services_pb2_grpc.AsyncInferenceStub(self.channel)
        self.logger.info(f"Initializing client to connect to server at {self.server_address}")

        self.shutdown_event = threading.Event()

        # 下面是动作相关的本地状态
        self.latest_action_lock = threading.Lock()
        self.latest_action = -1  # 当前已经执行到的最后一个 timestep
        self.action_chunk_size = -1  # 记录历史上收到的最大 chunk 长度

        self._chunk_size_threshold = config.chunk_size_threshold  # 控制发送 obs 的节奏

        # 动作队列 + 保护锁
        self.action_queue = Queue()
        self.action_queue_lock = threading.Lock()
        self.action_queue_size = []  # 用于 debug 可视化队列长度

        # 控制两条线程同步启动的 barrier：1) receive_actions 2) control_loop
        self.start_barrier = threading.Barrier(2)

        # FPS 统计（发送 obs 的频率）
        self.fps_tracker = FPSTracker(target_fps=self.config.fps)

        self.logger.info("Robot connected and ready")

        # must_go 事件：True 表示下次队列空时的 obs 必须被处理
        self.must_go = threading.Event()
        self.must_go.set()  # 初始化为必须处理（第一次一定要送一次）

    @property
    def running(self):
        return not self.shutdown_event.is_set()

    def start(self):
        """连接 policy server 并发送 policy 配置。"""
        try:
            # 1. 握手：调用 Ready，重置 server 状态
            start_time = time.perf_counter()
            self.stub.Ready(services_pb2.Empty())
            end_time = time.perf_counter()
            self.logger.debug(f"Connected to policy server in {end_time - start_time:.4f}s")

            # 2. 发送 RemotePolicyConfig
            policy_config_bytes = pickle.dumps(self.policy_config)
            policy_setup = services_pb2.PolicySetup(data=policy_config_bytes)

            self.logger.info("Sending policy instructions to policy server")
            self.logger.debug(
                f"Policy type: {self.policy_config.policy_type} | "
                f"Pretrained name or path: {self.policy_config.pretrained_name_or_path} | "
                f"Device: {self.policy_config.device}"
            )

            self.stub.SendPolicyInstructions(policy_setup)

            self.shutdown_event.clear()

            return True

        except grpc.RpcError as e:
            self.logger.error(f"Failed to connect to policy server: {e}")
            return False

    def send_observation(
        self,
        obs: TimedObservation,
    ) -> bool:
        """把一个 TimedObservation 发给 server。"""
        if not self.running:
            raise RuntimeError("Client not running. Run RobotClient.start() before sending observations.")

        if not isinstance(obs, TimedObservation):
            raise ValueError("Input observation needs to be a TimedObservation!")

        # 序列化
        start_time = time.perf_counter()
        observation_bytes = pickle.dumps(obs)
        serialize_time = time.perf_counter() - start_time
        self.logger.debug(f"Observation serialization time: {serialize_time:.6f}s")

        try:
            # 分 chunk 通过 gRPC 流发送
            observation_iterator = send_bytes_in_chunks(
                observation_bytes,
                services_pb2.Observation,
                log_prefix="[CLIENT] Observation",
                silent=True,
            )
            _ = self.stub.SendObservations(observation_iterator)
            obs_timestep = obs.get_timestep()
            self.logger.debug(f"Sent observation #{obs_timestep} | ")

            return True

        except grpc.RpcError as e:
            self.logger.error(f"Error sending observation #{obs.get_timestep()}: {e}")
            return False

    def receive_actions(self, verbose: bool = False):
        """子线程：持续从 server 拉取 action chunk，并更新本地 action_queue。"""
        # 等待另一条线程（control_loop）到齐再一起启动
        self.start_barrier.wait()
        self.logger.info("Action receiving thread starting")

        while self.running:
            try:
                # 调用 GetActions：server 会针对最新 obs 生成一个 action chunk
                actions_chunk = self.stub.GetActions(services_pb2.Empty())
                if len(actions_chunk.data) == 0:
                    # server 返回 Empty（比如没 obs），那就下次再试
                    continue

                receive_time = time.time()

                # 反序列化成 list[TimedAction]
                deserialize_start = time.perf_counter()
                timed_actions = pickle.loads(actions_chunk.data)  # nosec
                deserialize_time = time.perf_counter() - deserialize_start

                # 记录 chunk 长度（影响 _ready_to_send_observation 的逻辑）
                self.action_chunk_size = max(self.action_chunk_size, len(timed_actions))

                # 这里 verbose 下会打印延迟、队列变化等 debug 信息（可选）
                # 省略注释...

                # 合并到本地队列（相同 timestep 用 aggregate_fn 聚合）
                start_time = time.perf_counter()
                self._aggregate_action_queues(timed_actions, self.config.aggregate_fn)
                queue_update_time = time.perf_counter() - start_time

                # 一旦收到新的动作，说明“下一个队列为空时的 obs”又可以触发 must_go 了
                self.must_go.set()

            except grpc.RpcError as e:
                self.logger.error(f"Error receiving actions: {e}")

    def control_loop_action(self, verbose: bool = False) -> dict[str, Any]:
        """从本地 action_queue 拿一个 TimedAction，发送到机器人。"""
        get_start = time.perf_counter()
        with self.action_queue_lock:
            self.action_queue_size.append(self.action_queue.qsize())
            timed_action = self.action_queue.get_nowait()
        get_end = time.perf_counter() - get_start

        # 把 tensor 转成机器人能理解的 dict（关节名 -> 数值）
        _performed_action = self.robot.send_action(
            self._action_tensor_to_action_dict(timed_action.get_action())
        )
        with self.latest_action_lock:
            self.latest_action = timed_action.get_timestep()

        # verbose 下会多打印一些调试信息，这里略
        return _performed_action

    def _ready_to_send_observation(self):
        """控制何时发送新的 observation：队列太满就先不发。"""
        with self.action_queue_lock:
            # qsize / chunk_size <= 阈值 时才发
            return self.action_queue.qsize() / self.action_chunk_size <= self._chunk_size_threshold

    def control_loop_observation(self, task: str, verbose: bool = False) -> RawObservation:
        """采集一次观察，并根据队列状态决定 must_go，再发给 server。"""
        try:
            start_time = time.perf_counter()

            # 从 robot 采一次 idom observation（通常包含图像、关节状态等）
            raw_observation: RawObservation = self.robot.get_observation()
            raw_observation["task"] = task  # 附上任务名（给 policy 做条件）

            with self.latest_action_lock:
                latest_action = self.latest_action

            # 把 obs 包装成 TimedObservation（timestamp + timestep + must_go）
            observation = TimedObservation(
                timestamp=time.time(),
                observation=raw_observation,
                timestep=max(latest_action, 0),
            )

            obs_capture_time = time.perf_counter() - start_time

            # 队列为空 & must_go=true 的时候，这个 obs 会被标记为 must_go
            with self.action_queue_lock:
                observation.must_go = self.must_go.is_set() and self.action_queue.empty()
                current_queue_size = self.action_queue.qsize()

            _ = self.send_observation(observation)

            self.logger.debug(f"QUEUE SIZE: {current_queue_size} (Must go: {observation.must_go})")
            if observation.must_go:
                # 一旦触发了一次 must_go，立刻清掉
                # 等 server 推了一批新动作后，receive_actions 再把 must_go 设回 True
                self.must_go.clear()

            # verbose 下会输出 FPS 等信息，这里略
            return raw_observation

        except Exception as e:
            self.logger.error(f"Error in observation sender: {e}")

    def control_loop(self, task: str, verbose: bool = False) -> tuple[Observation, Action]:
        """主线程控制 loop：交替“执行动作”和“发 obs”。"""
        # 等待 receive_actions 线程，保证两边同时开始
        self.start_barrier.wait()
        self.logger.info("Control loop thread starting")

        _performed_action = None
        _captured_observation = None

        while self.running:
            control_loop_start = time.perf_counter()
            # (1) 如果有动作，就从队列拿一个执行
            if self.actions_available():
                _performed_action = self.control_loop_action(verbose)

            # (2) 根据队列“饱和度”决定是否采 obs 并发送
            if self._ready_to_send_observation():
                _captured_observation = self.control_loop_observation(task, verbose)

            self.logger.debug(f"Control loop (ms): {(time.perf_counter() - control_loop_start) * 1000:.2f}")
            # 根据 environment_dt 控制整个 loop 的频率
            time.sleep(max(0, self.config.environment_dt - (time.perf_counter() - control_loop_start)))

        return _captured_observation, _performed_action
