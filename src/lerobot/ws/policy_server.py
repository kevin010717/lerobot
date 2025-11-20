class PolicyServer(services_pb2_grpc.AsyncInferenceServicer):
    prefix = "policy_server"
    logger = get_logger(prefix)

    def __init__(self, config: PolicyServerConfig):
        # 保存配置 & 初始化状态
        self.config = config
        self.shutdown_event = threading.Event()

        # 用来统计 FPS（接收 obs 的频率）
        self.fps_tracker = FPSTracker(target_fps=config.fps)

        # obs 队列：最大只保留 1 个观测（只对最新观测推理）
        self.observation_queue = Queue(maxsize=1)

        # 记录哪些 timestep 已经预测过动作，防止重复计算
        self._predicted_timesteps_lock = threading.Lock()
        self._predicted_timesteps = set()

        self.last_processed_obs = None

        # 下面这些属性在接收 policy 指令时设置
        self.device = None
        self.policy_type = None
        self.lerobot_features = None
        self.actions_per_chunk = None
        self.policy = None
        self.preprocessor: PolicyProcessorPipeline[dict[str, Any], dict[str, Any]] | None = None
        self.postprocessor: PolicyProcessorPipeline[PolicyAction, PolicyAction] | None = None

    @property
    def running(self):
        # server 是否处于工作状态
        return not self.shutdown_event.is_set()

    @property
    def policy_image_features(self):
        # 从 policy config 里拿图像特征定义
        return self.policy.config.image_features

    def _reset_server(self) -> None:
        """当有新 client 连接时，重置 server 内部状态。"""
        # 先标记 shutdown，避免旧状态继续跑
        self.shutdown_event.set()
        # 重建 obs 队列，仅保留最新 obs 的逻辑由 maxsize=1 保证
        self.observation_queue = Queue(maxsize=1)

        # 清空已经预测过的 timestep 集合
        with self._predicted_timesteps_lock:
            self._predicted_timesteps = set()

    def Ready(self, request, context):  # noqa: N802
        """机器人 client 连接握手接口。"""
        client_id = context.peer()
        self.logger.info(f"Client {client_id} connected and ready")
        # 有新 client 连接，重置内部状态
        self._reset_server()
        self.shutdown_event.clear()

        return services_pb2.Empty()

    def SendPolicyInstructions(self, request, context):  # noqa: N802
        """从 robot client 接收 policy 配置，比如：
        - policy 类型（diffusion, smolvla, etc.）
        - 预训练模型路径
        - actions_per_chunk
        - device
        """
        if not self.running:
            self.logger.warning("Server is not running. Ignoring policy instructions.")
            return services_pb2.Empty()

        client_id = context.peer()

        # 反序列化 RemotePolicyConfig
        policy_specs = pickle.loads(request.data)  # nosec

        if not isinstance(policy_specs, RemotePolicyConfig):
            raise TypeError(f"Policy specs must be a RemotePolicyConfig. Got {type(policy_specs)}")

        if policy_specs.policy_type not in SUPPORTED_POLICIES:
            raise ValueError(
                f"Policy type {policy_specs.policy_type} not supported. "
                f"Supported policies: {SUPPORTED_POLICIES}"
            )

        self.logger.info(
            f"Receiving policy instructions from {client_id} | "
            f"Policy type: {policy_specs.policy_type} | "
            f"Pretrained name or path: {policy_specs.pretrained_name_or_path} | "
            f"Actions per chunk: {policy_specs.actions_per_chunk} | "
            f"Device: {policy_specs.device}"
        )

        # 记录来自 client 的配置信息
        self.device = policy_specs.device
        self.policy_type = policy_specs.policy_type
        self.lerobot_features = policy_specs.lerobot_features
        self.actions_per_chunk = policy_specs.actions_per_chunk

        # 通过工厂方法根据类型拿到 policy class（diffusion/smolvla/...）
        policy_class = get_policy_class(self.policy_type)

        # 1. 加载 policy 权重
        start = time.perf_counter()
        self.policy = policy_class.from_pretrained(policy_specs.pretrained_name_or_path)
        # 2. 把模型搬到指定 device（cuda/cpu/mps）
        self.policy.to(self.device)

        # 3. 构建 preprocessor/postprocessor：负责
        #    - 把 raw observation 转成模型输入
        #    - 把模型输出的 action 反归一化、搬回 cpu 等
        device_override = {"device": self.device}
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            self.policy.config,
            pretrained_path=policy_specs.pretrained_name_or_path,
            preprocessor_overrides={
                "device_processor": device_override,
                "rename_observations_processor": {"rename_map": policy_specs.rename_map},
            },
            postprocessor_overrides={"device_processor": device_override},
        )

        end = time.perf_counter()

        self.logger.info(f"Time taken to put policy on {self.device}: {end - start:.4f} seconds")

        return services_pb2.Empty()

    def SendObservations(self, request_iterator, context):  # noqa: N802
        """从 robot client 接收一批 observation（通过 gRPC 流 + chunk 发送）。"""
        client_id = context.peer()
        self.logger.debug(f"Receiving observations from {client_id}")

        receive_time = time.time()
        start_deserialize = time.perf_counter()

        # 把多个 gRPC chunk 拼成 bytes
        received_bytes = receive_bytes_in_chunks(
            request_iterator, None, self.shutdown_event, self.logger
        )
        # 反序列化成 TimedObservation（包含 timestamp/timestep/obs 字典）
        timed_observation = pickle.loads(received_bytes)  # nosec
        deserialize_time = time.perf_counter() - start_deserialize

        self.logger.debug(f"Received observation #{timed_observation.get_timestep()}")

        obs_timestep = timed_observation.get_timestep()
        obs_timestamp = timed_observation.get_timestamp()

        # 计算 obs 接收 FPS 和单向延迟
        fps_metrics = self.fps_tracker.calculate_fps_metrics(obs_timestamp)

        self.logger.debug(
            f"Received observation #{obs_timestep} | "
            f"Avg FPS: {fps_metrics['avg_fps']:.2f} | "
            f"Target: {fps_metrics['target_fps']:.2f} | "
            f"One-way latency: {(receive_time - obs_timestamp) * 1000:.2f}ms"
        )

        self.logger.debug(
            f"Server timestamp: {receive_time:.6f} | "
            f"Client timestamp: {obs_timestamp:.6f} | "
            f"Deserialization time: {deserialize_time:.6f}s"
        )

        # 根据 must_go + 去重逻辑决定要不要排队
        if not self._enqueue_observation(timed_observation):
            self.logger.debug(f"Observation #{obs_timestep} has been filtered out")

        return services_pb2.Empty()

    def GetActions(self, request, context):  # noqa: N802
        """robot client 来拉取一整段 action chunk 的接口。"""
        client_id = context.peer()
        self.logger.debug(f"Client {client_id} connected for action streaming")

        try:
            getactions_starts = time.perf_counter()
            # 从 obs 队列拿最新观测（带 timeout）
            obs = self.observation_queue.get(timeout=self.config.obs_queue_timeout)
            self.logger.info(
                f"Running inference for observation #{obs.get_timestep()} (must_go: {obs.must_go})"
            )

            # 记录这个 timestep 已经被预测过
            with self._predicted_timesteps_lock:
                self._predicted_timesteps.add(obs.get_timestep())

            # 1. 模型推理，拿到 action chunk
            start_time = time.perf_counter()
            action_chunk = self._predict_action_chunk(obs)
            inference_time = time.perf_counter() - start_time

            # 2. 序列化为 bytes 返回
            start_time = time.perf_counter()
            actions_bytes = pickle.dumps(action_chunk)  # nosec
            serialize_time = time.perf_counter() - start_time

            actions = services_pb2.Actions(data=actions_bytes)

            self.logger.info(
                f"Action chunk #{obs.get_timestep()} generated | "
                f"Total time: {(inference_time + serialize_time) * 1000:.2f}ms"
            )

            self.logger.debug(
                f"Action chunk #{obs.get_timestep()} generated | "
                f"Inference time: {inference_time:.2f}s |"
                f"Serialize time: {serialize_time:.2f}s |"
                f"Total time: {inference_time + serialize_time:.2f}s"
            )

            # 控制推理频率：保证单次调用不超过 config.inference_latency
            time.sleep(
                max(0, self.config.inference_latency - max(0, time.perf_counter() - getactions_starts))
            )

            return actions

        except Empty:
            # 在 obs_queue_timeout 内没拿到 obs，返回空
            return services_pb2.Empty()

        except Exception as e:
            # 推理过程中出异常（比如你之前遇到的 stack expects non-empty TensorList）
            self.logger.error(f"Error in StreamActions: {e}")
            return services_pb2.Empty()

    def _predict_action_chunk(self, observation_t: TimedObservation) -> list[TimedAction]:
        """根据单个 TimedObservation 预测一段动作序列（action chunk）。"""

        # 1. raw obs -> LeRobot 统一格式（填入 lerobot_features）
        start_prepare = time.perf_counter()
        observation: Observation = raw_observation_to_observation(
            observation_t.get_observation(),
            self.lerobot_features,
            self.policy_image_features,
        )
        prepare_time = time.perf_counter() - start_prepare

        # 2. preprocessor：tokenize / normalize / batch / 放到 device
        start_preprocess = time.perf_counter()
        observation = self.preprocessor(observation)
        self.last_processed_obs: TimedObservation = observation_t
        preprocessing_time = time.perf_counter() - start_preprocess

        # 3. 调 policy，拿一个 (B, chunk_size, action_dim) 的 action tensor
        start_inference = time.perf_counter()
        action_tensor = self._get_action_chunk(observation)
        inference_time = time.perf_counter() - start_inference
        self.logger.info(
            f"Preprocessing and inference took {inference_time:.4f}s, action shape: {action_tensor.shape}"
        )

        # 4. postprocessor：对 chunk 里每个 action 做反归一化 / device 移动
        start_postprocess = time.perf_counter()
        _, chunk_size, _ = action_tensor.shape

        processed_actions = []
        for i in range(chunk_size):
            # 取出第 i 个 action（B, action_dim）
            single_action = action_tensor[:, i, :]
            processed_action = self.postprocessor(single_action)
            processed_actions.append(processed_action)

        # 堆成 (B, chunk_size, action_dim)，然后 squeeze batch 维度
        action_tensor = torch.stack(processed_actions, dim=1).squeeze(0)
        self.logger.debug(f"Postprocessed action shape: {action_tensor.shape}")

        # 5. 把每个 action 包装成 TimedAction，附上时间戳和 timestep
        action_chunk = self._time_action_chunk(
            observation_t.get_timestamp(), list(action_tensor), observation_t.get_timestep()
        )
        postprocess_stops = time.perf_counter()
        postprocessing_time = postprocess_stops - start_postprocess

        self.logger.info(
            f"Observation {observation_t.get_timestep()} | "
            f"Total time: {1000 * (postprocess_stops - start_prepare):.2f}ms"
        )

        self.logger.debug(
            f"Observation {observation_t.get_timestep()} | "
            f"Prepare time: {1000 * prepare_time:.2f}ms | "
            f"Preprocessing time: {1000 * preprocessing_time:.2f}ms | "
            f"Inference time: {1000 * inference_time:.2f}ms | "
            f"Postprocessing time: {1000 * postprocessing_time:.2f}ms | "
            f"Total time: {1000 * (postprocess_stops - start_prepare):.2f}ms"
        )

        return action_chunk
