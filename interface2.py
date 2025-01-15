# initメソッド
# 初期化メソッド
# リセットメソッド
# ブロックメソッド
# ステップメソッド(入力：行動)
# 状態空間取得メソッド
# 報酬メソッド
# 行動空間メソッド

class Side(enum.Enum):
    LEFT = 1
    RIGHT = 2
    OTHER = 3

@dataclasses.dataclass
class EnvConfig:
    max_delta_m: float = 0.04  # max displacement for the arm per time step
    control_rate_hz: float = 20
    with_camera: bool = True
    debug_cam_vis: bool = False
    use_real: bool = True
    robot_type: RobotType = RobotType.XARM
    enable_z: bool = True
    task: Task = Task.PICKPLACE


JOINT = 4 # AL5Dのジョイントの数
class Al5dPickPlaceEnv:
    def __init__(self, cfg: EnvConfig):
        # if cfg.task == Task.SWEEP:
        #     assert cfg.enable_z, "z control must be enabled for sweeping"

        # if cfg.task == Task.SWEEP:
        #     raise NotImplementedError()  # TODO

        self.cfg = cfg
        self._arm: Union[XArmSimpleRobotWrapper, UR5SimpleRobotWrapper]
        if cfg.use_real:
            if self.cfg.robot_type == RobotType.XARM:
                self._arm = XArmSimpleRobotWrapper()
            elif self.cfg.robot_type == RobotType.UR5:
                self._arm = UR5SimpleRobotWrapper()
            else:
                raise NotImplementedError(f"arm: {self.cfg.robot_type} not implemented")
        else:
            self._arm = None  # type: ignore
            if not self.cfg.debug_cam_vis:
                return
        self.rate = Rate(cfg.control_rate_hz)

        if self.cfg.with_camera:
            ctx = rs.context()
            devices = ctx.query_devices()
            for dev in devices:
                dev.hardware_reset()
            time.sleep(2)
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            # if self.cfg.debug_cam_vis:
            if self.cfg.debug_cam_vis:
                while True:
                    image = self.get_frames()[0][:, :, ::-1]
                    depth = np.repeat(self.get_frames()[1], 3, -1)
                    cv2.imshow("img", np.concatenate([image, depth], 1))

                    import matplotlib.pyplot as plt
                    fig, ax = plt.subplots()
                    ax.hist(depth.ravel(), bins=100)
                    plt.show()

                    image = cv2.applyColorMap(255 - depth, cv2.COLORMAP_VIRIDIS)
                    cv2.imshow("img", depth)

                    cv2.waitKey(1)

    def get_frames(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.cfg.with_camera:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_frame = frames.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)

            if self.cfg.robot_type == RobotType.UR5:
                depth_image = depth_image[135:-100, 120:-170]
                # depth_image = depth_image[120:-80, 150:-190]
                color_image = color_image[20:-30, 40:-55]
                farthest = 0.180
            else:
                farthest = 0.120

            if self.cfg.debug_cam_vis:
                img_size = (480, 480)
            else:
                img_size = (64, 64)
            image = cv2.resize(color_image, img_size)[:, :, ::-1]
            depth = cv2.resize(depth_image, img_size)[:, :, None]
            # Map depth to used range for our robot setup.
            depth = depth.astype(np.float32) / 255
            nearest = 0.050
            depth = (depth - nearest) / (farthest - nearest)
            depth = (255 * np.clip(depth, 0, 1)).astype(np.uint8)

        else:
            image = np.zeros((64, 64, 3))
            depth = np.zeros((64, 64, 1))
        return image, depth

    @property
    def obs_space(self) -> Dict[str, embodied.Space]:
        return {
            "image": embodied.Space(np.uint8, (64, 64, 3)),
            "depth": embodied.Space(np.uint8, (64, 64, 1)),
            "cartesian_position": embodied.Space(np.float32, (6,)),
            "joint_positions": embodied.Space(
                np.float32, (JOINT,)
            ),
            "gripper_pos": embodied.Space(np.float32, (1,)),
            "gripper_side": embodied.Space(np.float32, (3,)),
            "grasped_side": embodied.Space(np.float32, (3,)),
            "reward": embodied.Space(np.float32),
            "is_first": embodied.Space(bool),
            "is_last": embodied.Space(bool),
            "is_terminal": embodied.Space(bool),
        }
    @property
    def act_space(self) -> Dict[str, embodied.Space]:
        return {
            "action": embodied.Space(np.int64, (), 0, 6),
        }
    
    # -------------------DayDreamer: pick and place---------------------------------------------
    def random_xy_grid(self, side: Side) -> Tuple[float, float]:
        if side == Side.LEFT:
            x = np.random.uniform(
                self._arm.LEFT_SAFE_XY_MIN[0], self._arm.LEFT_SAFE_XY_MAX[0]
            )
            y = np.random.uniform(
                self._arm.LEFT_SAFE_XY_MIN[1], self._arm.LEFT_SAFE_XY_MAX[1]
            )
        elif side == Side.RIGHT:
            x = np.random.uniform(
                self._arm.RIGHT_SAFE_XY_MIN[0], self._arm.RIGHT_SAFE_XY_MAX[0]
            )
            y = np.random.uniform(
                self._arm.RIGHT_SAFE_XY_MIN[1], self._arm.RIGHT_SAFE_XY_MAX[1]
            )
        else:
            raise NotImplementedError(f"Got side: {side} ")

        x = np.round(x / self.cfg.max_delta_m) * self.cfg.max_delta_m
        y = np.round(y / self.cfg.max_delta_m) * self.cfg.max_delta_m

        if side == Side.LEFT:
            x = np.clip(x, self._arm.LEFT_SAFE_XY_MIN[0], self._arm.LEFT_SAFE_XY_MAX[0])
            y = np.clip(y, self._arm.LEFT_SAFE_XY_MIN[1], self._arm.LEFT_SAFE_XY_MAX[1])
        elif side == Side.RIGHT:
            x = np.clip(
                x, self._arm.RIGHT_SAFE_XY_MIN[0], self._arm.RIGHT_SAFE_XY_MAX[0]
            )
            y = np.clip(
                y, self._arm.RIGHT_SAFE_XY_MIN[1], self._arm.RIGHT_SAFE_XY_MAX[1]
            )
        else:
            raise NotImplementedError(f"Got side: {side} ")
        return x, y

    def arm_side(self, margin: float = -0.002) -> Side:
        _, _, cart_pos = self._arm.get_robot_state()

        pos = np.array(cart_pos)[:2]
        if (np.array(self._arm.RIGHT_XY_MIN) + margin <= pos).all() and (
            np.array(self._arm.RIGHT_XY_MAX) - margin >= pos
        ).all():
            return Side.RIGHT
        elif (np.array(self._arm.LEFT_XY_MIN) + margin <= pos).all() and (
            np.array(self._arm.LEFT_XY_MAX) - margin >= pos
        ).all():
            return Side.LEFT
        else:
            print("ARM NOT ON EITHER SIDE")
            print(cart_pos[:2])
            self._print_debug_info()
            self._reset()
            return self.arm_side()

    def current_bounds(self) -> Tuple[np.ndarray, np.ndarray, float]:

        side = self.arm_side()
        if side == Side.LEFT:
            if self.is_hover():
                return (
                    np.array(self._arm.LEFT_SAFE_XY_MIN),
                    np.array(self._arm.LEFT_SAFE_XY_MAX),
                    self._arm.Z_HOVER,
                )
            else:
                return (
                    np.array(self._arm.LEFT_XY_MIN),
                    np.array(self._arm.LEFT_XY_MAX),
                    self._arm.Z_TABLE,
                )
        elif side == Side.RIGHT:
            if self.is_hover():
                return (
                    np.array(self._arm.RIGHT_SAFE_XY_MIN),
                    np.array(self._arm.RIGHT_SAFE_XY_MAX),
                    self._arm.Z_HOVER,
                )
            else:
                return (
                    np.array(self._arm.RIGHT_XY_MIN),
                    np.array(self._arm.RIGHT_XY_MAX),
                    self._arm.Z_TABLE,
                )
        else:
            raise NotImplementedError

    def is_hover(self) -> bool:
        _, _, cart_pos = self._arm.get_robot_state()
        return cart_pos[2] > (self._arm.Z_HOVER + self._arm.Z_TABLE) / 2

    def compute_arm_position(self, control_action: np.ndarray) -> np.ndarray:
        """Convert control action to TCP homogeneous transform.

        Args:
            env_config (EnvConfig): The environment configuration.
            control_action (np.ndarray, shape=self.control_shape()): control_action
            (should be values between -1 and 1, following the dm_control convention)
            curr_pose (np.ndarray, shape=(6, )): the current robot pose

        Returns:
            np.ndarray, shape=(6, ): The target pose.
        """
        control_action = np.clip(control_action, -1, 1) * self.cfg.max_delta_m
        assert control_action.shape == (2,), control_action

        _, _, cart_pos = self._arm.get_robot_state()
        target_pose = np.array(cart_pos)
        target_pose[:2] = target_pose[:2] + control_action

        xy_min, xy_max, z_loc = self.current_bounds()

        target_pose[:2] = (
            np.round(target_pose[:2] / self.cfg.max_delta_m)
        ) * self.cfg.max_delta_m

        desired_pose = np.copy(target_pose)
        target_pose[:2] = np.clip(target_pose[:2], xy_min, xy_max)

        if self.grasped_object and self.is_hover():
            side = self.arm_side()
            # cross the middle if holding object
            if (
                side == Side.LEFT
                and desired_pose[self._arm.AXIS] + 0.01 < target_pose[self._arm.AXIS]
            ):
                target_pose[:2] = np.clip(
                    target_pose[:2],
                    self._arm.RIGHT_SAFE_XY_MIN,
                    self._arm.RIGHT_SAFE_XY_MAX,
                )
            if (
                side == Side.RIGHT
                and desired_pose[self._arm.AXIS] - 0.01 > target_pose[self._arm.AXIS]
            ):
                target_pose[:2] = np.clip(
                    target_pose[:2],
                    self._arm.LEFT_SAFE_XY_MIN,
                    self._arm.LEFT_SAFE_XY_MAX,
                )

        target_pose[2] = z_loc
        if control_action[0] == 0:
            target_pose[0] = cart_pos[0]
        if control_action[1] == 0:
            target_pose[1] = cart_pos[1]
        return target_pose

    def step(self, action: Dict[str, Any]) -> Dict[str, Any]:
        if action["reset"]:
            if action.get("manual_resume", False):
                return self.get_obs(robot_in_safe_state=True, is_first=True)
            else:
                return self._reset()


        if action["action"] < 4:
            pos_delta = ((-1, 0), (1, 0), (0, -1), (0, 1))[action["action"]]
            xyzrpy = self.compute_arm_position(np.array(pos_delta))
            self._arm.set_position(xyzrpy[0], xyzrpy[1])

        elif action["action"] == 4:  # close
            if self._arm._gripper_state_open:
                self._arm.close_gripper()
            else:
                self._arm.open_gripper()

        elif action["action"] == 5:  # close
            arm_side: Side = self.arm_side()
            _, _, cart_pos = self._arm.get_robot_state()
            is_hover = cart_pos[2] > (self._arm.Z_HOVER + self._arm.Z_TABLE) / 2
            if is_hover:
                self._arm.set_z(self._arm.Z_TABLE)
            elif self.grasped_object:
                _, _, cart_pos = self._arm.get_robot_state()
                if arm_side == Side.LEFT:
                    cart_pos[:2] = np.clip(
                        cart_pos[:2],
                        self._arm.LEFT_SAFE_XY_MIN,
                        self._arm.LEFT_SAFE_XY_MAX,
                    )
                else:
                    cart_pos[:2] = np.clip(
                        cart_pos[:2],
                        self._arm.RIGHT_SAFE_XY_MIN,
                        self._arm.RIGHT_SAFE_XY_MAX,
                    )
                if self.cfg.enable_z:
                    self._arm.set_position(cart_pos[0], cart_pos[1], self._arm.Z_TABLE)
                    self._arm.set_position(cart_pos[0], cart_pos[1], self._arm.Z_HOVER)
            else:
                # no object so no op
                pass
        else:
            raise NotImplementedError(action)

        self.rate.sleep()

        obs = self.get_obs(
            robot_in_safe_state=True, is_first=False
        )  # TODO: check safe state
        if obs["reward"] != 0:
            obs = self.get_obs(
                robot_in_safe_state=True, is_first=False, reward=obs["reward"]
            )

        if action.get("manual_pause", False):
            self._arm.open_gripper()
        return obs

    def _reset(self) -> Dict[str, Any]:
        if self.grasped_object:
            # move to random pos in bin where object was grasped
            x, y = self.random_xy_grid(self._ball_side)
            self._arm.set_position(x, y, self._arm.Z_HOVER)

        self._arm.open_gripper()
        self.grasped_bin = Side.OTHER
        self.grasped_object = False

        if self._ball_side == Side.LEFT:
            xyz_min, xyz_max = self._arm.LEFT_XY_MIN, self._arm.LEFT_XY_MAX
        elif self._ball_side == Side.RIGHT:
            xyz_min, xyz_max = self._arm.RIGHT_XY_MIN, self._arm.RIGHT_XY_MAX
        else:
            raise NotImplementedError(f"ball side={self._ball_side}")

        if self.cfg.robot_type == RobotType.UR5:
            # get ball out of corners
            for corner_x, corner_y in ([1, 0], [0, 0], [0, 1], [1, 1]):
                if corner_x == 0:
                    x = xyz_min[0]
                else:
                    x = xyz_max[0]
                if corner_y == 0:
                    y = xyz_min[1]
                else:
                    y = xyz_max[1]
                time.sleep(2)
                self._arm.set_position(x, y, self._arm.Z_TABLE, acc=0.6)

        self._arm.open_gripper()
        x, y = self.random_xy_grid(self._ball_side)
        self._arm.set_position(x, y, self._arm.Z_TABLE, acc=0.6)
        time.sleep(1)  # wait for scene to settle after reset

        obs = self.get_obs(robot_in_safe_state=True, is_first=True)
        return obs

    def get_reward(self, curr_obs: Dict[str, Any]) -> float:
        grasped_old = self.grasped_object
        grasped_new = check_grasped_object_ur(curr_obs["gripper_pos"])
        self.grasped_object = grasped_new

        arm_side: Side = self.arm_side()
        if grasped_old and grasped_new and arm_side != self.grasped_bin:
            # let go of ball
            self._arm.open_gripper()
            self._ball_side = arm_side
            self.grasped_object = False

            # move arm down
            self._arm.set_z(self._arm.Z_TABLE)

            # move arm to random location on success
            x, y = self.random_xy_grid(self._ball_side)
            self._arm.set_position(x, y, self._arm.Z_TABLE)
            return 10

        if not grasped_old and not grasped_new:  # not holding it
            return 0

        if not grasped_old and grasped_new:  # grasped it
            self.grasped_bin = arm_side
            assert self.grasped_bin != Side.OTHER
            _, _, cart_pos = self._arm.get_robot_state()
            if arm_side == Side.LEFT:
                cart_pos[:2] = np.clip(
                    cart_pos[:2], self._arm.LEFT_SAFE_XY_MIN, self._arm.LEFT_SAFE_XY_MAX
                )
            else:
                cart_pos[:2] = np.clip(
                    cart_pos[:2],
                    self._arm.RIGHT_SAFE_XY_MIN,
                    self._arm.RIGHT_SAFE_XY_MAX,
                )
            if not self.cfg.enable_z:
                self._arm.set_position(cart_pos[0], cart_pos[1], self._arm.Z_TABLE)
                self._arm.set_position(cart_pos[0], cart_pos[1], self._arm.Z_HOVER)
            return 1

        if grasped_old and not grasped_new:  # dropped it
            assert arm_side == self.grasped_bin
            rew = -1
            self._arm.set_z(self._arm.Z_TABLE)
            self.grasped_bin = Side.OTHER
            return rew

        if grasped_old and grasped_new:  # holding it
            return 0

        raise NotImplementedError

    def main(env_config: EnvConfig) -> None:
        print(env_config)
        # select an env
        # env = KeyRewardEnv(env_config)
        # env = CornerRewardEnv(env_config)
        env = PickPlace(cfg=env_config)

        # select an agent
        # agent = UnsafeTestAgent(env_config)
        agent = SpaceMouseAgent(env_config)
        print(f"==> Running the environment with {agent}")

        obs = env._reset()
        count = 0
        while True:
            count += 1
            if count % 100 == 0:
                env._reset()
            try:
                action = agent.act(obs)  # type: ignore
            except IndexError:
                break
            t = time.time()
            action["reset"] = False
            start = time.time()
            obs = env.step(action)
            print(f"action={action}, {time.time() - start}")
            reward = obs["reward"]
            if np.abs(reward) > 0.5:
                print(f"Step time : {time.time() - t}, reward: {reward}")
            if obs["is_last"]:
                env.step({"reset": True})
        print("==> Finishing to run the environment")


if __name__ == "__main__":
    main(dcargs.parse(EnvConfig))