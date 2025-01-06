from gym import spaces
import time
import pybullet as p
import pybullet_data
class AL5D_ArmEnv(gym.Env):
  def __init__(self):
    super(AL5D_ArmEnv,self).__init__()
    self.physics_client = p.connect(p.DIRECT)#物理サーバーに接続
    p.setAdditionalSearchPath(pybullet_data.getDataPath())#データパスを設定
    #URDFファイルの読み込み
    self.AL5D_id = p.loadURDF("/content/drive/MyDrive/lastsubmission_worldmodel/al5d_robot.urdf")
    self.num_joints = p.getNumJoints(self.AL5D_id)
    #ブロック生成
    self.block_id = None
    self._generate_block()

    #状態空間(関節位置、ブロック座標)
    self.ovservation_space = spaces.Box(
        low=-np.inf, high=np.inf,shape=(self.num_joints+3),dtype=np.float32
    )
    #行動空間
    self.action_space = spaces.Box(
        low=-1.570796325,high=1.570796325,shape=(self.num_joints,),dtype=np.float32
    )
    #最大ステップ
    self.max_steps=200
    self.current_step=0

  def _generate_block(self):
    #ランダムな位置にボールを生成
    if block_id is not None:
      p.removeBody(self.block_id)
    block_position = [np.random.uniform(1,2),np.random.uniform(1,2),0]
    self.block_id = p.loadURDF("/content/drive/MyDrive/lastsubmission_worldmodel/al5d_block.urdf",basePosition=block_position)
  def reset(self):
    #シミュレーションを完全にリセット
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())#データパスを再設定

    #URDFの再読み込み
    self.AL5D_id = p.loadURDF("/content/drive/MyDrive/lastsubmission_worldmodel/al5d_robot.urdf")
    self.num_joints = p.getNumJoints(self.AL5D_id)

    #ブロックの再生成
    self._generate_block()
    self.current_step=0
    return self._get_ovservation()

  def step(self,action):
    #次の状態を計算
    #各関節の角度を更新
    #各関節の角度を更新
    for joint_idx,joint_target in enumerate(action):
      p.setJointMotorControl2(
          bodyIndex=self.robot_id,
          jointIndex=joint_idx,
          controlMode=p.POSITION_CONTROL,
          targetPosition=joint_target
      )
    #シミュレーションを進める
    p.stepSimulation()
    time.sleep(1.0/240.0)
    #状態を取得
    state = self._get_ovservation()
    #報酬の計算
    reward,done = self._calculate_reward()
    #ステップ数の増加
    self.current_step +=1
    #終了条件の判定
    if self.current_step>=self.max_steps or done:
      done = True

    info = {}

    return state, reward, done, info
  def render(self, mode="rgb_array"):
    """環境の可視化（フレームを生成）"""
    if mode != "rgb_array":
        raise NotImplementedError(f"Render mode {mode} is not supported")

    # カメラの設定
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0, 0, 0.5],  # カメラのターゲット位置（ロボット中心付近を指定）
        distance=2,  # カメラの距離
        yaw=45,  # カメラの角度
        pitch=-30,  # カメラの傾き
        roll=0,
        upAxisIndex=2,
    )
    proj_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=1.0, nearVal=0.1, farVal=100.0
    )

    # カメラ画像を取得
    img_arr = p.getCameraImage(
        width=640, height=480, viewMatrix=view_matrix, projectionMatrix=proj_matrix
    )
    # RGB画像データを返す
    rgb_array = np.array(img_arr[2])  # 画像データが3番目の要素として格納されている
    return rgb_array

  def close(self):
    p.disconnect(self.physics_client)
  def _get_observation(self):
    #状態を取得
    joint_states = [p.getJointState(self.AL5D_id, i)[0] for i in range(p.getNumJoints(self.AL5D_id))]
    block_position ,_= p.getBasePositionAndOrientation(self.block_id)
    return np.array(joint_states+list(block_position),dtype=np.float32)

  def _calculate_reward(self):
    #報酬と終了条件

    #報酬の計算
    if block_position > 5:
      reward = 10.0
      done = True

    return  reward,done
