# group32-work-repo

世界モデル 2024 最終課題グループ 32 の作業リポジトリです。

```python
import numpy as np

# ロボットアームの長さ
L1 = 150.0  # mm
L2 = 185.0  # mm
L3 = 140.0  # mm
L4 = 100.0  # mm（エンドエフェクタの長さ）

def inverse_kinematics(x, y, z):
    # θ1の計算
    θ1 = np.arctan2(y, x)

    # 平面上の距離
    r = np.sqrt(x**2 + y**2)

    # Z方向の位置
    z_offset = z - L1

    # θ2とθ3の計算
    d = np.sqrt(r**2 + z_offset**2)
    if d > (L2 + L3):
        raise ValueError("目標位置が到達不可能です。")

    # θ3の計算
    θ3 = np.arccos((L2**2 + L3**2 - d**2) / (2 * L2 * L3))

    # θ2の計算
    θ2 = np.arctan2(z_offset, r) - np.arctan2(L3 * np.sin(θ3), L2 + L3 * np.cos(θ3))

    # θ4の計算（ここでは固定とする）
    θ4 = 0  # 必要に応じて調整してください

    # ラジアンから度に変換
    θ1 = np.degrees(θ1)
    θ2 = np.degrees(θ2)
    θ3 = np.degrees(θ3)
    θ4 = np.degrees(θ4)

    return θ1, θ2, θ3, θ4

# 例: エンドエフェクタの座標を入力
x = int(input("x座標(mm):"))
y = int(input("y座標(mm):"))
z = int(input("z座標(mm):"))

try:
    angles = inverse_kinematics(x, y, z)
    print(f"θ1: {angles[0]:.2f}°, θ2: {angles[1]:.2f}°, θ3: {angles[2]:.2f}°, θ4: {angles[3]:.2f}°")
except ValueError as e:
    print(e)
```

データを入れて、角度を求めた結果がこんな感じ。
①z 軸だけ数値を変化させた場合（z=0,100,200)
(100,100,000)θ1: 45.00°, θ2: -79.05°, θ3: 77.37°, θ4: 0.00°
(100,100,100)θ1: 45.00°, θ2: -41.94°, θ3: 52.79°, θ4: 0.00°
(100,100,200)θ1: 45.00°, θ2: - 2.99°, θ3: 52.79°, θ4: 0.00°
エンドエフェクタの高さが変わるだけなので θ1 は不変。
徐々に θ2,3 の角度が 0 に近づいているので、エンドエフェクタが上がっていると考察
②x 軸だけ数値を変化させた場合（x=0,100,200)
(000,100,100)θ1: 90.00°, θ2: -42.45°, θ3: 37.08°, θ4: 0.00°
(100,100,100)θ1: 45.00°, θ2: -41.94°, θ3: 52.79°, θ4: 0.00°
(200,100,100)θ1: 26.57°, θ2: -49.19°, θ3: 88.53°, θ4: 0.00°
エンドエフェクタの高さは不変だが x 軸方向に遠くなるので、θ1 の角度は徐々に狭くなる。
また、アームが伸びる方向なので、θ2 の角度は狭くなり、エンドエフェクタを下に持って行くために θ3 の角度は大きくなる。
③y 軸だけ数値を変化させた場合（y=0,100,200）
(100,000,100)θ1: 0.00°, θ2: -42.45°, θ3: 37.08°, θ4: 0.00°
(100,100,100)θ1: 45.00°, θ2: -41.94°, θ3: 52.79°, θ4: 0.00°
(100,200,100)θ1: 63.43°, θ2: -49.19°, θ3: 88.53°, θ4: 0.00°
エンドエフェクタの高さは不変だが、y 軸方向に遠くなるので、θ1 の角度は徐々に広くなる。
また、アームは ② と同じように伸びる方向なので、② と同様の変化となる。

![](Robot_arm_4.jpg)

有効な範囲を調べるプログラム

```
    # AL5Dクラスのインスタンスを作成
    robot = AL5D()

    # 関節角度の範囲と刻み幅を定義
    MIN_ANGLE = robot.MIN_ANGLE  # -90度
    MAX_ANGLE = robot.MAX_ANGLE  # +90度
    ANGLE_STEP = 10  # 角度の刻み幅（度）

    # 各関節の角度範囲を生成
    theta1_values = np.arange(MIN_ANGLE, MAX_ANGLE + 1, ANGLE_STEP)
    theta2_values = np.arange(MIN_ANGLE, MAX_ANGLE + 1, ANGLE_STEP)
    theta3_values = np.arange(MIN_ANGLE, MAX_ANGLE + 1, ANGLE_STEP)

    # 有効な座標を格納するリスト
    valid_positions = []

    # 総当たりで各角度の組み合わせを計算
    for theta1_deg in theta1_values:
        for theta2_deg in theta2_values:
            for theta3_deg in theta3_values:
                # 角度をラジアンに変換
                theta1_rad = math.radians(theta1_deg)
                theta2_rad = math.radians(theta2_deg)
                theta3_rad = math.radians(theta3_deg)
                theta4_rad = robot.THETA4  # THETA4は定数

                # 順運動学を計算
                try:
                    position = robot.forward_kinematics(
                        theta1_rad, theta2_rad, theta3_rad, theta4_rad
                    )
                    x, y, z = position["エンドエフェクタ座標[x,y,z]"]

                    # 座標をリストに追加
                    valid_positions.append((x, y, z))
                except Exception as e:
                    # 計算エラーをキャッチ（エラーが発生した場合は無視）
                    print(
                        f"エラー: θ1={theta1_deg}, θ2={theta2_deg}, θ3={theta3_deg} - {e}"
                    )
                    continue

    # 結果の表示
    print(f"有効な座標の総数: {len(valid_positions)}")

    # オプション: 座標をファイルに保存
    with open("valid_positions.txt", "w", encoding="utf-8") as f:
        for pos in valid_positions:
            f.write(f"{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}\n")

    print("有効な座標を 'valid_positions.txt' ファイルに保存しました。")

```
