import keyboard
import threading


class RewardManager:
    def __init__(self):
        self.reward = 0
        self.running = True
        self.start_listener()

    def start_listener(self):
        keyboard.on_press_key("0", self.increase_reward)
        keyboard.on_press_key("9", self.decrease_reward)
        keyboard.on_press_key("8", self.large_increase_reward)
        listener_thread = threading.Thread(target=self.listen)
        listener_thread.daemon = True
        listener_thread.start()

    def listen(self):
        while self.running:
            keyboard.read_event()

    def increase_reward(self, e):
        self.reward += 1
        print(f"Reward increased by 1. Current reward: {self.reward}")

    def decrease_reward(self, e):
        self.reward -= 1
        print(f"Reward decreased by 1. Current reward: {self.reward}")

    def large_increase_reward(self, e):
        self.reward += 10
        print(f"Reward increased by 10. Current reward: {self.reward}")


def main_task(reward_manager):
    while True:
        print(reward_manager.reward)
        # ここにロボットの動作や他の処理を記述
        # 例:
        # ロボットの現在の状態を取得
        # ロボットにコマンドを送信
        # 必要に応じてreward_manager.rewardを使用
        pass


if __name__ == "__main__":
    reward_manager = RewardManager()
    main_task(reward_manager)
