import cv2
import numpy as np

class Camera:
    def __init__(self):
        """
        内蔵カメラを初期化します。通常、デバイスIDは0です。
        カメラが正常に開けなかった場合は例外を投げます。
        """
        # 内蔵カメラを初期化（デバイスID 0）
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("内蔵カメラを開くことができませんでした。")
        
        # カメラの解像度を設定（必要に応じて変更可能）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    def get_frame(self):
        """
        カメラからフレームを取得し、カラー画像を返します。
        フレーム取得に失敗した場合は例外を投げます。
        
        Returns:
            color_image (np.ndarray): 取得したカラー画像
        """
        try:
            ret, color_image = self.cap.read()
            if not ret:
                raise RuntimeError("フレームを取得できませんでした。")
            return color_image
        except Exception as e:
            print(f"フレーム取得中にエラーが発生しました: {e}")
            return None

    def __del__(self):
        """
        オブジェクトの破棄時にカメラリソースを解放します。
        """
        if self.cap.isOpened():
            self.cap.release()

def main():
    """
    カメラからリアルタイムでカラー画像を取得し、表示します。
    'q'キーを押すとプログラムを終了します。
    """
    camera = Camera()
    cv2.namedWindow("カメラ映像", cv2.WINDOW_NORMAL)

    try:
        while True:
            frame = camera.get_frame()
            if frame is None:
                break

            cv2.imshow("カメラ映像", frame)

            # 'q'キーで終了
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("リアルタイムカメラ表示を中断しました。")
    finally:
        del camera
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()