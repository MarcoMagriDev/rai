# Copyright (C) 2024 Robotec.AI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import re
import threading
import time
from typing import cast

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .tts_clients import ElevenLabsClient, OpenTTSClient, TTSClient
from .rai_tts_parameters import rai_tts_node as rai_tts_parameters

from .tts_audio_players import (
    TTSJob,
    AudioPlayer,
    FFPlayAudioPlayer,
    SoundDeviceAudioPlayer,
)


class TTSNode(Node):
    def __init__(self):
        super().__init__("rai_tts_node")
        self.param_listener = rai_tts_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.subscription = self.create_subscription(  # type: ignore
            String, self.params.topic, self.listener_callback, 10  # type: ignore
        )
        self.status_publisher = self.create_publisher(String, "tts_status", 10)  # type: ignore
        self.job_id: int = 0
        self.device_indexes = None
        self.tts_client = self._initialize_client()
        self.create_timer(0.01, self.status_callback)

        self.audio_player = AudioPlayer(FFPlayAudioPlayer(), self.get_logger())
        self.cancel_speaking_service = self.create_service(
            Trigger, "cancel_speaking", self.cancel_speaking_callback
        )
        self.get_logger().info("TTS Node has been started")  # type: ignore

    def status_callback(self):
        if self.audio_player.active:
            self.status_publisher.publish(String(data="waiting"))
        else:
            self.status_publisher.publish(String(data="playing"))

    def listener_callback(self, msg: String):
        self.playing = True
        self.get_logger().info(  # type: ignore
            f"Registering new TTS job: {self.job_id} length: {len(msg.data)} chars."  # type: ignore
        )
        self.params = self.param_listener.get_params()
        threading.Thread(
            target=self.synthesize_speech, args=(self.job_id, msg.data), daemon=True  # type: ignore
        ).start()
        self.job_id += 1

    def synthesize_speech(
        self,
        id: int,
        text: str,
    ) -> str:
        text = self._preprocess_text(text)
        if id > 0:
            time.sleep(0.5)

        tries = 0
        while tries < 2:
            try:
                temp_file_path = self.tts_client.synthesize_speech_to_file(text)
                break
            except Exception as e:
                self.get_logger().warn(f"Error ocurred during synthesizing speech: {e}.")  # type: ignore
                tries += 1
        else:
            self.get_logger().error(f"Error ocurred during synthesizing speech. Unable to proceed.")  # type: ignore
            return

        self.get_logger().info(f"Job {id} completed.")  # type: ignore
        tts_job = TTSJob(id, text, temp_file_path, self.params.device_indexes)
        self.audio_player.add_job(tts_job)
        return temp_file_path

    def _initialize_client(self) -> TTSClient:
        if self.params.tts_client == "opentts":
            return OpenTTSClient(
                base_url=self.params.base_url,
                voice=self.params.voice,
            )
        elif self.params.tts_client == "elevenlabs":
            return ElevenLabsClient(
                base_url=self.params.base_url,
                voice=self.params.voice,
            )
        else:
            raise ValueError(f"Unknown TTS client: {self.params.tts_client}")

    def _preprocess_text(self, text: str) -> str:
        """Remove emojis from text."""
        emoji_pattern = re.compile(
            "["
            "\U0001F600-\U0001F64F"  # emoticons
            "\U0001F300-\U0001F5FF"  # symbols & pictographs
            "\U0001F680-\U0001F6FF"  # transport & map symbols
            "\U0001F1E0-\U0001F1FF"  # flags (iOS)
            "]+",
            flags=re.UNICODE,
        )
        text = emoji_pattern.sub(r"", text)
        return text

    def cancel_speaking_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        if not self.audio_player.active:
            response.success = True
            response.message = "No speech in progress. Nothing to cancel."
            return response

        response.success = self.audio_player.cancel()
        response.message = (
            "Speech cancelled." if response.success else "Unable to cancel speech."
        )
        return response


def main():
    rclpy.init()

    tts_node = TTSNode()

    rclpy.spin(tts_node)

    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
