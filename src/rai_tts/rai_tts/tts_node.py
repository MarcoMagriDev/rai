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
import subprocess
import threading
import time
from queue import PriorityQueue
from typing import NamedTuple, cast

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .tts_clients import ElevenLabsClient, OpenTTSClient, TTSClient
from .rai_tts_parameters import rai_tts_node as rai_tts_parameters


class TTSJob(NamedTuple):
    id: int
    file_path: str


class TTSNode(Node):
    def __init__(self):
        super().__init__("rai_tts_node")
        self.param_listener = rai_tts_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.subscription = self.create_subscription(  # type: ignore
            String, self.params.topic, self.listener_callback, 10  # type: ignore
        )
        self.playing = False
        self.status_publisher = self.create_publisher(String, "tts_status", 10)  # type: ignore
        self.queue: PriorityQueue[TTSJob] = PriorityQueue()
        self.it: int = 0
        self.job_id: int = 0
        self.tts_client = self._initialize_client()
        self.create_timer(0.01, self.status_callback)
        threading.Thread(target=self._process_queue).start()
        self.get_logger().info("TTS Node has been started")  # type: ignore

    def status_callback(self):
        if self.queue.empty() and self.playing is False:
            self.status_publisher.publish(String(data="waiting"))
        else:
            self.status_publisher.publish(String(data="playing"))

    def listener_callback(self, msg: String):
        self.playing = True
        self.get_logger().info(  # type: ignore
            f"Registering new TTS job: {self.job_id} length: {len(msg.data)} chars."  # type: ignore
        )
        threading.Thread(
            target=self.synthesize_speech, args=(self.job_id, msg.data)  # type: ignore
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
        temp_file_path = self.tts_client.synthesize_speech_to_file(text)
        self.get_logger().info(f"Job {id} completed.")  # type: ignore
        tts_job = TTSJob(id, temp_file_path)
        self.queue.put(tts_job)

        return temp_file_path

    def _process_queue(self):
        while rclpy.ok():
            time.sleep(0.01)
            if not self.queue.empty():
                if self.queue.queue[0][0] == self.it:
                    self.it += 1
                    tts_job = self.queue.get()
                    self.get_logger().info(  # type: ignore
                        f"Playing audio for job {tts_job.id}. {tts_job.file_path}"
                    )
                    self._play_audio(tts_job.file_path)

    def _play_audio(self, filepath: str):
        subprocess.run(
            ["ffplay", "-v", "0", "-nodisp", "-autoexit", filepath],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.get_logger().debug(f"Playing audio: {filepath}")  # type: ignore
        self.playing = False

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


def main():
    rclpy.init()

    tts_node = TTSNode()

    rclpy.spin(tts_node)

    tts_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
