from abc import ABC, abstractmethod
import logging
import subprocess
import threading
import time
from queue import PriorityQueue
from typing import List, NamedTuple
import rclpy


class TTSJob(NamedTuple):
    id: int
    text: str
    file_path: str
    device_indexes: List[str]


class AudioClient(ABC):
    @abstractmethod
    def play_sync(self, audio_file_path: str, device_indexes: List[int] = None):
        pass

    @abstractmethod
    def cancel_sync(self):
        pass


class AudioPlayer:
    def __init__(self, audio_client: AudioClient, logger: logging.Logger) -> None:
        self.logger = logger
        self.queue: PriorityQueue[TTSJob] = PriorityQueue()
        self._playing = False
        self.audio_client = audio_client

        threading.Thread(target=self._process_queue, daemon=True).start()

    def add_job(self, tts_job: TTSJob):
        self.queue.put(tts_job)

    def cancel(self):
        if self._playing:
            self.audio_client.cancel_sync()
        self.queue = PriorityQueue()
        return True

    def _process_queue(self):
        # TODO: remove ros dependency
        while rclpy.ok():
            time.sleep(0.01)
            if not self.queue.empty():
                tts_job = self.queue.get()
                self.logger.info(  # type: ignore
                    f"Playing audio for job {tts_job.id}. {tts_job.file_path}"
                )
                self._playing = True
                self.audio_client.play_sync(tts_job.file_path, tts_job.device_indexes)
                self._playing = False

    @property
    def active(self):
        return not self.queue.empty() or self._playing

    @property
    def missing_text(self):
        return " ".join([q[1] for q in self.queue.queue])


class FFPlayAudioPlayer(AudioClient):
    def __init__(self) -> None:
        super().__init__()
        self.processes: List[subprocess.Popen] = None

    def play_sync(self, audio_file_path: str, device_indexes: List[int] = None):
        # TODO: find a way to use multiple device_indexes
        self.processes = [
            subprocess.Popen(
                ["ffplay", "-v", "0", "-nodisp", "-autoexit", audio_file_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        ]
        while True:
            finished = [not p.poll() is None for p in self.processes]
            if all(finished):
                break
            time.sleep(0.01)

    def cancel_sync(self):
        if self.processes is None:
            return
        for p in self.processes:
            p.kill()


class SoundDeviceAudioPlayer(AudioClient):
    def __init__(self) -> None:
        super().__init__()

    def play_sync(self, audio_file_path: str, device_indexes: List[int] = None):
        pass

    def cancel_sync(self):
        return super().cancel_sync()
