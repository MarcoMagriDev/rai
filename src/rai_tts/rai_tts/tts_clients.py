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

import os
import tempfile
from abc import abstractmethod
from typing import Optional, Tuple
import wave

import requests
from elevenlabs.client import ElevenLabs
from piper.voice import PiperVoice
from piper.download import ensure_voice_exists, get_voices, find_voice


class TTSClient:
    @abstractmethod
    def synthesize_speech_to_file(self, text: str) -> str:
        pass

    @staticmethod
    def save_audio_to_file(audio_data: bytes, suffix: str) -> str:
        with tempfile.NamedTemporaryFile(
            delete=False, suffix=suffix
        ) as temp_audio_file:
            temp_audio_file.write(audio_data)
            temp_file_path = temp_audio_file.name

        return temp_file_path

    @staticmethod
    def get_temp_file(suffix: str) -> str:
        with tempfile.NamedTemporaryFile(
            delete=False, suffix=suffix
        ) as temp_audio_file:
            temp_file_path = temp_audio_file.name

        return temp_file_path


class ElevenLabsClient(TTSClient):
    def __init__(self, voice: str, base_url: Optional[str] = None):
        self.base_url = base_url
        self.voice = voice
        api_key = os.getenv(key="ELEVENLABS_API_KEY")
        self.client = ElevenLabs(base_url=None, api_key=api_key)

    def synthesize_speech_to_file(self, text: str) -> str:
        response = self.client.generate(
            text=text,
            voice=self.voice,
            optimize_streaming_latency=4,
        )
        audio_data = b"".join(response)
        return self.save_audio_to_file(audio_data, suffix=".mp3")


class OpenTTSClient(TTSClient):
    def __init__(self, base_url: Optional[str] = None, voice: Optional[str] = None):
        self.base_url = base_url
        self.voice = voice

    def synthesize_speech_to_file(self, text: str) -> str:
        params = {
            "voice": self.voice,
            "text": text,
        }
        response = requests.get("http://localhost:5500/api/tts", params=params)
        response.raise_for_status()

        return self.save_audio_to_file(response.content, suffix=".wav")


class PiperTTSClient(TTSClient):
    def __init__(
        self,
        voice: str,
        base_url: Optional[str] = None,
        use_cuda: Optional[bool] = False,
    ):
        self.base_url = base_url
        self.voice = voice
        self.client = PiperVoice.load(*self.get_voice_model(voice), use_cuda=use_cuda)

    def get_voice_model(self, voice: str) -> Tuple[str, Optional[str]]:
        try:
            data_dir = os.environ["PIPER_DATA_DIR"]
        except KeyError:
            data_dir = os.getcwd()

        model = os.path.join(data_dir, voice)
        if os.path.exists(model):
            return model, None

        voices_info = get_voices(data_dir)
        aliases_info = {}
        for voice_info in voices_info.values():
            for voice_alias in voice_info.get("aliases", []):
                aliases_info[voice_alias] = {"_is_alias": True, **voice_info}

        voices_info.update(aliases_info)
        ensure_voice_exists(model, data_dir, data_dir, voices_info)
        return find_voice(model, data_dir)

    def synthesize_speech_to_file(self, text: str) -> str:
        temp_file_name = self.get_temp_file(".wav")
        self.client.synthesize(text, wave.Wave_write(temp_file_name))
        return temp_file_name
