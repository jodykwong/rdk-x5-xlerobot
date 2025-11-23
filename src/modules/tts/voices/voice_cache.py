#!/usr/bin/env python3
"""语音缓存 - 占位符实现"""

class VoiceCache:
    def __init__(self):
        self.cache = {}
    
    def get(self, key):
        return self.cache.get(key)
    
    def set(self, key, value):
        self.cache[key] = value
