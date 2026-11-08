"""Test-wide settings."""
import os

os.environ.setdefault("LLM_NO_DOTENV", "1")   # never use the real Gemini key from .env in tests
