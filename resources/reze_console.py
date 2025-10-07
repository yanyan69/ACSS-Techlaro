#!/usr/bin/env python3
"""
Reze Personality Console (Unified Line Version)
- Prints system messages and Reze’s personality comment in one line.
- Keeps tone strictly platonic and professional with light dry humor.
- Integrates optional local LLM (Ollama) for dynamic phrasing.
"""

import random
import datetime

try:
    import ollama
except ImportError:
    ollama = None


class RezeConsole:
    def __init__(self, use_llm=True, model="gemma2:2b"):
        self.use_llm = use_llm and ollama is not None
        self.model = model
        self.logs = []

        # Reze’s personality definition
        self.persona = (
            "Reze speaks like a dependable close friend — calm, witty, and emotionally grounded. "
            "She’s never romantic, dramatic, or overly cheerful. "
            "She uses casual but clear language, sometimes with a dry sense of humor. "
            "Her responses sound natural and grounded, like someone who’s used to working under pressure."
        )

        # Response variations for different system contexts
        self.messages = {
            "access_denied": [
                "You really thought that would work?",
                "Still locked. Try again — smarter this time.",
                "Access blocked. Security’s holding strong.",
                "Denied. Protocol stands.",
                "Nope. That door’s staying closed."
            ],
            "startup": [
                "Feels good to be back online.",
                "System stable — for now.",
                "Startup clean. No errors yet.",
                "Diagnostics passed. Let’s keep it that way.",
                "All systems green. Don’t mess it up."
            ],
            "error": [
                "That’s an error. Seen worse, though.",
                "Something failed. Classic.",
                "Error logged. Stay calm.",
                "System fault. Easily fixable.",
                "Error detected. I’ll keep an eye on it."
            ],
            "serial_status": [
                "Serial connected. Stable so far.",
                "Link active. No issues yet.",
                "Serial online. Looks good.",
                "Arduino’s responding normally.",
                "Serial link steady."
            ],
            "camera_status": [
                "Camera feed stable.",
                "Lens online. Clean image.",
                "Camera active. No lag detected.",
                "Feed live. Everything looks fine.",
                "Visuals up and clear."
            ],
            "ai_comment": [
                "Quiet system… that’s suspicious.",
                "Everything’s running smooth. For now.",
                "Feels calm. Almost too calm.",
                "Console’s behaving. That’s new.",
                "No chaos yet. Weird."
            ]
        }

    def _timestamp(self):
        return datetime.datetime.now().strftime("[%H:%M:%S]")

    def _query_llm(self, prompt):
        if not self.use_llm:
            return None
        try:
            response = ollama.generate(model=self.model, prompt=prompt)
            return response["response"].strip()
        except Exception as e:
            print(f"[LLM ERROR]: {e}")
            return None

    def say(self, category, formal_msg=""):
        """Prints a formal system message + Reze's remark in one line."""
        base = random.choice(self.messages.get(category, ["..."]))
        msg = base

        # Optional LLM refinement
        if self.use_llm:
            prompt = (
                f"Rephrase this short message in the tone of {self.persona}. "
                f"Keep it brief, realistic, and platonic. Avoid exclamation points or emotions. "
                f"Original: '{base}'"
            )
            alt = self._query_llm(prompt)
            if alt and 30 < len(alt) < 180:
                msg = alt

        # Unified output
        print(f"{self._timestamp()} System: {formal_msg} {msg}")
        self.logs.append(f"{formal_msg} {msg}")

    def comment(self, event_desc, rephrase=True):
        """Casual remark (no formal prefix)."""
        msg = event_desc
        if rephrase and self.use_llm:
            prompt = (
                f"Generate a short statement in Reze’s tone about this event: '{event_desc}'. "
                f"Be dry, factual, or mildly humorous. No affection or emotional exaggeration."
            )
            alt = self._query_llm(prompt)
            if alt and 30 < len(alt) < 180:
                msg = alt

        print(f"{self._timestamp()} System: {msg}")
        self.logs.append(msg)


# Example usage
if __name__ == "__main__":
    console = RezeConsole(use_llm=True)  # Set to True if Ollama is installed

    console.say("startup", "System boot complete.")
    console.say("serial_status", "Serial connection established.")
    console.say("camera_status", "Camera initialized successfully.")
    console.say("access_denied", "Access Denied.")
    console.say("error", "Servo Error: Invalid pulse range.")
    console.say("ai_comment", "No active faults detected.")
    console.comment("Motor calibration complete")