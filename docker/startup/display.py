#!/usr/bin/env python3
# Created by Nelson Durrant, Mar 2025
from asciimatics.effects import Print, Stars
from asciimatics.renderers import ColourImageFile, FigletText, SpeechBubble
from asciimatics.scene import Scene
from asciimatics.screen import Screen
from asciimatics.exceptions import ResizeScreenError
import os
import sys

# Go to the directory where the script is located
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)


def display(screen):
    scenes = []
    effects = [
        Print(screen,
              ColourImageFile(screen, "mars.gif", 2*screen.height//3,
                              uni=screen.unicode_aware),
              screen.height//6),
        Print(screen,
              FigletText("MARS ROVER",
                         font='banner3' if screen.width > 90 else 'banner'),
              screen.height//2-3,
              colour=7, bg=7 if screen.unicode_aware else 0),   
        Stars(screen, (screen.width + screen.height) // 2),
        Print(screen,
              SpeechBubble("Press SPACE to continue..."),
              5*screen.height // 6,
              speed=1,
              transparent=False)
    ]
    scenes.append(Scene(effects, -1))

    screen.play(scenes, stop_on_resize=True, repeat=False)


if __name__ == "__main__":
    while True:
        try:
            Screen.wrapper(display)
            sys.exit(0)
        except ResizeScreenError:
            pass
