import asyncio

import numpy as np

from silverscreen_multicam import Silverscreen
from termcolor import cprint

async def ainput(prompt: str) -> str:
    return await asyncio.to_thread(input, prompt)


async def main(sscr):
    obs_fps = 50

    # await ainput("enter")
    cprint("Start main loop", "green")

    while True:
        await asyncio.sleep(1 / obs_fps)


if __name__ == "__main__":
    sscr = Silverscreen(fps=50, use_cert=True)

    sscr.app.run()
