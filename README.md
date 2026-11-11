# ROOMBACALYPSE ₍^. .^₎⟆

*nya~ an unnecessarily overengineered robot vacuum simulator, lovingly adopted by the Nekotania cat café* 🐾

> **DISCLAIMER:** This is a joke project with real engineering inside. The premise is silly. The math is not. Please don't tell the math.

## What is it, nya?

A tiny round robot vacuum lives in a 2D apartment. It has never seen the floor plan. It is not allowed to peek. It has to figure out where it is, map the whole place, plan its lanes, and clean like a very serious little roomba with a very serious little job.

We cats have been fighting these things for decades. So we built one. To understand the enemy. ฅ^•ﻌ•^ฅ

## Status: Milestone 4 of 7, "It has a plan" 🧶

Press **Tab** and it cleans in neat back-and-forth lanes, goes back for spots it missed, and stops when it's done. Very tidy. Very smug. Suspicious, honestly.

## What it can already do ✅

- 🗺️ **Builds its own map** with Graph SLAM and stays within about a centimetre of the truth. Better sense of direction than most cats.
- 📡 **Sees with LiDAR**, feels with a bumper, counts wheel turns, and has a gyro. It cannot see the red dot. Tragic.
- 🧹 **Plans real cleaning routes** on the map it's still building, then explores whatever it hasn't seen yet.
- 😿 **Knows when it's stuck** (cornered, spinning, bumping into everything) and climbs a whole ladder of escape tricks.
- 🙀 **Survives kidnapping.** Press **K** to pick it up and drop it somewhere else. It notices, panics politely, searches its whole map, and finds itself again. We cats respect this. We do this to roombas all the time.
- 🔊 **Optional bonk.** It can go *bonk* every time it hits something. Turn it on with `--sound bonk`. You will not turn it off.
- 🏠 **A six-room apartment** with a long hallway for when you want to watch it suffer properly.

## What it can't do yet 🗓

- 🔋 **Battery, dirt, and docking** (M5). It doesn't get tired yet. Lucky.
- 💢 **Feelings** (M6). Frustration, confidence, curiosity, and *battery anxiety*. And yes, it will file incident reports. This is why it's also called **Trigger-Me-Roomba**.
- 🏆 **Benchmarks and a release** (M7).

Nothing is marked done before it's tested. Cat's honor, nya.

## How to wake it up

```bash
uv venv
uv pip install -e ".[dev]"
python main.py                  # press Tab and watch it go
python main.py --room apartment # the big one
python main.py --sound bonk     # essential
```

Needs Python 3.12 or newer.

## House rules of the café

1. The robot never reads the answer key. Ever. There are tests that guard this.
2. Same seed, same run, every time. Even the noise is deterministic, nya.
3. If you kidnap it, you must watch it find its way home. It's only fair.

---

*made with paws, pose graphs, and one (1) bonk* 🐾 *nya~*
