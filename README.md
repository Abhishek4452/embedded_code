# Floating Ball in a Water Canal — Simple Simulation

This project simulates a small ball floating in shallow, slow-moving water. The ball is held in place sideways (like it's tied with a string), so it can only bob **up and down**. The simulation shows how it moves when pushed around by three things: gentle waves, swirling water (vortices), and random turbulence.

Built in MATLAB. No extra toolboxes needed.

---

## What This Does

Think of the ball sitting on a spring in water:
- The water surface gently rises and falls (like a slow wave) → pushes the ball
- Water swirling around the ball (vortex shedding) → shakes it a bit faster
- Random turbulence → adds small, unpredictable jitters

The script:
1. Lets the ball bounce freely for a few seconds (like plucking a spring and letting go)
2. Then adds the real water forces and watches how it settles into a rhythm
3. Measures how much it shakes (RMS, peak movement, dominant frequency)
4. Shows you a live animation of the ball bobbing, plus 8 charts explaining the physics

## Quick Facts About This Setup

- The ball naturally wants to vibrate at about **2.95 times per second**
- It's lightly "bouncy" — vibrations fade out fairly quickly (within a couple seconds)
- The wave and vortex forces are both slower than the ball's natural rhythm, so nothing resonates badly
- Random turbulence is the one factor that could get closer to the resonant frequency if increased

## How to Run It

1. Save the script as `vertical_floating_ball_simulation.m`
2. Open MATLAB, press **Run** (or type `run('vertical_floating_ball_simulation.m')`)
3. A window pops up showing the ball animating, plus 8 explanatory charts
4. The command window prints a summary of numbers (forces, frequencies, etc.)

Takes about a minute to play through (68 seconds of simulated time).

Want to try different conditions? Open the script, change values near the top (Section 1) — like water speed, ball size, or stiffness — and re-run.

## Adding a Simulation Video to Your README

The script currently only *shows* the animation live — it doesn't save a video file automatically. Here's how to get one:

**Step 1: Record the animation as a video while it runs**

In the script, find the animation loop (near the end, Section 19) and add a few lines to save each frame as a video:

```matlab
% Add this BEFORE the animation loop starts:
videoObj = VideoWriter('docs/simulation_demo.mp4', 'MPEG-4');
videoObj.FrameRate = 25;
open(videoObj);

% Inside the loop, right after drawnow:
writeVideo(videoObj, getframe(fig));

% Add this AFTER the loop ends:
close(videoObj);
```

Run the script — you'll now have `docs/simulation_demo.mp4`.

**Step 2: Turn it into a GIF (recommended — GIFs always play inline on GitHub, videos sometimes don't)**

If you have [ffmpeg](https://ffmpeg.org/) installed, run this in your terminal:

```bash
ffmpeg -i docs/simulation_demo.mp4 -vf "fps=15,scale=720:-1" docs/simulation_demo.gif
```

**Step 3: Add it to your README**

```markdown
![Ball simulation demo](docs/simulation_demo.gif)
```

**Alternative for a real video (no ffmpeg needed):** Go to your README file on GitHub, click **Edit**, and drag-and-drop your `.mp4` file directly into the text box. GitHub uploads it and automatically creates a working, playable link for you.

## Good to Know (Limitations)

This is a simplified engineering model, not a full fluid dynamics simulation:
- The water forces are approximated with simple math (sine waves + filtered noise), not a real simulated flow
- The ball affects nothing about the water — only the water affects the ball
- Only vertical motion is modeled — no sideways movement or spinning

Great for spotting trends and testing "what if" scenarios — not a replacement for real experiments or full CFD software.

## License

No license yet. Add a `LICENSE` file (MIT, Apache-2.0, etc.) before sharing this publicly.
