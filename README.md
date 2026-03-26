# qu3e-js

A 1:1 javascript port of the lightweight 3D physics engine "qu3e" for use in browsers. Here's a demo:<br>

![hippo](https://raw.githubusercontent.com/Costruvo/qu3e-js/refs/heads/main/bandicam%202026-03-26%2014-17-44-088.gif)

# Purpose

I did this for a 3D game I'm developing that I'd like to port to JS, so that it's playable in browsers (and supported on many devices).<br>
I needed physics to be synchronized across all clients, and also be fast enough to run on an iPhone, and what better way to do that than with the same engine.<br>

# Important

This port needs a little fine-tuning, mainly at different object scales (see below).<br>
![hippo](https://raw.githubusercontent.com/Costruvo/qu3e-js/refs/heads/main/bandicam%202026-03-26%2014-18-35-016.gif)

Every 9 out of 10 times they collide and generate the correct impulses, but sometimes the impulse is a little extreme.<br>
It rarely misses a collision, but it usually corrects itself.<br>
Some help with tuning it is always appreciated :)<br>

All credits go to Randy Gaul
