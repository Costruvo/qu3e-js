# qu3e-js

This is a 1:1 javascript port of the lightweight 3D physics engine "qu3e" for use in browsers.

[![Demo](https://imgur.com/a/vOdl3g1)](https://imgur.com/a/vOdl3g1)

# Purpose

I decided to do this for a 3D game I've developed that I'd like to port to JS, so that it's playable in browsers (and be supported across many devices/platforms).
I needed physics to be synchronized across all clients, and also be fast enough to run on an iPhone, and what better way to do that than with the same engine.

# Important

This port needs a tiny bit of improving, mostly at different object scales, which can be seen in the demos.
They usually collide and generate the correct impulses, and other times the impulse is a bit too extreme.
It rarely misses a collision (an object will randomly fall through what it's on top of, or go inside another).
Nonetheless the majority of collisions are a success.

Any help is appreciated :)


All credits go to Randy Gaul
