# qu3e-js

This is a 1:1 javascript port of the lightweight 3D physics engine qu3e for use in browsers.

[![Demo](https://imgur.com/a/vOdl3g1)](https://imgur.com/a/vOdl3g1)

# Purpose

I decided to do this for a 3D game I've been developing that I'd like to port to JS, so that it's playable in browsers (and support other devices).
I also wanted something fast and simple that performs well even on an iphone.
But most importantly I wanted physics to be perfectly synchronized across all clients, and what better way to do that than use the same engine.

# Important

This port needs just a bit of improving, mainly for different object scales.
They usually collide and generate the correct impulses, other times the impulse is a bit extreme.
And other times it simply misses a collision (an object will randomly fall through what it's on top of, or go
inside another).
However the majority of collisions are a success..

If anyone's interested in lending some help, let me know :)


All credits go to Randy Gaul
