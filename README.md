# qu3e-js

This is a 1:1 javascript port of the lightweight 3D physics engine "qu3e" for use in browsers.<br>

![](https://i.imgur.com/F5JHVn8.mp4)<br>

![](https://i.imgur.com/1jLcZqr.mp4)<br>


# Purpose

I did this for a 3D game I'm developing that I'd like to port to JS, so that it's playable in browsers (and supported on many devices).<br><br>
I needed physics to be synchronized across all clients, and also be fast enough to run on an iPhone, and what better way to do that than with the same engine.<br>

# Important

This port needs a tiny bit of improving, mostly with different object scales, which can be seen in the demos.<br>
They usually collide and generate the correct impulses, but other times the impulse is a bit extreme.<br>
It rarely misses a collision, but when it does the object will randomly fall through what it's on, or go inside another.<br>
But it usually corrects itself.<br>

Any help with fine-tuning this is much appreciated<br>
<br>

All credits go to Randy Gaul
