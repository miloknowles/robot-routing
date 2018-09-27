# Appendix

I added some extra proofs here to show the correctness of my approach.

## Lemma 1
*There are at most 12 unique board states, which are cycled through in order.* Wormholes and lasers are the only board elements that change their behavior at each timestep. Since lasers change direction exactly 4 times in a cycle, a board has 4 unique states due to lasers; every 4th timestep, all lasers return to their initial direction.

Call these configurations: Q0, Q1, Q2, Q3

Wormholes appear every 3rd timestep, so their state shifts in time relative to the lasers. In the diagram below, wormholes are active on Q0 (at t=0), then Q3 at (t=3), then at Q2 at (t=6) and finally at Q1 (t=9). The cycle repeats at t=12, which I claim is equivalent to Q0 for planning purposes.

Q0 Q1 Q2 Q3
W        W
      W    
   W

## Lemma 2
*Any state on the optimal path must have at least 1 legal square to move to at the next timestep.* If the optimal path to the goal had a state at which there was no legal move to make at the next timestep, this would violate the requirement that a robot must move at every timestep. Clearly, this could not be a valid path to the goal, which proves the lemma above by contradiction.


## Lemma 3
Let x_min, x_max, y_min, and y_max be the outermost x and y coordinates of all obstacles, wormholes, start location, and end location. This means that all items given in the problem are contained within this box.
*Outside of the bounding box given by B:=([x_min, y_min], [x_max, y_max]) any square can only be occupied for one out of every four timesteps.* Outside the bounding box, at most one laser can strike a given square. It can only point there one out of every four timesteps. This square is free the other three timesteps.


## Proof 1
This proof is important because it bounds the size of the search space. Otherwise, my BFS algorithm could run indefinitely if no path to the goal exists.

Again, assume some B:=([x_min, y_min], [x_max, y_max]) is a bounding box containing every item on the map.

*We only need to consider states within 2 squares of padding around the bounding box ([x_min, y_min], [x_max, y_max])*.

From *Lemma 2* the robot must have at least one legal move at the next timestep. Without loss of generality, imagine the robot is to the left of the bounding box B, and wants to make progress downwards so that it can get around the lasers to the destination D. In this adversarial case, it is forced to move to the left by lasers above and below.

```
Right now:

      L
     RL*****
      L   D
      *
      *
      *

Next timestep:

 *****L
    R L
 *****L   D
```

This was the only move the robot could make, so it had to step further outside of the bounding box by one square (now 2 squares outside the bounding box).

At the next timestep, the laser below *must* be clear, so the robot can proceed downwards. Whenever a laser blocks the robot from below, the robot should step to the right to wait. Again, it only needs to go left and get further away from the boundary of B when it is adjacent to B and constrained from moving and further right. In any other case, there is no reason to move left, since this will only take the robot further from the goal.

Therefore, if the robot has to go around the bounding box B on the outside, it will never have to get more than 2 squares away.
