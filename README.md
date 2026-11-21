# Sokoban Solver

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/rezimx-btc/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/rezimx-btc/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver written for the make.fun competition
`sokoban-solver-01-run-20261121`. It reads every level under the input
directory, solves each one, and writes the player's move string
(`u`/`d`/`l`/`r`) for every instance.

## How it works

The generated levels share one structure: a single hallway across the
board with one to four small rooms off it. A box pushed out of its room
through the door can never be pushed back, so every box stays in its
room — and the push-optimal total for a level is the sum of the rooms'
optima. The solver exploits that:

1. **Room decomposition.** The hallway row (the player's row) is
   removed and the remaining open floor splits into connected
   components: one room each, with its own boxes, goals and door cells.
2. **Push-optimal A\* per room.** The search state is the sorted box
   positions plus the player's reachability region (canonicalised by
   its smallest cell — the exact player cell inside a region does not
   change which pushes are possible). Cost is pushes; between pushes
   the player walks by BFS. The heuristic is the cheapest assignment
   of boxes to goals by precomputed walking distance, which is
   admissible. Pruning covers static dead squares (corners and
   goal-less wall runs) and 2x2 box/wall blocks that trap a box.
3. **Stitching.** Rooms are solved in order, each search starting
   where the previous one left the player; the first walk of the next
   room already covers the hallway transit. Every room except the last
   must leave the player able to reach the hallway again, so the next
   room stays enterable.

The solver is deterministic: fixed neighbour order, a binary heap with
insertion-order tie-breaking, and no randomness or wall-clock decisions
in the search itself, so two runs write identical bytes.

## Run locally

```sh
MAKEFUN_INPUT_DIR=./practice/input MAKEFUN_OUTPUT_DIR=./out node main.ts
```

`node main.ts` reads `$MAKEFUN_INPUT_DIR/<id>/level.txt` for every
instance directory and writes `$MAKEFUN_OUTPUT_DIR/<id>/solution.txt`
(defaults `/input` and `/output`). It needs nothing but node 22+ and
the standard library — no dependencies, no network.

On the five practice levels the solver reaches the reference optima
exactly: i001 13, i002 16, i003 29, i004 47, i005 60 pushes.
