# Day 3 - July 12

## FSM

### Picobot

https://www.cs.hmc.edu/picobot/

#### Example

```
0 x*** -> N 0
0 N*x* -> X 1
0 N*w* -> S 1
1 *E** -> S 1
1 *x** -> S 1
```

#### Testing

- State 0
	- If can go north, go north
	- Else if can go east, go east, state 1
	- Else if can go west, go west, state 2
	- Else if can go south, go south, state 3
- State 1
	- If can go east, go east
	- Else if can go north, go north, state 0
	- Else if can go south, go south, state 3
	- Else if can go west, go west, state 2
- State 2
	- If can go west, go west
	- Else if can go north, go north, state 0
	- Else if can go south, go south, state 3
	- Else if can go east, go east, state 1
- State 3
	- If can go south, go south
	- Else if can go east, go east, state 1
	- Else if can go west, go west, state 2
	- Else if can go north, go north, state 0

```
0 x*** -> N 0
0 Nx** -> E 1
0 NEx* -> W 2
0 NEwx -> S 3

1 *x** -> E 1
1 xE** -> N 0
1 NE*x -> S 3
1 NExS -> W 2

2 **x* -> W 2
2 x*W* -> N 0
2 N*Wx -> S 3
2 NxWS -> E 1

3 ***x -> S 3
3 *x*S -> E 1
3 *ExS -> W 2
3 xEWS -> N 0
```

#### Maze

- General idea: left hand rule
	- If can turn left, turn left
	- Else if can go forward, go forward
	- Else turn right
- State 0: `fowards` = North
	- `left` = West
	- `right` = East
	- `back` = South
	- If can go West, state 1
	- Else if can go North, go North, state 0
	- Else: state 2
- State 1: `fowards` = West
	- `left` = South
	- `right` = North
	- `back` = East
	- If can go South, state 3
	- Else if can go West, go West, state 1
	- Else: state 0
- State 2: `fowards` = East
	- `left` = North
	- `right` = South
	- `back` = West
	- If can go North, state 0
	- Else if can go East, go East, state 2
	- Else: go South, state 3
- State 3: `fowards` = South
	- `left` = East
	- `right` = West
	- `back` = North
	- If can go East, state 2
	- Else if can go South, go South, state 3
	- Else: state 1

```
# state sensors -> direction state
# sensors order: NEWS

0 **x* -> W 1 # if can turn west, turn and go west
0 x*W* -> N 0 # elif can go north, go north
0 N*W* -> X 2 # else turn east

1 ***x -> S 3 # if can turn south, turn and go south
1 **xS -> W 1 # elif can go west, go west
1 **WS -> X 0 # else turn north

2 x*** -> N 0 # if can turn north, turn and go north
2 Nx** -> E 2 # elif can go east, go east
2 NE** -> X 3 # else turn south

3 *x** -> E 2 # if can turn east, turn and go east
3 *E*x -> S 3 # elif can go south, go south
3 *E*S -> X 1 # else turn west
```

#### Diamond - Not finished

- General idea
	- Up as much as possible
	- If can't go up, try going left
		- if: Left -> down as much as possible -> up as much as possible
		- else: down -> left -> down as much as possible -> up as much as possible
	- When get to left corner, go right until can't
- State 0: up as much as possible
	- If can go north, go north, state 0
	- Elif can go west go west, state 1
	- Else: go south, state 2
- State 1: down as much as possible
	- If can go south, go south, state 1
	- Else: up, state 0
- State 2:
	- Go left, state 2
- State 3: right as much as possible
	- If can go east, go east, state 3
	- Else: state 0
```
0 x*** -> N 0
0 N*x* -> W 1
0 N*W* -> S 2
0 NxWS -> E 3

1 ***x -> S 1
1 ***S -> N 0
0 NxWS -> E 3

2 **** -> W 1

3 *x** -> E 3
3 *E*x -> X 0
```