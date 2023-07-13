# Day 4 - July 13

## Ethics stuff

- https://ucsd.libguides.com/cosmos
	- https://ucsd.libguides.com/cosmos/robots
- https://ucsd.libguides.com/stem-open
- https://library.ucsd.edu
- Get on protected wifi to get access to it
- Use some primary sources
	- Balance between opinion, research sources
- `robot*` => robots, robotics, etc
- Start:
	- UC library search
	- Springer Link
		- https://link.springer.com/search?facet-discipline=%22Engineering%22&facet-sub-discipline=%22Control%2C+Robotics%2C+Mechatronics%22&just-selected-from-overlay=facet-sub-discipline&just-selected-from-overlay-value=%22Control%2C+Robotics%2C+Mechatronics%22
- Try to narrow search
	- Dates: 2018+
	- Search: library catologe
- Focus on abstract, intro, and conclusion
	- Big picture

### Topic

- Robots in law enforcement
- Includes search and rescue

### Citation style

- Superscript
	- Number matches a numbered list of bibliography
- MLA for bibliography
- Bibliography doesn't have to be in order

## Image Processing

For week 2

### Color theory

- (0, 0) = top left
- Light: additive (vs paint: subtractive)
- Colorspace: a way to represent a color
	- Examples: RGB, HSV, CMYK
- HSV
	- Hue: color
	- Saturation: how much color
	- Value: how bright
	- Good for color detection
		- Because hue is the same regardless of brightness/lighting

### NumPy

- n dimensional array
	- 1D: vector
	- 2D: matrix
	- 3D: tensor
	- ND: ND array
- All types with an array should be the same type
- `import numpy as np`
- `mat = np.ones( (4, 3) )` => 4x3 array of 1s (4 tall, 3 wide)
- `mat[1, 2]` => 2nd row, 3rd column
	- Note: indexing by `[row, column, etc]` instead of `[row][column][etc]`
- `mat.shape` => `(4, 3)`
- `mat2 = mat` : reference, not a copy
	- `mat2[0, 0] = 5` => `mat[0, 0]` is also 5
	- `mat2 = mat.copy()` : copy
- Slicing:
	- `mat2 = mat[1:3, 1:3]` => 2x2 array
	- `mat2 = mat[1:3, :]` => 2x3 array (`:` means all)
	- Note: does not copy, just references that section
	- Can use slicing as the left side of assignment
		- `mat[0:2, :] = mat[2:4, :]` => 3rd and 4th rows now = 1st and 2nd rows
		- `mat[0:2, :] = 5` => 1st and 2nd rows now = 5
	- Can slice backwards
		- `mat[::-1, :]` => reverse rows (up to down flip)
		- `mat[:, ::-1]` => reverse columns (left to right flip)
		- `mat[2:4, :] = mat[3:1:-1,:]`
- Can do scalar math (`mat = mat + 5`)
- Can create boolean arrays (`mat2 = mat > 5`)
- Sum: `mat.sum()`
	- Sum on axis: `mat.sum(axis=0)` => sum of each column
		- `axis=1` => sum of each row