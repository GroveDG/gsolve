
# Solving

Geometric constraint solving (GCS) is an NP problem, solving complexity increases exponentially. As such, the current philosophy of GCAD solving is that solvers should...
- return a single result without regard for other possible solutions.
- timeout according to reasonable limits on resources.
- not give false confidence (ie. "there is no solution" or "these are the only solutions" without having proven so).
# gsolve
This GCS solver is of my own design. It converts constraints into the geometry which represents all points which satisfy it, called the possibility space. These are then intersected to create a discrete set of points from which one is chosen and assumed to be the correct position to inform solving for the next point. If the set of possibility spaces have no valid intersections, the solver backtracks.
### Ordering
To determine the order in which to solve the points, GSolver uses a breadth-first search.

For each origin/root point, The solver picks a second point called an orbiter. The orbiter's position is arbitrarily selected from a possibility space applicable from only the root. It is called an orbiter because if the constraint were a Distance constraint, the possibility space would be a circle and this point could "orbit" the root.

Then all possibility spaces currently applicable to all unknown are counted up. A point is considered discrete (and therefore known for ordering purposes) when two or more 1D constraints are applied. This is assumed because two 1D elements (lines, including curves) intersect at a finite set of 0D elements (points) as long as they are not identical for some continuous range.

Constraints whose possibility space is a 2D element (an area) are applied only to already discrete sets of points and are not counted towards the two constraints necessecary for discretizing.

If there are no more unknown points, the current order is declared useable and the solver moves onto Solving. Otherwise it moves to the next root-oribiter pair. If this pair has already appeared in another order, it's order must be a subset of a previous order and can be skipped. Otherwise, Ordering repeats until a valid order has been found or there are no valid root-orbiter pairs.

Currently an order is only valid if it contains all points. There is likely a way to seperate independent orders into their own figures.
### Solving
Solving is straightforward with ordering completed beforehand. All applicable possibility spaces for the next point are intersected. Select an intersection point to use as this point's position. If there are no intersection points, backtrack.

A currently unimplemented technique is to select the intersection point based on which is the closest to a future point's current possibility space. This should keep long chains from wandering off when they will need to loop back around to another point.