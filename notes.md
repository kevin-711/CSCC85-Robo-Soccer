Interpreting robot, goal, ball, and wall locations
Based on the imageCapture.c code we examined, the key finding is that the project transforms the camera image into a rectified field coordinate system before detecting blobs. This gives us a consistent coordinate system that we can use to locate the robots, ball, goals, and walls.
1. Understanding the coordinate system
The project uses two coordinate systems:
1. Original camera coordinates: The raw image captured by the webcam. Perspective makes distances and positions distorted.
2. Rectified field coordinates: The image after applying the homography transformation. The field is mapped into a rectangular image.
The getH() function defines the four corners of the rectified field:
corners2[0][0] = 1.0;
corners2[0][1] = 1.0;

corners2[1][0] = sx - 1;
corners2[1][1] = 1.0;

corners2[2][0] = sx - 1;
corners2[2][1] = sy - 1;

corners2[3][0] = 1.0;
corners2[3][1] = sy - 1;


Here:
- sx = width of the rectified image.
- sy = height of the rectified image.
- (1, 1) = approximate top-left field corner.
- (sx - 1, sy - 1) = approximate bottom-right field corner.
fieldUnwarp2() uses the homography to map the original camera image into this rectified coordinate system.

Important: blobDetect2() detects blobs in fieldIm, meaning its resulting centroid coordinates are already in the rectified coordinate system.

Therefore, when we access:
ai->st.self->cx
ai->st.self->cy


we are getting the robot's approximate position in rectified field-image coordinates, not its original camera-image coordinates.
2. Determining goal locations
Since the field is represented as a rectangle, we can approximate the goal locations using its boundaries.

The vertical midpoint is:
double goalY = sy / 2.0;


The left and right goal positions are approximately:
// Left goal
double leftGoalX = 1.0;
double leftGoalY = sy / 2.0;

// Right goal
double rightGoalX = sx - 1.0;
double rightGoalY = sy / 2.0;












These coordinates represent the approximate centers of the left and right field boundaries.

One limitation: these are not necessarily the exact centers of the physical goal openings. If the goal mouth is inset from the boundary or its location is offset, we would need to account for that separately.
3. Determining global positions of robots and the ball
Because the robot, opponent, and ball are detected in the same rectified image, their centroid coordinates can be treated as global field-image positions.

For example:
double robotX = ai->st.self->cx;
double robotY = ai->st.self->cy;

double opponentX = ai->st.opp->cx;
double opponentY = ai->st.opp->cy;

double ballX = ai->st.ball->cx;
double ballY = ai->st.ball->cy;


These are all positions relative to the same field coordinate system.

We can therefore calculate the relative position of one object to another:
double relativeX = ballX - robotX;
double relativeY = ballY - robotY;


This tells us how far the ball is from the robot in image-coordinate units.

Similarly, we can calculate the distance between two objects:
\[
d = \sqrt{(x_2-x_1)^2+(y_2-y_1)^2}
\]

For example, this allows us to determine:
- How far the robot is from the ball.
- How far the robot is from the opponent.
- How far the robot is from its target goal.
- Whether the opponent is obstructing the path to the ball.
Note: These are global positions in the rectified image coordinate system, not yet physical positions in centimetres or metres.
4. Determining wall locations
The field boundaries can also be represented using the rectified image dimensions.
Wall	Approximate coordinate
Left	\(x = 1\)
Right	\(x = sx - 1\)
Top	\(y = 1\)
Bottom	\(y = sy - 1\)

For example, to determine whether the robot is close to the right wall:
double distanceToRightWall = (sx - 1) - robotX;


To determine whether it is close to the top wall:
double distanceToTopWall = robotY - 1;


This lets us implement boundary checks and prevent the robot from driving outside the field.
5. Converting image coordinates into physical coordinates
Currently, the positions are measured in pixels. If we want actual physical distances, we need to establish the real-world dimensions of the field.

For example, suppose the field has a known physical width and height:
- Physical width: \(W\) centimetres.
- Physical height: \(H\) centimetres.
- Rectified image width: approximately \(sx-2\) pixels.
- Rectified image height: approximately \(sy-2\) pixels.
We can approximate the conversion using:
\[
X_{\text{cm}} = \frac{x-1}{sx-2}W
\]

\[
Y_{\text{cm}} = \frac{y-1}{sy-2}H
\]

This assumes the rectified field is mapped uniformly to a rectangle with the same aspect ratio as the physical field.

Once converted, the robot, opponent, ball, goals, and walls can all be represented in centimetres.

6. Overall representation
The resulting coordinate system can be summarized as follows:
Rectified field coordinate system
All positions are represented using the same rectified field-image coordinate system.

Object	Global position
Our robot	(self->cx, self->cy)
Opponent	(opp->cx, opp->cy)
Ball	(ball->cx, ball->cy)
Left goal	(1, sy / 2.0)
Right goal	(sx - 1, sy / 2.0)
Top wall	y = 1
Bottom wall	y = sy - 1
Left wall	x = 1