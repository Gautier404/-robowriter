### Inverse Kinematics
- Addition of bounds in terms of length and how to handle
- We need to map our theoretical angles for the inverse kinematics to the actual angles of the motors (there's some offset and mabye inverse scaling that needs to happen)

### Control Function
- something that takes in a angular toolpath and executes it on the robot
- Will need to be tested

### Cartesian Toolpath Generation
- take an svg and convert it to a series of cartesian points
- How do we know where to place the toolpath?
- How to scale it?
- How to divide up long motions into shorter interpolated paths