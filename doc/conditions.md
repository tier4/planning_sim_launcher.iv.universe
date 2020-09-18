When the all success end conditions become OK, the simulation was judged as succeeded.
If one of the failure end conditions become OK, the simulation was judged as failure.

Now we support Conditions below

# Supported Conditions
## ReachPositionCondition
Check ego vehicle reached the target pose or not

### Example

```yaml
Type: "ReachPositionCondition"
Name: "EgoReachGoal"
Keep: True
Pose:
    Position:
    X: 10
    Y: 0
    Z: 0
    Orientation:
    X: 0.0
    Y: 0.0
    Z: 0.0
    W: 1.0
    FrameId: "initial_pose"
Tolerance : 0.5
```

### Fields

all fields are required.  

1. Type : Type should be "ReachPositionCondition"  
2. Name : Name of the Condition  
3. Pose : goal pose  
4. Keep : if true, this condition always true when the Ego Vehicle leave the goal pose  
5. Torelance : torelance of the goal pose 

## CollisionByEntityCondition
Check ego vehicle collide to the target entity or not

### Example

```yaml
Type: "CollisionByEntityCondition"
Name: "EgoCollideToObjCar1"
TargetEntity: ObjCar1
```

### Fields

all fields are required.  

1. Type: Type Should be "CollisionByEntityCondition"
2. Name: Name of the Condition 
3. TargetEntity: Name of the target Entity