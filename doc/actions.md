Currently, we only support initial action under the Story/Init/Entity/(EntityName)/Actions

Now we support Actions below

# supported Actions

## FollowRoute

Follow Route Action enables Ego vehicle follow the target waypoint

```yaml
Actions:
    - Type: FollowRoute
    Params:
        GoalPose:
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
```

### Fields

all fields are required.  

1. Type : Type should be "ReachPositionCondition"  
2. GoalPose : goal pose of the ego vehicle