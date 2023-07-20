
# RotateTurtleState and Userdata

A key part of FlexBE that extends the concept beyond pure state machines is `userdata` that can be passed from one state to another.

For example, the [Rotate Turtle State](flexbe_turtlesim_demo_flexbe_states/flexbe_turtlesim_demo_flexbe_states/rotate_turtle_state.py) uses
`userdata` to define the desired angle.

We will be our discussion with a simpler example, and then return to the specifics of the "Rotate" transition in `FlexBE Turtlesim Demonstration`.

----

### `Rotate Turtlesim Demonstration (TODO)`

Separate from the `FlexBE Turtlesim Demonstration (TODO)` behavior, we have provided a simpler `Rotate Turtlesim Demonstration (TODO)` behavior.
We will start by describing that first.

Each state can accept data according to specified `Input Keys`.
These key names can be remapped at the state level.

```Python
class RotateTurtleState(EventState):
    """
    ...
    Parameters
    -- timeout             Maximum time allowed (seconds)
    -- action_topic        Name of action to invoke

    Outputs
    <= rotation_complete   Only a few dishes have been cleaned.
    <= failed              Failed for some reason.
    <= canceled            User canceled before completion.
    <= timeout             The action has timed out.

    User data
    ># angle     float     Desired rotational angle in (degrees) (Input)
    #> duration  float     Amount time taken to complete rotation (seconds) (Output)

    """

    def __init__(self, timeout, action_topic="/turtle1/rotate_absolute"):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['rotation_complete', 'failed', 'canceled', 'timeout'],
                         input_keys=['angle'],
                         output_keys=['duration'])

```
The `RotateTurtleState` implementation specifies an input key called `angle` and an output key `duration`.  Internally, the state implementation will use `userdata.angle` to access the stored data
using the FlexBE core [`userdata.py` class](https://github.com/flexbe/flexbe_behavior_engine/flexbe_core/flexbe_core/userdata.py) that
extends the capabilities of the basic `dict` object.

<img src="img/key_remap_edit.png" alt="Remapping keys." width="300">
<img src="img/data_flow_graph.png" alt="Data flowing into Rotation State." width="300">
<img src="img/sm_userdata_dashboard.png" alt="Defining user data at the state machine level." width="300">

The `Data Flow` view allows the behavior designer to view how userdata
is passed through the state machine.  Once defined, a `userdata` value
persists for the life of the state machine, but is only accessible
within states that access that mapped key.

In the `Rotate Turtlesim Demonstration (TODO)` behavior, we define
the `userdata` at the FlexBE UI Dashboard.  In this case, the desired
angle in degrees.

Now when the state is executed the turtle will rotate to the key value that was defined after converting to `radians` as required by the
`RotateAbsolute` action provided by `Turtlesim`.

> Note: Normally, we suggest you stick to a consistent convention
> for passing data, and ROS uses `radians` by convention.  
> Here, we chose `degrees` here to illustrate data conversions
> and for operator convenience at the UI.

The `userdata` is passed to the standard `on_enter`, `execute`, and `on_exit` methods of each FlexBE state.  Here we validate the data and
use to create a `Goal` request for the `RotateAbsolute` action.

```python
def on_enter(self, userdata):

    # make sure to reset the error state since a previous state execution might have failed
    self._error = False
    self._return = None

    # Recording the start time to set rotation duration output
    self._start_time = self._node.get_clock().now()

    goal = RotateAbsolute.Goal()

    if isinstance(userdata.angle, (float, int)):
        goal.theta = (userdata.angle * math.pi) / 180  # convert to radians
    else:
        self._error = True
        Logger.logwarn("Input is %s. Expects an int or a float.", type(userdata.angle).__name__)

    # Send the goal.
    try:
        self._client.send_goal(self._topic, goal)
    except Exception as e:
        # Since a state failure not necessarily causes a behavior failure,
        # it is recommended to only print warnings, not errors.
        # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
        Logger.logwarn('Failed to send the RotateAbsolute command:\n%s' % str(e))
        self._error = True
```

Then in the `execute` method we monitor for the successful result, and
set the outgoing `userdata.duration` value.  This will be stored in the
global `userdata` instance according to the remapping defined in the state edit window above.

```python
def execute(self, userdata):
    # While this state is active, check if the action has been finished and evaluate the result.

    # Check if the client failed to send the goal.
    if self._error:
        return 'failed'

    if self._return is not None:
        # Return prior outcome in case transition is blocked by autonomy level
        return self._return

    # Check if the action has been finished
    if self._client.has_result(self._topic):
        _ = self._client.get_result(self._topic)  # The delta result value is not useful here
        userdata.duration = self._node.get_clock().now() - self._start_time
        Logger.loginfo('Rotation complete')
        self._return = 'rotation_complete'
        return self._return

    if self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds > self._timeout.nanoseconds:
        # Checking for timeout after we check for goal response
        self._return = 'timeout'
        return 'timeout'

    # If the action has not yet finished, no outcome will be returned and the state stays active.
    return None
```

To demonstrate "collaborative autonomy" aspects of FlexBE,
the next section discusses the "Rotate" transition from the `FlexBE Turtlesim Demonstration`.

----

## "Rotate" - Collaborative Autonomy with Operator Input

The FlexBE Behavior Engine](https://github.com/flexbe/flexbe_behavior_engine) provides an [`InputState`](https://github.com/flexbe/flexbe_behavior_engine/flexbe_states/flexbe_states/input_state.py)
that accepts operator data via a [`BehaviorInput` action](https://github.com/flexbe/flexbe_behavior_engine/flexbe_msgs/action/BehaviorInput.action) interface.

Additionally, FlexBE provides a simple action server with PyQt based UI window as part of the [`flexbe_input` package](https://github.com/flexbe/flexbe_behavior_engine/flexbe_input).

`ros2 run flexbe_input input_action_server`

When the FlexBE onboard `InputState` requests data of a given type, the
UI window will open, prompt the user with the provided text, and wait for user input.  After the user presses `Enter/Return` or clicks the `Submit` button, the data is serialized and sent back to the `InputState` as a string of bytes data as part of the action result.

In the `FlexBE TurtleSim Demonstration (TODO)` statemachine,
 the container labeled `Rotate` is itself a simple state machine;
 that is, we have a Hierarchical Finite State Machine (HFSM).

 <img src="img/rotate_sm_view.png" alt="Rotate sub-state machine." width="350">
<img src="img/input_state_config.png" alt="Configuration of input state." width="350">
<img src="img/input_ui.png" alt="Configuration of input state." width="350">

In the `InputState` configuration, we specify result type 1 (`BehaviorInput.Goal.RESULT_FLOAT`) to request a single number from the user (Note, we accept integer values without decimals as well.)

<img src="img/input_state_demo.png" alt="Using input state to provide data to turtle rotation state." width="350">
