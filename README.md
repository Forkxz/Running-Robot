# Running Robot 2020
[Official Rule for year 2020](http://www.running-robot.net/rules/7.html)

@[TOC]
</font>
<hr style=" border:solid; width:100px; height:1px;" color=#000000 size=1">

# 1. Video Stream
In `Video_stream.py` defined video stream.

This stream can load both from device id and url stream in the robot but they cannot be used in the same time.

I recommend to close the url stream service on the robot and load from the device id directly:
```python
    Chest = LoadStreams(0)
    Head = LoadStreams(2)
```
`Chest.imgs` will be the frame that you want
 
You can also view the video stream in the main function in `Video_stream.py`:

```python
if __name__ == '__main__':
    dataset = LoadStreams(2)

    while True:
        cv2.imshow('img0', dataset.imgs)
        if cv2.waitKey(1) == ord('q'):  # q to quit
            raise StopIteration
```

<hr style=" border:solid; width:100px; height:1px;" color=#000000 size=1">

# 2. Mission State Definition
We defined such missions in this track:
```python
event_dist = {
        1: "holeg",
        2: "landmine",
        3: "flap",
        4: "turn",
        5: "door",
        6: "bridge",
        7: "ball",
        8: "step",
        9: "holeb"}
```
If the track has been changed in later years of competetion, change the `event_dist` in the file `State_machine.py` to match the real situation.

<hr style=" border:solid; width:100px; height:1px;" color=#000000 size=1">

# 3. Mission Function definition
In each mission state, call the needed mission function in that state in the top module `Top_model.py` such as:

```python
def landmine_state(Event_list):  #Check
    print('This is landmine_state')
    Back_to_center(Chest)
    landmine_lyu(Chest)
    return ("Trans",Event_list)
```
The mission function should be defined as:
```python
def Mission (Input_img, Other paramters you need):
    """
	Please describe the function first
	Usually, do not return anything in this function    
    """	
    while True:
    	# This is a loop for continuous recognition and move decision
```

Design Suggestions:
:  1. Load img stream in each mission function
:  2. Make move decision in each mission function
:  3. Add necessary sleep time or judgment to make sure the robot has finished current movment when next loop starts

<hr style=" border:solid; width:100px; height:1px;" color=#000000 size=1">

# 4. Demo & Test
## 1. Test
For test, you can record videos of the track and then use [OBS virtue camera](https://blog.csdn.net/qq_19338977/article/details/105395737) to replay the video as test.


Then change the import:
```python
#from CMDcontrol import CMD_transfer, action_list
#from CMDcontrol import action_append
from Robot_control import action_append,action_list
```
`Robot_control ` will print the actions out for test.

## 2. Mission order
We use the most advanced human intelligence method to recognize the mission order.

Change `event_order = [9,2,3,5,6,7,8]` with the real order on the track before you start the competition.

## 3. Color dictionary
Many visual methods need a proper color dict, most of them are stored in the file: `Color_define.py` 

Adjust the colors based on the real situation of the track.

> `Test (1).py` is another method but not well organized, you can also try that.

# Good luck
