# process_manager
This package manage system processes with ROS messages

## Commands

| Commands |     Process      |
|:--------:|:----------------:|
|    0     |   Shutdown PC    | 
|    1     |    Reboot PC     |
|    2     |  Start Autoware  |
|    3     | Restart Autoware |
|    4     |  Kill Autoware   |

## Diagnostics


## Inputs / Outputs

### Input

| Name                     | Type             | Description                |
|--------------------------|------------------|----------------------------|
| `/ui/ui_process_command` | `std_msgs/UInt8` | Input for process commands |

### Output

| Name                        | Type           | Description |
|-----------------------------| ---------------|-------------|
| `/ui/ui_process_diagnostic` |`std_msgs/UInt8`| WIP         |
