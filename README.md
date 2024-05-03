# dexhand_manus
ROS 2 Node for Dexhand to Manus VR Glove Integration With Examples


## Function

The `dexhand_manus` node is a bridge between the `manus_ros2` which publishes Manus VR Glove events as ROS 2 messges and DexHand hardware and software simulations. 

The `manus_ros2` node broadcasts ROS 2 PoseArray messages named `manus_left` and `manus_right` which contain position and rotation information from the Manus VR Gloves.

This node, `dexhand_manus` receives those messages and converts them into joint angles used by the DexHand hardware and simulation packages, republishing the data as angles in a ROS 2 JointState message.

### Left and Right Gloves

At the moment, the package expects that you will only use one glove at a time, as that's our currently supported use case. Either left or right may be used. If you use both at the same time, the data will conflict as both are broadcast to the same output JointState messsage. We'll revise this in the future when we support multiple hands.


## Inputs

The `dexhand_manus` node takes the following inputs:

- `manus_left`: Hand pose data for the left glove, as broadcast by the `manus_ros2` node
- `manus_right` : Hand pose data for the right glove, as broadcast by the `manus_ros2` node
- `dexhand_manus_cmd`: String message for controlling the hand. Currently, only `reset_origin` is supported which resets the zero pose of the hand.

## Outputs

The `dexhand_manus` node provides the following outputs:

- `joint_states`: Hand poses formatted to match the DOF's on the DexHand. This should be compatible with both hardware and simulation subscribers from the other packages in the `dexhand_ros2_meta` package collection.

## Usage

This package is designed to work in conjunction with the following packages:

- **dexhand_ros2_meta** - The metapackage containing the DexHand ROS 2 nodes for description, simulations, etc. 
- **manus_ros2** - ROS 2 package for reading data from the Manus SDK and converting the poses into ROS 2 messages.
- **dexhand_manus_gui** - A simple control panel for sending messages to this node (currently, just a reset pose button)

To use this GUI, follow these steps:

1. Clone the repository to your dexhand ROS 2 workspace (typically created via the dexhand_ros2_meta package):

    ```bash
    cd <Your dexhand workspace folder>/src
    git clone https://github.com/iotdesignshop/dexhand_manus.git
    ```

2. Build the ROS 2 workspace:

    ```bash
    cd <Your dexhand workspace folder>
    colcon build
    ```

3. Source the ROS 2 environment:

    ```bash
    source install/setup.bash
    ```

4. Launch the Manus/DexHand simulation launch file

    ```bash
    ros2 launch dexhand_manus simulation.launch.py
    ```

    This will start the DexHand and Manus nodes required to see a simulated version of the DexHand controlled by Manus VR Gloves.

## Contributing

Contributions are welcome! If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.