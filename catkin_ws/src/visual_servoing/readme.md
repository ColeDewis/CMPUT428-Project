# Visual Servoing
Contains a node that implements UVS directly with the kinova gen3 arm. Uses code from the `kortex_bringup` package to move the arm. Listens to topics `/image_error` and `/eef_pos` for information about the robot and sends movement based on that information.
