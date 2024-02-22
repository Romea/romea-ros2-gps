# 1 Overview #

This package provides odometry plugin for vehicle localisation. It takes data coming from vehicle controller to convert them to a simple 2D twist measure with its covariance.

# 2 Node #

### 2.1 Subscribed Topics ###

- vehicle_controller/odom (nav_msgs::msg::Odometry)

  This topic is provided by standard vehicle controllers (diff_drive_controller,ackermann_steering_controller, four_wheel_steering_controller),it provides a lot informations like linear angular and speeds and dead reckoning pose but without consistent uncertainties informations.

- vehicle_controller/kinematic (romea_msgs::KinematicMeasuredStamped)

  This topic is provided by romea mobile base controllers. It's provides robot kinematic measure and its covariance computed in taking into account odometry data and sensors uncertainties. This topic should be preferred to the other because the uncertainties are reliable.

### 2.2 Published Topics ###

- twist (romea_localisation_msgs::ObservationTwist2DStamped)

### 2.3 Parameters ###

- odo_source (string, default: kinematic)

    This parameter is used to select odometry source:

    - odom : odometry data are coming from vehicle_controller/odom topic
    - kinematic : odometry data are coming from vehicle_controller/kinematic

- restamping (bool, default: false)

    If this parameter is set to true stamp of twist message is equal to computer current time else this stamp is equal odo stamp.  This paremeter will be used when odometry data are coming from a remote master.
