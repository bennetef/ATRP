# This file contains descriptions of several structs.

export PositionalData
"""
Struct to store the positional data acquired by the AT-RP.

# Fields 
- `steerAngle::Integer`: The steering angle as set in control of the robot. 
- `sensorAngle::Integer`: The steer angle as reported by the steer sensor. 
- `maxSpeed::Float32`: Max speed of the robt (19-40) as arbitrary speed value. 
- `sensorSpeed::Float32`: The speed measured by the wheel encoder. 
- `cameraPos::Vector{Float32}`: The position as estimated by the camera.
- `cameraOri::Vector{Float32}`: The camera orientation as quaternion.
- `imuGyro::Vector{Float32}`: The angular velocity in all 3 axis of gyroscope.
- `imuAcc::Vector{Float32}`: The acceleration in all 3 axis of accelerometer.
- `imuMag::Vector{Float32}`: The magnetic field strength in all 3 axis of magnetometer.
- `deltaTime::Float32`: The time passed since last state update. 
- `cameraConfidence::Float32`: The camera confidence value as reported by VO-System.
- `command::String`: The command send to the robot. 
- `gpsPosition::Vector{Float32}`: The gps location if such a device is connected.
"""
mutable struct PositionalData
    steerAngle::Integer
    sensorAngle::Integer
    maxSpeed::Float32
    sensorSpeed::Float32
    cameraPos::Vector{Float32}
    cameraOri::Vector{Float32}    
    imuGyro::Vector{Float32}
    imuAcc::Vector{Float32}
    imuMag::Vector{Float32}
    deltaTime::Float32
    cameraConfidence::Float32
    command::String
    gpsPosition::Vector{Float32}

    PositionalData() = new(0, 0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 0.0, String(""), [0.0, 0.0])

    PositionalData(p::PositionalData) = new(
        p.steerAngle, p.sensorAngle, p.maxSpeed, p.sensorSpeed, p.cameraPos, p.cameraOri, p.imuGyro, p.imuAcc, p.imuMag, p.deltaTime, p.cameraConfidence, p.command, p.gpsPosition
    )
end

"""
This struct describes the camera model used in the rendering process.

# Fields
- `speed::Float32`: Movement speed of the camera.
- `position::Vector{GLfloat}`: The position of the camera in global coordinates.
- `target::Vector{Float32}`: The position the camera looks at in global coordinates.
- `up::Vector{Float32}`: The vector pointing up in the camera frame. 
- `scrollSpeed::Float32`: Factor of zoom speed. 
- `aspectRatio::Float32`: The aspect ratio that the camera should display.
- `fov::Int`: The field of view of the camera. 
"""
mutable struct Camera
    speed::Float32
    position::Vector{GLfloat}
    target::Vector{Float32}
    up::Vector{Float32}
    scrollSpeed::Float32
    aspectRatio::Float32
    fov::Int

    function Camera()
        new(0.2, GLfloat[0.0, 0.0, 2.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 10.0, 16/9, 30)
    end
end