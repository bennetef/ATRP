# This file implements functions that are used to handle data.

# The first value of the magnetometer
firstMagValue = -1

export extractData
"""
This method extracts the data send by the AT-RP and 
stores them into the PositionalData datatype.

# Arguments
- `data::String`: The data as a string with structure: \n
<repetition of command>-<status>-<speed value>-<steering angle>-<detected speed in m/s>-<camera vector>-<imu data>-<gps data>.

# Returns 
- `PositionalData`: All the positional data combined in one datatype.
"""
function extractData(data::String)
    splitted = split(data, "|")

    # if data is corrupted
    if !(length(splitted) == 12)
        println(splitted)
        println("Length was not correct: " * string(length(splitted)))
        return
    end

    posData = PositionalData()
    posData.command = splitted[1] == "_nothing" ? String("") : splitted[1]
    posData.maxSpeed = parse(Float32, splitted[2])
    posData.steerAngle = parse(Int, splitted[3])
    posData.sensorAngle = parse(Int, splitted[4])
    posData.sensorSpeed = parse(Float32, splitted[5])
    posData.cameraPos = parse.(Float32, split(chop(splitted[6]; head=1, tail=1), ','))
    posData.cameraOri = parse.(Float32, split(chop(splitted[7]; head=1, tail=1), ','))
    posData.imuAcc = parse.(Float32, split(chop(splitted[9]; head=1, tail=1), ','))
    posData.imuGyro = parse.(Float32, split(chop(splitted[10]; head=1, tail=1), ','))
    posData.imuMag = parse.(Float32, split(chop(splitted[11]; head=1, tail=1), ','))
    posData.deltaTime = deltaTime
    posData.cameraConfidence = parse(Float32, splitted[8])
    posData.gpsPosition = parse.(Float32, split(chop(splitted[12]; head=1, tail=1), ','))

    return posData
end

"""
Converts a dictionary to PositionalData datatype.

# Arguments
- `dict::Dict`: The dict to convert
- `rotateCameraCoords::Bool`: Should the camera coordinates be rotated to match IMU data
- `flipCameraCoords::Bool`: Additional rotation of the camera data by 180 degrees
- `loadGPSData::Bool`: Should gps data be loaded, otherwise its `[0, 0]`
# Returns 
- `PositionalData`: All the positional data combined in one datatype
"""
function convertDictToPosData(dict::Dict, rotateCameraCoords::Bool, flipCameraCoords::Bool, loadGPSData::Bool)
    posData = PositionalData()
        
    posData.steerAngle = dict["steerAngle"] - 120
    posData.sensorAngle = dict["sensorAngle"]
    posData.maxSpeed = dict["maxSpeed"]
    posData.sensorSpeed = dict["sensorSpeed"]
    posData.imuMag = dict["imuMag"]
    camPos = dict["cameraPos"]
    camPos = [camPos[1], -camPos[3], camPos[2], camPos[4]]
    if rotateCameraCoords 
        firstMagValue == -1 && global firstMagValue = posData.imuMag
        camPos = transformCameraCoords(Float32.(camPos), convertMagToCompass(firstMagValue) + (flipCameraCoords ? π : 0)) 
    end
    posData.cameraPos = camPos
    posData.cameraOri = dict["cameraOri"]
    posData.imuGyro = deg2rad.(dict["imuGyro"])
    posData.imuAcc = dict["imuAcc"]    
    posData.deltaTime = dict["deltaTime"]
    posData.cameraConfidence = dict["cameraConfidence"] ./ 100
    posData.command = dict["command"]
    if loadGPSData
        posData.gpsPosition = dict["gpsPosition"]
    end

    return posData
end