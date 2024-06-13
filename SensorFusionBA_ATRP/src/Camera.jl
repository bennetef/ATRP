# This file contains methods for handling with the rendering camera

# distance to front and back plane
const znear = 0.1
const zfar = 100

"""
This function returns the projection matrix by given camera properties.

# Arguments
- `cam::Camera`: The camera object.
# Returns
- `Matrix{GLfloat}`: The 4x4 projection matrix.
"""
function getProjectionMatrix(cam::Camera)
    range = tan(0.5*cam.fov) * znear
    return GLfloat[(2.0*znear / (range*cam.aspectRatio + range*cam.aspectRatio)) 0.0 0.0 0.0;
                    0.0 (znear / range) 0.0 0.0;
                    0.0 0.0 (-(zfar+znear) / (zfar-znear)) (-(2.0*zfar*znear) / (zfar-znear));
                    0.0 0.0 -1.0 0.0]
end

"""
This method calculates the view matrix with given camera properties.

# Arguments
- `cam::Camera`: The camera object.
# Returns
- `Matrix{GLfloat}`: The 4x4 view matrix according to the transform of the camera.
"""
function getViewMatrix(cam::Camera)
    # Construct the coordinate frame of the camera 
    zAxis = normalize(cam.target - cam.position)
    xAxis = normalize(cross(zAxis, cam.up))
    yAxis = cross(xAxis, zAxis)

    zAxis = -1 * zAxis

    # Use coordinate axis to calculate view matrix
    viewMatrix = GLfloat[xAxis[1] xAxis[2] xAxis[3] -dot(xAxis, cam.position);
                  yAxis[1] yAxis[2] yAxis[3] -dot(yAxis, cam.position);
                  zAxis[1] zAxis[2] zAxis[3] -dot(zAxis, cam.position);
                  0.0 0.0 0.0 1.0]
    return viewMatrix
end

"""
This method returns the rotational matrix in homogonous coordinates. 

# Arguments
`degrees::Float64`: The degrees to rotate.
`axis::Vector{Float32}`: The axis to rotate around. 

# Returns
`Matrix{Float64}`: 4x4 rotational matrix that describes the specified rotational behaviour.
"""
function rotateAroundAxis(degrees::Float64, axis::Vector{Float32})
    ncos = 1 - cosd(degrees)
    sin = sind(degrees)
    cos = cosd(degrees)

    return [axis[1]^2*ncos+cos axis[1]*axis[2]*ncos-axis[3]*sin axis[1]*axis[3]*ncos+axis[2]*sin 0.0;
            axis[2]*axis[1]*ncos+axis[3]*sin axis[2]^2*ncos+cos axis[2]*axis[3]*ncos-axis[1]*sin 0.0;
            axis[1]*axis[3]*ncos-axis[2]*sin axis[2]*axis[3]*ncos+axis[1]*sin axis[3]^2*ncos+cos 0.0;
            0.0 0.0 0.0 1.0]
end

"""
Function to update the camera position according to mouse movements.

# Arguments
- `mousePos::Vector{Float64}`: The mouse position in pixel coordinates.
- `cam::Camera`: The camera object to influence.
"""
function checkCameraMovement(mousePos::Vector{Float64}, cam::Camera)
    # Only update mouse position if right mouse button is pressed
    if isRightMouseButtonDown
        # calculate the change in mouse position
        difVector = mousePos - oldMousePosition

        camX = cam.speed * difVector[1]
        camY = cam.speed * difVector[2]

        # calculate side axis
        sideAxis = cross(cam.up, cam.position);
        # create transformation matrix in homogonous coordinates
        transformMatrix = rotateAroundAxis(camX, cam.up) * rotateAroundAxis(camY, normalize!(sideAxis))

        # update camera position and orientation 
        cam.up = normalize!(deleteat!(transformMatrix * push!(cam.up, 1.0), 4))
	    cam.position = deleteat!(transformMatrix * push!(cam.position, 1.0), 4);
    end 

    # update previous mouse position
    global oldMousePosition = mousePos
end
