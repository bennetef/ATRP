# This file contains functions to handle input of the user

include("DataExtractor.jl")

# Does the user want to record data currently
recordData = false
# Vector containing automatic inputs as commands and the duration
automaticInput = Vector{Tuple{String, Float64}}(undef, 0)
automaticInputIndex = 1 

"""
This function handles the key inputs and sends appropriate commands to robot. Also sensor data is received.

# Arguments
- `window::GLFW.Window`: The openGL window context
# Optional Arguments
- `automaticInput::Union{String, Nothing}=nothing`: The command for the robot to execute
# Returns 
- `PositionalData`: The sensor data saved in one datatype
"""
function commandLoop(window::GLFW.Window; automaticInput::Union{String, Nothing}=nothing)
    # Key Inputs:
    #a = 65, d=68, w=87, s=83, shift =340,ctrl = 341, space=32, esc = 256
    if !connected
        # if not connected to robot return immediately
        return 0
    end
    
    # create command
    command = ""

    # Escape
    if GLFW.GetKey(window, GLFW.KEY_ESCAPE)
        @info "Sending Escape Command and Disconnect."
        answer = sendAndRecvData("escape")

        if answer == "closing"
            global connected = false
            global connectStatus = ""
            pysocket.close()
            return 0
        end
    end

    # Left and right
    if GLFW.GetKey(window, GLFW.KEY_A)
        command *= "_left"
    elseif GLFW.GetKey(window, GLFW.KEY_D)
        command *= "_right"
    end

    # Accelerate and Deccelerate
    if GLFW.GetKey(window, GLFW.KEY_LEFT_SHIFT)
        command *= "_accelerate"
    elseif GLFW.GetKey(window, GLFW.KEY_LEFT_CONTROL)
        command *= "_deccelerate"
    end

    # Forward, Backwards and Stop
    if GLFW.GetKey(window, GLFW.KEY_W)
        command *= "_forward"
    elseif GLFW.GetKey(window, GLFW.KEY_S)
        command *= "_backward"
    elseif GLFW.GetKey(window, GLFW.KEY_SPACE)
        command *= "_stop"
    else
        command *= "_nothing"
    end    

    try
        # send command to robot 
        answer = sendAndRecvData(isnothing(automaticInput) ? command : automaticInput)

        # return the sensor data as positional data datatype
        return extractData(answer)
    catch error
        @warn "Error was caught: " * string(error)
    end    
end

"""
This method creates a command for the robot using specified items in the automatic input dialog.

# Arguments
- `itemL::Int32`: Index of comboBox specifying longitudinal inputs 
- `itemS::Int32`: Index of comboBox specifying steering inputs 
- `itemS::Int32`: Index of comboBox specifying acceleration inputs 
# Returns
- `String`: The command as can be interpreted by the robot
"""
function createCommand(itemL::Int32, itemS::Int32, itemA::Int32)
    command = ""

    # Left and right
    if itemS == 0
        command *= "_left"
    elseif itemS == 1
        command *= "_right"
    end

    # Accelerate and Deccelerate
    if itemA == 0
        command *= "_accelerate"
    elseif itemA == 1
        command *= "_deccelerate"
    end

    # Forward, Backwards and Stop
    if itemL == 0
        command *= "_forward"
    elseif itemL == 1
        command *= "_backward"
    elseif itemL == 2
        command *= "_stop"
    else
        command *= "_nothing"
    end

    return command
end

"""
This function is called when pressing a mouse button. Sets according global variables.
"""
function onMouseButton(button, action)
    if button == GLFW.MOUSE_BUTTON_LEFT && action == GLFW.PRESS
        global isLeftMouseButtonDown = true       
    elseif  button == GLFW.MOUSE_BUTTON_LEFT && action == GLFW.RELEASE
        global isLeftMouseButtonDown = false
    end
    
    if button == GLFW.MOUSE_BUTTON_RIGHT && action == GLFW.PRESS
        global isRightMouseButtonDown = true       
    elseif  button == GLFW.MOUSE_BUTTON_RIGHT && action == GLFW.RELEASE
        global isRightMouseButtonDown = false
    end
end

function onWindowResize(width, height)
    glViewport(0, 0, width, height)
end

"""
This function is called when pressing the connect button in the connection dialog. The function checks
    the correctness of inputs and tries to connect to specified port.

# Arguments 
- `ipData::String`: The string input as ip 
- `portData::String`: The string input as port
"""
function connectButtonPress(ipData::String, portData::String)
    if connected
        disconnect()
        return
    end

    ip = ""
    port = ""

    for char in ipData
        if char === '.' || isdigit(char)
            ip = ip * char
        end
    end

    for char in portData
        if isdigit(char)
            port = port * char
        end
    end

    global connectStatus = "Trying to connect to: " * string(ip) * " on " * string(port)
    global connectStatus = checkConnection(ip, port)
end