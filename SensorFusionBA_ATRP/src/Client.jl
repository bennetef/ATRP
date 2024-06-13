# This file implements functions that are used for TCP communication

#= Since the host program runs a python socket the PyCall package 
is used to simplify communication. =#
using PyCall 

# set up of variables and defining constants
pysockets = pyimport("socket")

# standard port and ip
export HOST, PORT 
const HOST = "10.42.0.1";
      PORT = 2222

export connected
connected = false

# Create a python socket
export pysocket
pysocket = pysockets.socket(pysockets.AF_INET, pysockets.SOCK_STREAM)
pysocket.settimeout(20)

export connectToJetson
"""
Connects to the host program run on the robot.

# Arguments
- `ip::String`: The IP-Address as string to connect to.
- `port::Integer`: The Port of the software to connect to.

# Returns
- `boolean`: True if connection to a valid controller was succesful.
"""
function connectToJetson(ip::String=HOST, port::Integer=PORT)    
    # Update connect status (displayed in GUI)
    global connectStatus = "Trying to connect to: " * string(ip) * " on " * string(port)
    global pysocket = pysockets.socket(pysockets.AF_INET, pysockets.SOCK_STREAM)

    try
        pysocket.settimeout(20)
        pysocket.connect((ip, port))
    catch error
        if isa(error, PyCall.PyError)
            @warn "Connection to Jetson Timed Out!"
        else
            print(error)
        end

        pysocket.close()
        return false
    end

    # Connected to something -----
    data = pysocket.recv(1024)

    pysocket.send(b"atsv_controller")

    return data == "atsv_brain_welcome"
end

export sendAndRecvData
"""
This function sends and receives data over the established tcp connection.

# Arguments 
- `data::String`: The data to send as String.

# Returns
- `String`: The data the sockets receives.
"""
function sendAndRecvData(data::String)
    pysocket.send(codeunits(data))

    return pysocket.recv(2048)
end

"""
This function tries to connect with given ip and port to the host.
Note that the arguments are both of type string.

# Arguments
- `ip::String`: The ipv4 address of the host
- `post::String`: The port of the host program.
"""
function checkConnection(ip::String, port::String)    
    try        
        if connectToJetson(ip, parse(Int64, port))
            global connected = true
            return "Connection established!"
        end
           
        return "Connection failed, try again!"
    catch error
        if isa(error, ArgumentError)
            # If no port or ip is given, the program tries to connect with standard settings 
            @warn "Port or IP incorrect \n Connection to standard port and IP"
            if connectToJetson()
                global connected = true
                return "Connection established!"
            end

            return "Connection failed, try again!"
        end

        return "Connection failed, try again!"
    end
end

"""
Disconnects from established connection and resets according settings.
"""
function disconnect()
    @info "Disconnecting..."

    try
        sendAndRecvData("escape")
    finally
        global connected = false
        global connectStatus = ""
        pysocket.close()
        GC.gc()

        # close appropriate windows
        global renderRobot = false
        global showDataPlots = false
        global showAutomaticInputWindow = false
        return
    end
end