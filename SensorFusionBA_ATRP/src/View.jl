# This file is mostly for the GUI and the controls desponsible.

using GLFW
using CImGui
using ImPlot
using ModernGL
using CSyntax
using CImGui: ImVec2

# Status Text for connection Window
connectStatus = ""

# time since last frame / data
deltaTime = 0.0

include("Client.jl")
include("InputHandler.jl")

# asset paths
const robotSource = GLTF.load("assets/robot.gltf")
const robotData = [read("assets/"*b.uri) for b in robotSource.buffers]

const plateSource = GLTF.load("assets/plate.gltf")
const plateData = [read("assets/"*b.uri) for b in plateSource.buffers]

const vertShaderScript = read("shader/shader.vert", String)
const fragShaderScript = read("shader/shader.frag", String)


export setUpWindow
"""
Set up a GLFW window, callbacks and render context.

# Arguments
- `size::Tuple{Integer, Integer}`: Size of the window.
- `title::String`: The title of the window.
"""
function setUpWindow(size::Tuple{Integer, Integer}, title::String)
    window = GLFW.CreateWindow(size[1], size[2], title)    
    GLFW.MakeContextCurrent(window)

    ctx = CImGui.CreateContext()

    # Create ImPlot context
    ctxp = ImPlot.CreateContext()
    ImPlot.SetImGuiContext(ctx)

    # Load fonts and select style....
    CImGui.StyleColorsDark()

    CImGui.ImGui_ImplGlfw_InitForOpenGL(window, true)
    CImGui.ImGui_ImplOpenGL3_Init(410) # GLSL Version

    GLFW.SetWindowCloseCallback(window, (_) -> onWindowClose())
    GLFW.SetMouseButtonCallback(window, (_, button, action, mods) -> onMouseButton(button, action))
    # adjust glViewport when resizing
    GLFW.SetWindowSizeCallback(window, (window, width, height) -> onWindowResize(width, height))

    #enable depth test 
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    program = createShaders()

    GC.gc()

    return window, ctx, program
end

"""
Create shader program.
"""
function createShaders()
    # compile shaders
    vertShader = glCreateShader(GL_VERTEX_SHADER)
    glShaderSource(vertShader, 1, Ptr{GLchar}[pointer(vertShaderScript)], C_NULL)
    glCompileShader(vertShader)
    fragShader = glCreateShader(GL_FRAGMENT_SHADER)
    glShaderSource(fragShader, 1, Ptr{GLchar}[pointer(fragShaderScript)], C_NULL)
    glCompileShader(fragShader)

    # create and link shader program
    program = glCreateProgram()
    glAttachShader(program, vertShader)
    glAttachShader(program, fragShader)
    glLinkProgram(program)

    # enable face culling
    glEnable(GL_CULL_FACE)
    glCullFace(GL_FRONT)
    glFrontFace(GL_CW)

    # set background color to gray
    glClearColor(0.2, 0.2, 0.2, 1.0)    

    return program
end

"""
Small helper function that displays a text when hovered over '(?)' symbol. 

# Arguments 
- `description::String`: The text to display. 
"""
function ShowHelpMarker(description)
    CImGui.TextDisabled("(?)")
    if CImGui.IsItemHovered()
        CImGui.BeginTooltip()
        CImGui.PushTextWrapPos(CImGui.GetFontSize() * 35.0)
        CImGui.TextUnformatted(description)
        CImGui.PopTextWrapPos()
        CImGui.EndTooltip()
    end
end

"""
Displays the helper window. 
"""
function handleHelperWidow()
    CImGui.SetNextWindowPos((0, 20))
    CImGui.Begin("Help", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize)
    CImGui.SetWindowFontScale(globalFontScale)
    CImGui.ShowUserGuide()
    CImGui.Text("Tip: Double Click on Plots to recenter")
    CImGui.Text("Robot Control:")
    CImGui.Text("
    \"W\" - Accelerate Forward \n
    \"S\" - Accelerate Backward \n 
    \"A\" - Increase Steering Left \n
    \"D\" - Increase Steering Right \n
    \"SPACE\" - Stop Motor \n
    \"Shift\" - Increase Max Speed \n
    \"Crtl\" - Decrease Max Speed
    ")
    CImGui.End()
end

"""
Displays the connect to jetson window.

# Arguments
- Requires ipData and portData as references. 
"""
function handleConnectWindow(ipData, portData)
    # Create a window
    CImGui.Begin("Connect to Jetson", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize | CImGui.ImGuiWindowFlags_NoCollapse)
    CImGui.SetWindowFontScale(globalFontScale)

    CImGui.Text("Please Enter the IP Adress and Port for the Jetson")

    CImGui.Text("Enter IP:")
    CImGui.SameLine()
    CImGui.InputText("", ipData, length(ipData), CImGui.ImGuiInputTextFlags_EnterReturnsTrue) && connectButtonPress(ipData, portData)         
    CImGui.Text("Enter Port:")
    CImGui.SameLine()
    CImGui.InputText(" ", portData, length(portData), CImGui.ImGuiInputTextFlags_EnterReturnsTrue) && connectButtonPress(ipData, portData)                        
    CImGui.Button(connected == false ? "Connect" : "Disconnect") && connectButtonPress(ipData, portData)
    CImGui.Text(connectStatus)

    CImGui.End()
end

"""
This function shows the automatic inputs dialog.
"""
function handleAutomaticInputWindow()
    CImGui.Begin("Automatic Inputs", C_NULL, CImGui.ImGuiWindowFlags_AlwaysAutoResize)

    @cstatic itemL=Cint(3) itemS=Cint(2) itemA=Cint(2) time=Cdouble(1.0) begin
        @c CImGui.Combo("Select longitudinal Input", &itemL, "Forward\0Backward\0Stop\0Nothing")
        @c CImGui.Combo("Select Steering Input", &itemS, "Left\0Right\0Straight")
        @c CImGui.Combo("Acceleration", &itemA, "Accelerate\0Deccelerate\0Keep")
        @c CImGui.InputDouble("Enter Time of Input [s]", &time)

        CImGui.Button("Add") && push!(automaticInput, (createCommand(itemL, itemS, itemA), time))
        CImGui.SameLine()    
        if !isempty(automaticInput) 
            CImGui.Button("Delete") && pop!(automaticInput) 
            CImGui.SameLine()

            # Confirmation button (toggles sending of inputs)
            if CImGui.Button(useAutomaticInput ? "Stop Sending!" : "Send to Robot!") 
                global useAutomaticInput = !useAutomaticInput
                global automaticInputIndex = 1
            end
        end
        CImGui.SameLine()
        ShowHelpMarker("Add Input to List of automatic inputs. Or delete last input")
    end 

    # Display of the list of automatic inputs
    CImGui.Text("Input List:")
    for s ∈ automaticInput
        CImGui.Text(s[1]*" for $(s[2])s.")
    end

    CImGui.End()
end

"""
Plot the positional data received from the AT-RP.
Has to be called inside the render loop.

# Arguments 
- `rectSize::Tuple{Integer, Integer}`: The size of the rectangle to draw position on.
- `posData::StructVector{PositionalData}`: The positional data from the atrp to plot.
- `windowName::String`: The name of the window.
- `settings::PredictionSettings`: The parameters that control the estimation.
"""
function plotData(rectSize::Tuple{Integer, Integer}, posData::StructVector{PositionalData}, windowName::String)
    CImGui.SetNextWindowSizeConstraints(rectSize, (rectSize[1], windowSize[2]))
    CImGui.Begin(windowName, C_NULL, CImGui.ImGuiWindowFlags_AlwaysVerticalScrollbar)    
    CImGui.SetWindowFontScale(globalFontScale)

    cameraPosMatrix = reduce(vcat, transpose.(posData.cameraPos))

    if CImGui.CollapsingHeader("Camera Position")
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(cameraPosMatrix), maximum(cameraPosMatrix))
        if ImPlot.BeginPlot("Relative Camera Position", "Data Point", "Distance [m]")            
            yValues = float.(cameraPosMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(cameraPosMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(cameraPosMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Camera Orientation")
        cameraOriMatrix = reduce(vcat, transpose.(posData.cameraOri))
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(cameraOriMatrix), maximum(cameraOriMatrix))
        if ImPlot.BeginPlot("Relative Camera Orientation", "Data Point", "Degrees [°]") 
            eulerAngles = Matrix{Float32}(undef, 3, 0)
            for i ∈ 1:size(cameraOriMatrix)[1]
                eulerAngles = hcat(eulerAngles, convertQuaternionToEuler([cameraOriMatrix[i, 1], cameraOriMatrix[i, 2], cameraOriMatrix[i, 3], cameraOriMatrix[i, 4]]))
            end
            yValues = float.(eulerAngles[2, :]) 
            ImPlot.PlotLine("Yaw", yValues, size(yValues, 1))
            yValues = float.(eulerAngles[1, :]) 
            ImPlot.PlotLine("Pitch", yValues, size(yValues, 1))
            yValues = float.(eulerAngles[3, :]) 
            ImPlot.PlotLine("Roll", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Camera Confidence")
        ImPlot.SetNextPlotLimits(0, length(posData), 0, 100)
        if ImPlot.BeginPlot("Confidence Value for Camera", "Data Point", "Percent [%]")
            values = float.(posData.cameraConfidence) 
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Camera Position Change")
        camPosChange = float.(cameraPosMatrix[:, 4])
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(camPosChange), maximum(camPosChange))
        if ImPlot.BeginPlot("Positional Change", "Data Point", "Absolute Change")
            ImPlot.PlotLine("", camPosChange, size(camPosChange, 1))
            ImPlot.EndPlot()
        end
    end
    if CImGui.CollapsingHeader("Steering Angle")
        ImPlot.SetNextPlotLimits(0, length(posData), -14, 14)
        if ImPlot.BeginPlot("Steering Angle", "Data Point", "Angle [°]")
            values = Int64.(posData.steerAngle)  
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Steering Angle (Sensor)")
        ImPlot.SetNextPlotLimits(0, length(posData), -100, 100)
        if ImPlot.BeginPlot("Steering Angle (Sensor)", "Data Point", "Steering [%]")
            values = Int64.(posData.sensorAngle)  
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Max Speed")
        ImPlot.SetNextPlotLimits(0, length(posData), 19, 40)
        if ImPlot.BeginPlot("Max Speed", "Data Point", "Max Speed [PWM - Duty Cycle]")
            values = float.(posData.maxSpeed)  
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end
    
    if CImGui.CollapsingHeader("Speed")
        values = float.(posData.sensorSpeed)
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(values), maximum(values))
        if ImPlot.BeginPlot("Speed", "Data Point", "Speed [m/s]")             
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Magnetometer")
        # Convert vector of vectors to matrix:
        imuMagMatrix = reduce(vcat, transpose.(posData.imuMag))
        ImPlot.SetNextPlotLimits(0, length(posData), -1, 1)
        if ImPlot.BeginPlot("Magnetic Field", "Data Point", "Field Strength [G]")
            yValues = float.(imuMagMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuMagMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuMagMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Angular Velocity")
        # Convert vector of vectors to matrix:
        imuGyroMatrix = reduce(vcat, transpose.(posData.imuGyro))
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(imuGyroMatrix), maximum(imuGyroMatrix))
        if ImPlot.BeginPlot("Angular Velocity", "Data Point", "Distance [°/s]")            
            yValues = float.(imuGyroMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuGyroMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuGyroMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end 
    end

    if CImGui.CollapsingHeader("Acceleration")
        # Convert vector of vectors to matrix:
        imuAccMatrix = reduce(vcat, transpose.(posData.imuAcc))
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(imuAccMatrix), maximum(imuAccMatrix))
        if ImPlot.BeginPlot("Acceleration", "Data Point", "Distance [g]")            
            yValues = float.(imuAccMatrix[:, 1]) 
            ImPlot.PlotLine("x", yValues, size(yValues, 1))
            yValues = float.(imuAccMatrix[:, 2]) 
            ImPlot.PlotLine("y", yValues, size(yValues, 1))
            yValues = float.(imuAccMatrix[:, 3]) 
            ImPlot.PlotLine("z", yValues, size(yValues, 1))
            ImPlot.EndPlot()
        end
    end

    if CImGui.CollapsingHeader("Delta Time")
        values = float.(posData.deltaTime)
        ImPlot.SetNextPlotLimits(0, length(posData), minimum(values), maximum(values))
        if ImPlot.BeginPlot("Delta Time", "Data Point", "dt [s]")             
            ImPlot.PlotLine("", values, size(values, 1))
            ImPlot.EndPlot()
        end
    end
end

# Display of framerate and calculation of delta time
let (previousTime, previousTimeCounting) = (time(), time())
    frame = 0
    global function updateFPS(window::GLFW.Window)
        currentTime = time()
        countingTime = currentTime - previousTimeCounting
        global deltaTime = currentTime - previousTime

        # update display every 0.25sec
        if countingTime > 0.25
            previousTimeCounting = currentTime
            fps = frame / countingTime
            GLFW.SetWindowTitle(window, "AT-RP Controller | FPS: $fps | dt: $deltaTime")
            frame = 0
        end
        previousTime = currentTime
        frame += 1
    end
end

function onWindowClose()
    @info "Program terminated."
end


