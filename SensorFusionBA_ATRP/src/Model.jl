# This file contains function and types to handle and describe 3D models

# Used for model loading
using GLTF

"""
This struct describes the transform of a 3D model. The properties of the transform can be given through 
    a transform matrix or by the 3 vectors describing position, rotation and scale.
"""
mutable struct Transform
    position::Vector{Float64}
    eulerRotation::Vector{Float64}
    scale::Vector{Float64}
    matrix::Union{Matrix{GLfloat}, Nothing}

    function Transform()
        new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0], nothing)
    end

    function Transform(position::Vector{Float64}, eulerRotation::Vector{Float64}, scale::Vector{Float64}; matrix::Union{Matrix{GLfloat}, Nothing}=nothing) 
        new(position, eulerRotation, scale, matrix)
    end
end

"""
This simple struct holds the basic properties of a material used in 3D graphics.
"""
mutable struct Material
    diffuseColor::Vector{Float32}
    specularColor::Vector{Float32}
    ka::Float32
    kd::Float32
    ks::Float32
    shininess::Float32
end

"""
This struct holds the information required to display a mesh in 3D graphics using openGL. 
# Fields
- `vao::UInt32`: The vertex array object that contains position of each vertex. This object stored in VRAM and is accessed through a pointer.
- `ebo::UInt32`: The element buffer object that contains the indice information of the triangles making up the mesh. This object stored in VRAM and is accessed through a pointer.
- `sizeOfIndices::Int64`: Number of indices this mesh contains.
- `transform::Transform`: The transform object describing the position, orientation and scale in 3D space. 
- `material::Material`: The material object describing the mesh's appearance.
"""
mutable struct Mesh
    vao::UInt32
    ebo::UInt32
    sizeOfIndices::Int64
    transform::Transform
    material::Material
end

"""
Struct that creates an abstract model consisting out of mutiple meshes.
"""
mutable struct Model
    meshes::Vector{Mesh}
    transform::Transform
end

"""
Transforms around a positional reference point. This is needed because the reference point of many
of the meshes in a given model are not at the reference point of the model. This might not be useful when trying 
to modify only one mesh in the model. For this it is necessary to know where meshes are located in the model.

# Arguments
- `transform::Transform`: The transform to manipulate
- `pointOfReference::Vector{Float64}`: The refernce point in 3D space to move the transform to
"""
function transformAroundReference(transform::Transform, pointOfReference::Vector{Float64})
    T = GLfloat[1.0 0.0 0.0 pointOfReference[1];
                0.0 1.0 0.0 pointOfReference[2];
                0.0 0.0 1.0 pointOfReference[3];
                0.0 0.0 0.0 1.0]
    
    T⁻¹ = GLfloat[1.0 0.0 0.0 -pointOfReference[1];
                  0.0 1.0 0.0 -pointOfReference[2];
                  0.0 0.0 1.0 -pointOfReference[3];
                  0.0 0.0 0.0 1.0]

    return T⁻¹ * transformToMatrix(transform) * T
end

"""
Converts transform object to homogonous transformation matrix.

# Arguments
- `t::Transform`: The transform object to convert.
# Returns
- `Matrix{GLfloat}`: The transform matrix resulting from the given transform object
"""
function transformToMatrix(t::Transform)
    xRotation = GLfloat[1.0 0.0 0.0 0.0;
                        0.0 cos(t.eulerRotation[1]) -sin(t.eulerRotation[1]) 0.0;
                        0.0 sin(t.eulerRotation[1]) cos(t.eulerRotation[1]) 0.0;
                        0.0 0.0 0.0 1.0]
    yRotation = GLfloat[cos(t.eulerRotation[2]) 0.0 sin(t.eulerRotation[2]) 0.0;
                        0.0 1.0 0.0 0.0;
                        -sin(t.eulerRotation[2]) 0.0 cos(t.eulerRotation[2]) 0.0;
                        0.0 0.0 0.0 1.0]
    zRotation = GLfloat[cos(t.eulerRotation[3]) -sin(t.eulerRotation[3]) 0.0 0.0;
                        sin(t.eulerRotation[3]) cos(t.eulerRotation[3]) 0.0 0.0;
                        0.0 0.0 1.0 0.0;
                        0.0 0.0 0.0 1.0]

    scaleAndPos = GLfloat[t.scale[1] 0.0 0.0 t.position[1];
                          0.0 t.scale[2] 0.0 t.position[2];
                          0.0 0.0 t.scale[3] t.position[3];
                          0.0 0.0 0.0 1.0]

    return xRotation * yRotation * zRotation * scaleAndPos
end

function convertQuaternionToEuler(q::Vector{Float32})
    x = q[1]
    y = q[2]
    z = q[3]
    w = q[4]

    return [atan(2*(w*x+y*z), 1-2*(x^2+y^2)); asin(2*(w*y-z*x)); atan(2*(w*z+x*y), 1-2*(y^2+z^2))]
end

"""
This method loads a model from gltf objects. Note that texture coords are not used here.

# Arguments 
- `model::GLTF.Model`: The basic structure object of GLTF data format
- `modelData::GLTF.ZVector`: The object containing the model information in bytes 

# Returns
- `Model`: The created model datatype 
"""
function loadGLTFModelInBuffers(model::GLTF.Object, modelData::GLTF.ZVector)
    newModel = Model(Vector{Mesh}(undef, 0), Transform())
    accessors = model.accessors

    # looop over all meshes in model 
    for mesh in model.meshes
        pos = accessors[mesh.primitives[0].attributes["POSITION"]]           
        normals = accessors[mesh.primitives[0].attributes["NORMAL"]]        
        indices = accessors[mesh.primitives[0].indices]   
        material = model.materials[mesh.primitives[0].material]   

        # load information for each mesh 
        mesh = loadGLTFMeshInBuffers(pos, normals, indices, material, model, modelData)
        push!(newModel.meshes, mesh)
    end

    return newModel
end

"""
Create a mesh using information from GLTF file. 

# Arguments 
- `pos::GLTF.Accessor`: An object that hold positional information of vertices in mesh 
- `normals::GLTF.Accessor`:  An object that hold normal vectors of vertices in mesh 
- `indices::GLTF.Accessor`: An object that hold indices of vertices for the triangles in mesh 
- `material::GLTF.Material`: Material object from GLTF package holding material information
- `model::GLTF.Object`: The basic structure object of GLTF data format
- `modelData::GLTF.ZVector`: The object containing the model information in bytes 

# Returns 
- `Mesh`: The created mesh datatype 
"""
function loadGLTFMeshInBuffers(pos::GLTF.Accessor, normals::GLTF.Accessor, indices::GLTF.Accessor, material::GLTF.Material, model::GLTF.Object, modelData::GLTF.ZVector)
    # Create Buffer views
    posBufferView = model.bufferViews[pos.bufferView]
    normalBufferView = model.bufferViews[normals.bufferView]
    idxBufferView = model.bufferViews[indices.bufferView]

    # create buffers located in the memory of graphic card
    # position
    posVBO = GLuint(0)
    @c glGenBuffers(1, &posVBO)
    glBindBuffer(posBufferView.target, posVBO)
    glBufferData(posBufferView.target, posBufferView.byteLength, C_NULL, GL_STATIC_DRAW)
    posData = modelData[posBufferView.buffer]
    @c glBufferSubData(posBufferView.target, 0, posBufferView.byteLength, &posData[posBufferView.byteOffset])
    # normals
    normalsVBO = GLuint(0)
    @c glGenBuffers(1, &normalsVBO)
    glBindBuffer(normalBufferView.target, normalsVBO)
    glBufferData(normalBufferView.target, normalBufferView.byteLength, C_NULL, GL_STATIC_DRAW)
    normalData = modelData[normalBufferView.buffer]
    @c glBufferSubData(normalBufferView.target, 0, normalBufferView.byteLength, &normalData[normalBufferView.byteOffset])
    # indices
    idxEBO = GLuint(0)
    @c glGenBuffers(1, &idxEBO)
    glBindBuffer(idxBufferView.target, idxEBO)
    glBufferData(idxBufferView.target, idxBufferView.byteLength, C_NULL, GL_STATIC_DRAW)
    idxData = modelData[idxBufferView.buffer]
    @c glBufferSubData(idxBufferView.target, 0, idxBufferView.byteLength, &idxData[idxBufferView.byteOffset + indices.byteOffset])

    # create VAO
    vao = GLuint(0)
    @c glGenVertexArrays(1, &vao)
    glBindVertexArray(vao)
    glBindBuffer(posBufferView.target, posVBO)
    glVertexAttribPointer(0, 3, pos.componentType, pos.normalized, posBufferView.byteStride, Ptr{Cvoid}(pos.byteOffset))
    glBindBuffer(normalBufferView.target, normalsVBO)
    glVertexAttribPointer(1, 3, normals.componentType, normals.normalized, normalBufferView.byteStride, Ptr{Cvoid}(normals.byteOffset))
    glEnableVertexAttribArray(0)
    glEnableVertexAttribArray(1)

    # unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0)
	glBindVertexArray(0)

    # Extract color values and hardcode a other values
    newMaterial = Material(
        material.pbrMetallicRoughness.baseColorFactor[1:3],
        material.pbrMetallicRoughness.baseColorFactor[1:3],
        1.0,
        0.5,
        0.2,
        32
    )

    # create new mesh object
    return Mesh(vao, idxEBO, indices.count, Transform(), newMaterial)
end

"""
Write data to uniforms. This is to give the shaders information about how to render vertices. 

# Arguments 
- `program`: The shader program used.
- `transformMatrix::Matrix`: 4x4 matrix as transform matrix for model.
- `cam::Camera`: The camera object. 
- `ambientLightColor::Vector{GLfloat}`: The color of the ambient light (RGB). 
- `material::Material`: The material of the mesh to render. 
"""
function writeToUniforms(program, transformMatrix::Matrix, cam::Camera, ambientLightColor::Vector{GLfloat}, material::Material)
    # get uniform locations 
    modelMatrixLoc = glGetUniformLocation(program, "modelMatrix")
    viewMatrixLoc = glGetUniformLocation(program, "viewMatrix")
    projMatrixLoc = glGetUniformLocation(program, "projMatrix")
    camPositionLoc = glGetUniformLocation(program, "cameraPosition")
    ambientLightCoLoc = glGetUniformLocation(program, "ambientLightColor")

    glUseProgram(program)

    # write to uniforms
    glUniformMatrix4fv(modelMatrixLoc, 1, GL_FALSE, transformMatrix)
    glUniformMatrix4fv(viewMatrixLoc, 1, GL_FALSE, getViewMatrix(cam))
    glUniformMatrix4fv(projMatrixLoc, 1, GL_FALSE, getProjectionMatrix(cam))
    glUniform3fv(camPositionLoc, 1, cam.position)
    glUniform3fv(ambientLightCoLoc, 1, ambientLightColor)
    glUniform3fv(glGetUniformLocation(program, "material.diffuseColor"), 1, material.diffuseColor)
    glUniform3fv(glGetUniformLocation(program, "material.specularColor"), 1, material.specularColor)
    glUniform1f(glGetUniformLocation(program, "material.ka"), material.ka)
    glUniform1f(glGetUniformLocation(program, "material.kd"), material.kd)
    glUniform1f(glGetUniformLocation(program, "material.ks"), material.ks)
    glUniform1f(glGetUniformLocation(program, "material.shininess"), material.shininess)
end