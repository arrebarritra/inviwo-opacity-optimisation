# Inviwo Order Independent Transparceny (OIT) Module
ivw_module(OIT)

# Add header files
set(HEADER_FILES
    include/modules/oit/algorithm/calcnormals.h
    include/modules/oit/datastructures/halfedges.h
    include/modules/oit/datastructures/rasterization.h
    include/modules/oit/datastructures/transformedrasterization.h
    include/modules/oit/oitmodule.h
    include/modules/oit/oitmoduledefine.h
    include/modules/oit/ports/rasterizationport.h
    include/modules/oit/processors/calcnormalsprocessor.h
    include/modules/oit/processors/linerasterizer.h
    include/modules/oit/processors/meshrasterizer.h
    include/modules/oit/processors/rasterizationrenderer.h
    include/modules/oit/processors/rasterizer.h
    include/modules/oit/processors/sphererasterizer.h
    include/modules/oit/processors/transformrasterization.h
    include/modules/oit/rasterizeevent.h
    include/modules/oit/rendering/fragmentlistrenderer.h
)
ivw_group("Header Files" ${HEADER_FILES})

# Add source files
set(SOURCE_FILES
    src/algorithm/calcnormals.cpp
    src/datastructures/halfedges.cpp
    src/datastructures/rasterization.cpp
    src/datastructures/transformedrasterization.cpp
    src/oitmodule.cpp
    src/ports/rasterizationport.cpp
    src/processors/calcnormalsprocessor.cpp
    src/processors/linerasterizer.cpp
    src/processors/meshrasterizer.cpp
    src/processors/rasterizationrenderer.cpp
    src/processors/rasterizer.cpp
    src/processors/sphererasterizer.cpp
    src/processors/transformrasterization.cpp
    src/rasterizeevent.cpp
    src/rendering/fragmentlistrenderer.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

# Add shaders
set(SHADER_FILES
    glsl/illustration/display.frag
    glsl/illustration/illustrationbuffer.glsl
    glsl/illustration/neighbors.frag
    glsl/illustration/smooth.frag
    glsl/illustration/sortandfill.frag
    glsl/oit/abufferlinkedlist.glsl
    glsl/oit/clear.frag
    glsl/oit/commons.glsl
    glsl/oit/display.frag
    glsl/oit/simplequad.vert
    glsl/oit/sort.glsl
    glsl/fancymeshrenderer.frag
    glsl/fancymeshrenderer.geom
    glsl/fancymeshrenderer.vert
    glsl/oit-linerenderer.frag
    glsl/oit-sphereglyph.frag
)
ivw_group("Shader Files" ${SHADER_FILES})

# Add Unittests
set(TEST_FILES
    tests/unittests/compresscolor-test.cpp
    tests/unittests/halfedges-test.cpp
    tests/unittests/oit-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to pack
ivw_add_to_module_pack(glsl)

