set(CycloneDDS_FOUND TRUE)
add_library(CycloneDDS::ddsc SHARED IMPORTED)
set_target_properties(CycloneDDS::ddsc PROPERTIES
    IMPORTED_LOCATION /workspace/thirdparty/unitree_sdk2/thirdparty/lib/x86_64/libddsc.so.0
    INTERFACE_INCLUDE_DIRECTORIES /workspace/thirdparty/unitree_sdk2/thirdparty/include)
