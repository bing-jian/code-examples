include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})
message(STATUS "PROTOBUF_INCLUDE_DIR ==> ${PROTOBUF_INCLUDE_DIR}")
protobuf_generate_cpp(PROTO_SRC PROTO_HEADER message.proto)

add_library(proto ${PROTO_HEADER} ${PROTO_SRC})
target_link_libraries(proto ${PROTOBUF_LIBRARIES})
target_include_directories(proto PUBLIC ${PROTOBUF_INCLUDE_DIR})
