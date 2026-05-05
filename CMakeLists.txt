find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/act.msg"
    "msg/obs.msg"
    "msg/recv.msg"
    "srv/init_sim.srv"
)