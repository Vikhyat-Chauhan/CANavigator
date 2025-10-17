ros2 run hydra_teleop event_spawner_gz --ros-args \
  -p event_topic:=/hydra/event \
  -p world_name:=default \
  -p spawn_service:=/spawn_entity \
  -p delete_service:=/delete_entity \
  -p setpose_service:=/world/default/set_pose \
  -p z_level:=0.0 \
  -p enemy_animate:=true \
  -p enemy_update_rate_hz:=20.0 \
  -p persist_after_deadline:=false
