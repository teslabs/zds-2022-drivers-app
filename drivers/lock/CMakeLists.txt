zephyr_library()
zephyr_library_sources_ifdef(CONFIG_LOCK_SERVO servo.c)
