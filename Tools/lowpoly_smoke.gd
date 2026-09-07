extends SceneTree

var failures: Array[String] = []
var scene: Node

func _initialize() -> void:
    call_deferred("run")

func check(ok: bool, message: String) -> void:
    if not ok:
        failures.append(message)
        push_error(message)

func press(code: Key) -> void:
    var event := InputEventKey.new()
    event.keycode = code
    event.pressed = true
    Input.parse_input_event(event)
    var released := InputEventKey.new()
    released.keycode = code
    Input.parse_input_event(released)

func capture(name: String) -> void:
    await RenderingServer.frame_post_draw
    var image := root.get_texture().get_image()
    check(not image.is_empty(), "Empty viewport: " + name)
    var path := ProjectSettings.globalize_path("res://.tmp/lowpoly/" + name + ".png")
    check(image.save_png(path) == OK, "Could not save " + path)
    print("CAPTURE ", path)

func run() -> void:
    DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path("res://.tmp/lowpoly"))
    root.size = Vector2i(1440, 900)
    scene = load("res://Levels/lowpoly.tscn").instantiate()
    root.add_child(scene)
    current_scene = scene
    var loading_started := Time.get_ticks_msec()
    while scene.get_node_or_null("CameraRig/Lens") == null and Time.get_ticks_msec()-loading_started < 120000:
        await process_frame
    await process_frame
    var cars := scene.get_children().filter(func(child): return child.name.begins_with("Car_"))
    check(cars.size() == 1, "Default practice must contain exactly one car")
    check(scene.get("SelectedCarIndex") == 0, "Follow camera must select the only car")
    check(not scene.get("IsPaused"), "Solo practice should start automatically")
    await create_timer(1.0).timeout
    check(scene.get("RaceSeconds") > 0, "Default practice must advance automatically")
    press(KEY_SPACE)
    await create_timer(1.0).timeout
    var frozen: float = scene.get("RaceSeconds")
    await create_timer(0.3).timeout
    check(is_equal_approx(frozen, scene.get("RaceSeconds")), "Pause must stop Core race time")
    var paused_started := Time.get_ticks_msec()
    var paused_frames := 0
    while Time.get_ticks_msec()-paused_started < 2000:
        await process_frame
        paused_frames += 1
    print("SMOKE paused frames=", paused_frames, " over ms=", Time.get_ticks_msec()-paused_started)
    await capture("01-follow")
    for mode in [2, 3]:
        press(KEY_2 if mode == 2 else KEY_3)
        await create_timer(0.3).timeout
        check(scene.get("CameraMode") == mode, "Camera shortcut did not switch")
        await capture("02-aerial" if mode == 2 else "03-high-side")
    press(KEY_1)
    await process_frame
    press(KEY_RIGHT)
    await create_timer(0.3).timeout
    check(scene.get("SelectedCarIndex") == 0, "Single-car selection must remain valid")
    var camera: Camera3D = scene.get_node("CameraRig/Lens")
    var size_before := camera.size
    var wheel := InputEventMouseButton.new()
    wheel.button_index = MOUSE_BUTTON_WHEEL_UP
    wheel.pressed = true
    Input.parse_input_event(wheel)
    await create_timer(0.5).timeout
    check(camera.size < size_before, "Wheel did not zoom camera")
    root.size = Vector2i(1024, 640)
    await create_timer(0.3).timeout
    await capture("04-compact")
    root.size = Vector2i(1440, 900)
    press(KEY_SPACE)
    var start := Time.get_ticks_msec()
    var frame_count := 0
    while Time.get_ticks_msec() - start < 6000:
        await process_frame
        frame_count += 1
    check(scene.get("RaceSeconds") > frozen, "Resume must advance simulation")
    await capture("05-running")
    print("SMOKE rendered frames=", frame_count, " over ms=", Time.get_ticks_msec()-start, " simulation_seconds=", scene.get("RaceSeconds"))
    press(KEY_SPACE)
    await create_timer(1.0).timeout
    var stopped: float = scene.get("RaceSeconds")
    await create_timer(0.4).timeout
    check(is_equal_approx(stopped,scene.get("RaceSeconds")), "Pause must settle an in-flight worker and stop further steps")
    print("SMOKE ", "PASS" if failures.is_empty() else "FAIL", " errors=", failures)
    quit(0 if failures.is_empty() else 1)
