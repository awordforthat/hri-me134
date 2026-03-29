def lerp(a, b, t):
    """Linearly interpolate between a and b by t."""
    return a + (b - a) * t


def advance_keyframe_sequence(
    servos, keyframes=[], step_num=0, t=0, time_step=0.05, speed_factor=1
):

    total_duration = sum(kf["max_duration"] for kf in keyframes) / 1000 * speed_factor

    if total_duration <= 0:
        return True, 0

    cycle_t = t % total_duration

    elapsed = 0
    current_step_num = 0
    offset_t_in_current_step = 0

    for i, kf in enumerate(keyframes):
        step_duration = kf["max_duration"] / 1000 * speed_factor
        if cycle_t < elapsed + step_duration:
            current_step_num = i
            offset_t_in_current_step = cycle_t - elapsed
            break
        elapsed += step_duration
    else:
        # Fallback for floating point edge cases
        current_step_num = len(keyframes) - 1
        offset_t_in_current_step = keyframes[-1]["max_duration"] / 1000 * speed_factor

    current_keyframe = keyframes[current_step_num]
    prior_keyframe = keyframes[current_step_num - 1] if current_step_num > 0 else None

    current_step_duration = current_keyframe["max_duration"] / 1000 * speed_factor
    percent_through_current_step = (
        offset_t_in_current_step / current_step_duration
        if current_step_duration > 0
        else 1.0
    )

    # Clamp in case of tiny floating point weirdness
    percent_through_current_step = max(0.0, min(1.0, percent_through_current_step))

    for servo_name, target_angle in current_keyframe["servo_angles"].items():
        if prior_keyframe and servo_name in prior_keyframe["servo_angles"]:
            starting_angle = prior_keyframe["servo_angles"][servo_name]
        else:
            starting_angle = servos[servo_name]["servo"].get_physical_angle()

        interpolated_angle = lerp(
            starting_angle,
            target_angle,
            percent_through_current_step,
        )

        servos[servo_name]["servo"].move(
            int(interpolated_angle), int(time_step * 1000), wait=True
        )

    for servo_name, _ in servos.items():
        sv = servos[servo_name]["servo"]
        if sv._waiting_for_move:
            sv.move_start()

    return False, current_step_num
