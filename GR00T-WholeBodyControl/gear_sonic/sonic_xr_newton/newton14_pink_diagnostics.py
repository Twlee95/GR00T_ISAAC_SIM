"""Pink IK frame, initialization and state freshness diagnostics."""

import torch


def _tensor(value):
    if hasattr(value, "torch"):
        value = value.torch
    return value.detach().float()


def _short(value, digits=4):
    return [round(float(x), digits) for x in value.flatten()]


def install():
    from isaaclab.envs.mdp.actions.pink_task_space_actions import (
        PinkInverseKinematicsAction,
    )

    if getattr(PinkInverseKinematicsAction, "_sonic_diag_installed", False):
        return

    original_process = PinkInverseKinematicsAction.process_actions
    original_apply = PinkInverseKinematicsAction.apply_actions

    def diagnostic_process(self, actions):
        count = getattr(self, "_sonic_diag_count", 0) + 1
        self._sonic_diag_count = count

        original_process(self, actions)

        # 입력 world pose와 pelvis 기준 변환 결과
        world_poses = self._extract_controlled_frame_poses(actions)
        base_positions, base_rotations = (
            self._transform_poses_to_base_link_frame(world_poses)
        )

        self._sonic_diag_world_poses = world_poses.detach().clone()
        self._sonic_diag_base_positions = base_positions.detach().clone()
        self._sonic_diag_base_rotations = base_rotations.detach().clone()

    def diagnostic_apply(self):
        data = self._asset.data

        q_before = _tensor(data.joint_pos)[0].clone()
        body_before = _tensor(data.body_link_pose_w)[0].clone()

        original_apply(self)

        controlled_ids = torch.as_tensor(
            self._controlled_joint_ids,
            device=q_before.device,
            dtype=torch.long,
        )

        target = _tensor(self._processed_actions)[0]
        current = q_before[controlled_ids]
        error = target - current

        count = self._sonic_diag_count
        previous_q = getattr(self, "_sonic_diag_previous_q", None)
        previous_body = getattr(self, "_sonic_diag_previous_body", None)

        q_change = (
            float(torch.linalg.vector_norm(q_before - previous_q))
            if previous_q is not None else 0.0
        )
        body_change = (
            float(torch.linalg.vector_norm(body_before - previous_body))
            if previous_body is not None else 0.0
        )

        # 첫 30프레임과 이후 60프레임마다 출력
        if count <= 30 or count % 60 == 0:
            abs_error = error.abs()
            top_values, top_indices = torch.topk(
                abs_error,
                k=min(8, len(abs_error)),
            )

            top = []
            for value, local_index in zip(top_values, top_indices):
                index = int(local_index)
                top.append(
                    (
                        self._controlled_joint_names[index],
                        round(float(current[index]), 4),
                        round(float(target[index]), 4),
                        round(float(error[index]), 4),
                    )
                )

            base = self.base_link_frame_in_world_rf[0]
            world_poses = self._sonic_diag_world_poses[:, 0]
            base_positions = self._sonic_diag_base_positions[:, 0]

            print(
                "[PINK-INIT] "
                f"step={count} "
                f"max_joint_error={float(abs_error.max()):.5f} "
                f"error_norm={float(torch.linalg.vector_norm(error)):.5f} "
                f"top=(name,current,target,error){top}",
                flush=True,
            )

            print(
                "[PINK-FRAME] "
                f"step={count} "
                f"base_world_pos={_short(base[:3, 3])} "
                f"target_world_pos="
                f"{[_short(pose[:3, 3]) for pose in world_poses]} "
                f"target_base_pos="
                f"{[_short(pos) for pos in base_positions]}",
                flush=True,
            )

            print(
                "[PINK-STATE] "
                f"step={count} "
                f"q_change={q_change:.8f} "
                f"body_pose_change={body_change:.8f}",
                flush=True,
            )

            if count == 1:
                for controller in self._ik_controllers:
                    for task in controller.cfg.variable_input_tasks:
                        task_info = {
                            key: value
                            for key, value in vars(task).items()
                            if isinstance(value, (str, int, float, bool))
                        }
                        print(
                            f"[PINK-TASK] {type(task).__name__} "
                            f"{task_info}",
                            flush=True,
                        )

        self._sonic_diag_previous_q = q_before
        self._sonic_diag_previous_body = body_before

    PinkInverseKinematicsAction.process_actions = diagnostic_process
    PinkInverseKinematicsAction.apply_actions = diagnostic_apply
    PinkInverseKinematicsAction._sonic_diag_installed = True

    print("[PINK-DIAG] diagnostics installed", flush=True)
