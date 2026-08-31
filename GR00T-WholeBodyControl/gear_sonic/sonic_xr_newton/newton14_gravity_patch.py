"""Newton 1.4 gravity compensation bridge for Isaac Lab beta2."""

import warp as wp

from isaaclab.utils.warp import ProxyArray
from isaaclab_newton.assets.articulation.articulation_data import (
    ArticulationData,
    SimulationManager,
)


@wp.kernel
def _gather_articulation_gravity(
    full_gravity: wp.array(dtype=wp.float32),
    articulation_ids: wp.array(dtype=wp.int32),
    articulation_start: wp.array(dtype=wp.int32),
    joint_qd_start: wp.array(dtype=wp.int32),
    output: wp.array2d(dtype=wp.float32),
):
    instance, dof = wp.tid()

    articulation_id = articulation_ids[instance]
    first_joint = articulation_start[articulation_id]
    first_dof = joint_qd_start[first_joint]

    output[instance, dof] = full_gravity[first_dof + dof]


def _gravity_compensation_forces(self):
    self._ensure_fk_fresh()

    model = SimulationManager.get_model()
    state = SimulationManager.get_state_0()

    if not hasattr(self, "_gravity_force_full_buf"):
        self._gravity_force_full_buf = wp.zeros(
            model.joint_dof_count,
            dtype=wp.float32,
            device=self.device,
        )

        # Floating-base G1: 6 root DoFs + 43 actuated joints.
        dof_count = self._mass_matrix_buf.shape[1]

        self._gravity_force_view_buf = wp.zeros(
            (self._num_instances, dof_count),
            dtype=wp.float32,
            device=self.device,
        )
        self._gravity_compensation_forces_ta = ProxyArray(
            self._gravity_force_view_buf
        )

        print(
            f"[NEWTON14] gravity output shape="
            f"{self._gravity_force_view_buf.shape}",
            flush=True,
        )

    self._root_view.eval_inverse_dynamics_passive(
        state,
        gravity_force=self._gravity_force_full_buf,
    )

    wp.launch(
        _gather_articulation_gravity,
        dim=self._gravity_force_view_buf.shape,
        inputs=[
            self._gravity_force_full_buf,
            self._jacobian_view_art_ids,
            model.articulation_start,
            model.joint_qd_start,
        ],
        outputs=[self._gravity_force_view_buf],
        device=self.device,
    )

    return self._gravity_compensation_forces_ta


def install():
    if getattr(ArticulationData, "_newton14_gravity_installed", False):
        return

    ArticulationData.gravity_compensation_forces = property(
        _gravity_compensation_forces
    )
    ArticulationData._newton14_gravity_installed = True

    print(
        "[NEWTON14] gravity compensation bridge installed",
        flush=True,
    )
