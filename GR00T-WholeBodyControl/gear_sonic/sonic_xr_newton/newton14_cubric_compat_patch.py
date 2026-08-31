"""Use the safe Fabric hierarchy path for incompatible Cubric ABI."""

def install():
    from isaaclab_newton.physics.newton_manager import NewtonManager

    if getattr(NewtonManager, "_sonic_safe_fabric_installed", False):
        return

    original = NewtonManager.sync_transforms_to_usd.__func__

    def safe_sync(cls):
        # Installed Cubric plugin reports ABI v0.2 while the beta2
        # IsaacLab shim expects v0.1. Use update_world_xforms instead.
        cls._cubric = None
        cls._cubric_adapter = None
        return original(cls)

    NewtonManager.sync_transforms_to_usd = classmethod(safe_sync)
    NewtonManager._sonic_safe_fabric_installed = True

    print(
        "[NEWTON14] Cubric disabled; safe Fabric hierarchy enabled",
        flush=True,
    )
