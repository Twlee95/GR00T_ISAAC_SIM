"""Keep the dynamic reference prim separate from the XR anchor prim."""

def install():
    from isaaclab_teleop.xr_anchor_manager import XrAnchorManager

    if getattr(XrAnchorManager, "_sonic_independent_anchor_installed", False):
        return

    init_name = chr(95) * 2 + "init" + chr(95) * 2
    original_init = getattr(XrAnchorManager, init_name)

    def independent_init(self, xr_cfg):
        reference_path = xr_cfg.anchor_prim_path

        if reference_path is None:
            return original_init(self, xr_cfg)

        # Manager가 /World/XRAnchor를 만들도록 잠시 정적 모드로 생성한다.
        xr_cfg.anchor_prim_path = None
        try:
            original_init(self, xr_cfg)
        finally:
            # Synchronizer는 원래 pelvis를 동적 참조로 계속 사용한다.
            xr_cfg.anchor_prim_path = reference_path

        print(
            "[NEWTON14-XR] independent anchor: "
            f"reference={reference_path} "
            f"output={self.anchor_headset_path}",
            flush=True,
        )

    setattr(XrAnchorManager, init_name, independent_init)
    XrAnchorManager._sonic_independent_anchor_installed = True

    print(
        "[NEWTON14-XR] independent world anchor patch installed",
        flush=True,
    )
