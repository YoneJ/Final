class PoseManager:
    _instance = None
    _pose = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(PoseManager, cls).__new__(cls)
        return cls._instance

    def set_pose(self, pose):
        self._pose = pose

    def get_pose(self):
        return self._pose
